import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

LEVEL_COLORS = {'restricted': (1.0, 0.15, 0.15), 'watch': (1.0, 0.8, 0.0)}


def point_in_polygon(x, y, polygon):
    """Ray casting: count how many polygon edges a ray to the right of (x, y) crosses."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ZoneMonitor(Node):
    def __init__(self):
        super().__init__('zone_monitor')

        default_zones = str(Path(get_package_share_directory('security_robot_security')) / 'config' / 'zones.yaml')
        self.declare_parameter('zones_file', default_zones)
        self.declare_parameter('locations_topic', '/perception/person_locations')
        self.declare_parameter('enter_delay_s', 1.0)       # inside this long before "entered"
        self.declare_parameter('exit_delay_s', 2.0)        # outside this long before "exited"
        self.declare_parameter('person_timeout_s', 3.0)    # not reported this long = no longer visible

        gp = self.get_parameter
        self.enter_delay = float(gp('enter_delay_s').value)
        self.exit_delay = float(gp('exit_delay_s').value)
        self.person_timeout = float(gp('person_timeout_s').value)

        self.load_zones(gp('zones_file').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_frame = self.zone_frame
        self.warned_no_tf = False

        self.people = {}        # id -> {'x', 'y', 't'} in the zone frame
        self.state = {}         # (id, zone) -> {'inside', 'in_since', 'out_since'}
        self.last_alert = {}    # (id, zone) -> time of last alert

        self.create_subscription(Detection3DArray, gp('locations_topic').value, self.locations_callback, 10)
        self.events_pub = self.create_publisher(String, '/security/zone_events', 10)
        self.count_pub = self.create_publisher(Int32, '/security/intrusion_count', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/security/zone_markers', 10)

        self.create_timer(0.5, self.evaluate)
        self.create_timer(1.0, self.publish_markers)

        names = ', '.join(f"{z['name']} ({z['level']})" for z in self.zones)
        self.get_logger().info(f'Zone monitor ready with {len(self.zones)} zones: {names}')

    # ---------- setup ----------

    def load_zones(self, path):
        data = yaml.safe_load(Path(path).read_text())
        self.zone_frame = data.get('frame_id', 'map')
        self.alert_cooldown = float(data.get('alert_cooldown_s', 60.0))
        self.zones = []
        for z in data.get('zones', []):
            polygon = [(float(p[0]), float(p[1])) for p in z['polygon']]
            if len(polygon) < 3:
                raise ValueError(f"Zone {z.get('name')} needs at least 3 points")
            level = z.get('level', 'restricted')
            if level not in LEVEL_COLORS:
                raise ValueError(f"Zone {z['name']}: level must be one of {list(LEVEL_COLORS)}")
            self.zones.append({'name': z['name'], 'level': level, 'polygon': polygon})
        if not self.zones:
            raise ValueError(f'No zones defined in {path}')

    # ---------- inputs ----------

    def to_zone_frame(self, x, y, src_frame):
        if src_frame == self.zone_frame:
            self.marker_frame = self.zone_frame
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(self.zone_frame, src_frame, Time())
        except Exception:
            # No map frame (Nav2/AMCL not running). In this world the map and odom frames both start at the
            # robot's spawn point, so odom coordinates are a good stand-in for map coordinates.
            if not self.warned_no_tf:
                self.get_logger().warn(
                    f"No '{src_frame}' -> '{self.zone_frame}' transform (Nav2 not running?) - "
                    f"using {src_frame} coordinates for the zones")
                self.warned_no_tf = True
            self.marker_frame = src_frame
            return x, y
        self.marker_frame = self.zone_frame
        yaw = yaw_from_quaternion(tf.transform.rotation)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        return (tx + x * math.cos(yaw) - y * math.sin(yaw),
                ty + x * math.sin(yaw) + y * math.cos(yaw))

    def locations_callback(self, msg):
        now = time.monotonic()
        for det in msg.detections:
            x, y = self.to_zone_frame(det.bbox.center.position.x, det.bbox.center.position.y,
                                      msg.header.frame_id)
            self.people[det.id] = {'x': x, 'y': y, 't': now}

    # ---------- logic ----------

    def emit(self, event, pid, zone, person, extra=''):
        payload = {
            'event': event,
            'person_id': int(pid) if str(pid).isdigit() else pid,
            'zone': zone['name'],
            'level': zone['level'],
            'x': round(person['x'], 2),
            'y': round(person['y'], 2),
            'time': time.time(),
        }
        self.events_pub.publish(String(data=json.dumps(payload)))

        where = f"({person['x']:.1f}, {person['y']:.1f})"
        key = (pid, zone['name'])
        if event == 'zone_entered':
            recent = time.monotonic() - self.last_alert.get(key, -1e9) < self.alert_cooldown
            if zone['level'] == 'restricted' and not recent:
                self.last_alert[key] = time.monotonic()
                self.get_logger().warn(f"[ALERT] Person #{pid} entered restricted zone '{zone['name']}' at {where}")
            else:
                note = ' (already alerted recently)' if zone['level'] == 'restricted' else ''
                self.get_logger().info(f"Person #{pid} is in {zone['level']} zone '{zone['name']}' at {where}{note}")
        elif event == 'zone_exited':
            self.get_logger().info(f"Person #{pid} left zone '{zone['name']}'")
        elif event == 'lost_sight':
            self.get_logger().info(f"Person #{pid} no longer visible (last seen in '{zone['name']}'){extra}")

    def evaluate(self):
        now = time.monotonic()
        for pid in list(self.people):
            person = self.people[pid]
            present = now - person['t'] <= self.person_timeout

            for zone in self.zones:
                key = (pid, zone['name'])
                st = self.state.setdefault(key, {'inside': False, 'in_since': None, 'out_since': None})

                if not present:
                    if st['inside']:
                        st['inside'] = False
                        self.emit('lost_sight', pid, zone, person)
                    st['in_since'] = st['out_since'] = None
                    continue

                if point_in_polygon(person['x'], person['y'], zone['polygon']):
                    st['out_since'] = None
                    if st['in_since'] is None:
                        st['in_since'] = now
                    if not st['inside'] and now - st['in_since'] >= self.enter_delay:
                        st['inside'] = True
                        self.emit('zone_entered', pid, zone, person)
                else:
                    st['in_since'] = None
                    if st['inside']:
                        if st['out_since'] is None:
                            st['out_since'] = now
                        if now - st['out_since'] >= self.exit_delay:
                            st['inside'] = False
                            self.emit('zone_exited', pid, zone, person)

            if not present:
                del self.people[pid]

        intrusions = sum(
            1 for (pid, zname), st in self.state.items()
            if st['inside'] and any(z['name'] == zname and z['level'] == 'restricted' for z in self.zones)
        )
        self.count_pub.publish(Int32(data=intrusions))

    # ---------- RViz ----------

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        mid = 0
        for zone in self.zones:
            r, g, b = LEVEL_COLORS[zone['level']]

            outline = Marker()
            outline.header.frame_id = self.marker_frame
            outline.header.stamp = stamp
            outline.ns, outline.id = 'zones', mid
            outline.type, outline.action = Marker.LINE_STRIP, Marker.ADD
            outline.scale.x = 0.06
            outline.color.r, outline.color.g, outline.color.b, outline.color.a = r, g, b, 1.0
            outline.pose.orientation.w = 1.0
            for px, py in zone['polygon'] + [zone['polygon'][0]]:
                outline.points.append(Point(x=px, y=py, z=0.05))
            markers.markers.append(outline)
            mid += 1

            label = Marker()
            label.header = outline.header
            label.ns, label.id = 'zone_labels', mid
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            cx = sum(p[0] for p in zone['polygon']) / len(zone['polygon'])
            cy = sum(p[1] for p in zone['polygon']) / len(zone['polygon'])
            label.pose.position.x, label.pose.position.y, label.pose.position.z = cx, cy, 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
            label.text = f"{zone['name']} [{zone['level']}]"
            markers.markers.append(label)
            mid += 1

        for pid, person in self.people.items():
            in_restricted = any(
                self.state.get((pid, z['name']), {}).get('inside') and z['level'] == 'restricted'
                for z in self.zones
            )
            body = Marker()
            body.header.frame_id = self.marker_frame
            body.header.stamp = stamp
            body.ns, body.id = 'people', mid
            body.type, body.action = Marker.CYLINDER, Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = person['x'], person['y'], 0.85
            body.pose.orientation.w = 1.0
            body.scale.x, body.scale.y, body.scale.z = 0.4, 0.4, 1.7
            if in_restricted:
                body.color.r, body.color.g, body.color.b = 1.0, 0.1, 0.1
            else:
                body.color.r, body.color.g, body.color.b = 0.1, 0.9, 0.2
            body.color.a = 0.7
            markers.markers.append(body)
            mid += 1

            tag = Marker()
            tag.header = body.header
            tag.ns, tag.id = 'people_labels', mid
            tag.type, tag.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = person['x'], person['y'], 2.0
            tag.pose.orientation.w = 1.0
            tag.scale.z = 0.3
            tag.color.r = tag.color.g = tag.color.b = tag.color.a = 1.0
            tag.text = f'Person #{pid}' + (' INTRUDER' if in_restricted else '')
            markers.markers.append(tag)
            mid += 1

        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ZoneMonitor()
    except Exception as e:
        rclpy.logging.get_logger('zone_monitor').error(f'Could not start: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
