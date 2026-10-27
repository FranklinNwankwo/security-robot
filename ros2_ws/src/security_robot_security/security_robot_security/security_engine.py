import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class SecurityEngine(Node):
    """PATROL -> INVESTIGATE -> OBSERVE -> (SEARCH) -> back to PATROL."""

    def __init__(self):
        super().__init__('security_engine')

        default_route = str(Path(get_package_share_directory('security_robot_patrol')) / 'config' / 'patrol_route.yaml')
        self.declare_parameter('route_file', default_route)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff_m', 2.0)          # stop this far from the intruder (camera sees a whole person)
        self.declare_parameter('retarget_m', 0.7)          # re-plan if the intruder moves this far (following)
        self.declare_parameter('lost_timeout_s', 5.0)      # not seen this long -> SEARCH
        self.declare_parameter('observe_max_s', 45.0)      # give up observing after this long
        self.declare_parameter('search_timeout_s', 40.0)
        self.declare_parameter('ignore_after_s', 120.0)    # after giving up on someone, ignore them this long
        self.declare_parameter('max_goal_cost', 50)        # 0-100 costmap value allowed at a goal spot
        self.declare_parameter('handover_radius_m', 1.0)   # someone this close to the target's spot IS the target

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.standoff = float(gp('standoff_m').value)
        self.retarget = float(gp('retarget_m').value)
        self.lost_timeout = float(gp('lost_timeout_s').value)
        self.observe_max = float(gp('observe_max_s').value)
        self.search_timeout = float(gp('search_timeout_s').value)
        self.ignore_after = float(gp('ignore_after_s').value)
        self.max_goal_cost = int(gp('max_goal_cost').value)
        self.handover_radius = float(gp('handover_radius_m').value)

        self.load_route(gp('route_file').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.nav_manage_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.nav_active = False
        self.nav_check_pending = False
        self.nav_down_since = None
        self.last_nav_recovery = 0.0

        self.create_subscription(String, '/security/zone_events', self.zone_event_callback, 50)
        self.create_subscription(Detection3DArray, '/perception/person_locations', self.locations_callback, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 1)
        self.state_pub = self.create_publisher(String, '/security/state', 10)

        # --- state ---
        self.state = 'STARTUP'
        self.detail = 'waiting for Nav2'
        self.last_state_pub = 0.0
        self.last_wait_log = 0.0

        self.intruders = {}       # person id -> {'zone', 'x', 'y', 't'}
        self.locations = {}       # person id -> (x, y, t) in the map frame
        self.ignored_until = {}   # person id -> time
        self.ignored_spots = []   # (x, y, until): places where an intruder was already handled
        self.costmap = None

        self.target_id = None
        self.target_zone = None
        self.target_last_seen = 0.0
        self.goal_person_pos = None
        self.approach_attempts = 0
        self.observe_since = 0.0
        self.search_since = 0.0
        self.search_phase = None
        self.search_yaws = []

        self.patrol_index = 0
        self.patrol_pause_until = 0.0

        # --- Nav2 goal bookkeeping ---
        self.goal_seq = 0
        self.goal_handle = None
        self.goal_purpose = None
        self.goal_result = None    # None / 'succeeded' / 'aborted' / 'canceled' / 'rejected'
        self.nav_busy = False
        self.last_goal_time = 0.0

        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.check_nav)
        names = ' -> '.join(w['name'] for w in self.waypoints)
        self.get_logger().info(f'Security engine ready. Patrol route: {names}{" (looping)" if self.loop else ""}')

    # ================= setup =================

    def load_route(self, path):
        data = yaml.safe_load(Path(path).read_text())
        self.route_frame = data.get('frame_id', 'map')
        self.loop = bool(data.get('loop', True))
        self.pause_seconds = float(data.get('pause_seconds', 0.0))
        self.waypoints = data['waypoints']
        if not self.waypoints:
            raise ValueError(f'No waypoints in {path}')

    # ================= inputs =================

    def zone_event_callback(self, msg):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        pid = str(ev.get('person_id'))
        now = time.monotonic()
        if ev.get('event') == 'zone_entered' and ev.get('level') == 'restricted':
            self.intruders[pid] = {'zone': ev['zone'], 'x': ev['x'], 'y': ev['y'], 't': now}
        elif ev.get('event') == 'zone_exited':
            info = self.intruders.get(pid)
            if info and info['zone'] == ev.get('zone'):
                del self.intruders[pid]

    def locations_callback(self, msg):
        now = time.monotonic()
        for det in msg.detections:
            xy = self.to_map(det.bbox.center.position.x, det.bbox.center.position.y, msg.header.frame_id)
            if xy is not None:
                self.locations[det.id] = (xy[0], xy[1], now)

    def costmap_callback(self, msg):
        self.costmap = msg

    # ================= geometry =================

    def to_map(self, x, y, frame):
        if frame == self.map_frame:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
        except Exception:
            return None
        yaw = yaw_from_quaternion(tf.transform.rotation)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        return (tx + x * math.cos(yaw) - y * math.sin(yaw), ty + x * math.sin(yaw) + y * math.cos(yaw))

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y, yaw_from_quaternion(tf.transform.rotation)

    def cost_at(self, x, y):
        """Costmap value (0-100, -1 unknown) at (x, y), or None if there is no costmap / outside it."""
        cm = self.costmap
        if cm is None:
            return None
        info = cm.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return -1
        return cm.data[my * info.width + mx]

    def is_free(self, x, y):
        c = self.cost_at(x, y)
        return c is None or 0 <= c <= self.max_goal_cost

    def line_of_sight(self, gx, gy, px, py):
        """No wall (lethal cell) between the goal and the person; ignores the last 0.5 m (the person's own legs)."""
        if self.costmap is None:
            return True
        dist = math.hypot(px - gx, py - gy)
        step = self.costmap.info.resolution
        n = int(max(0.0, dist - 0.5) / step)
        for i in range(n):
            t = i * step / max(dist, 1e-6)
            c = self.cost_at(gx + t * (px - gx), gy + t * (py - gy))
            if c is not None and (c >= 100 or c < 0):
                return False
        return True

    def approach_goal(self, person):
        """A free spot about `standoff` from the person, with a clear view of them, as close to the robot as possible."""
        robot = self.robot_pose()
        if robot is None:
            return None
        px, py = person
        candidates = []
        for radius in (self.standoff, self.standoff - 0.5, self.standoff + 0.5):
            for k in range(12):
                a = 2.0 * math.pi * k / 12.0
                gx, gy = px + radius * math.cos(a), py + radius * math.sin(a)
                if self.is_free(gx, gy) and self.line_of_sight(gx, gy, px, py):
                    candidates.append((math.hypot(gx - robot[0], gy - robot[1]), gx, gy))
            if candidates:
                break
        if candidates:
            _, gx, gy = min(candidates)
        else:
            dx, dy = robot[0] - px, robot[1] - py
            n = math.hypot(dx, dy)
            if n < 1e-3:
                return None
            gx, gy = px + dx / n * self.standoff, py + dy / n * self.standoff
        return gx, gy, math.atan2(py - gy, px - gx)

    # ================= Nav2 goals =================

    def send_goal(self, x, y, yaw, purpose):
        if not self.nav_client.server_is_ready():
            return False
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_purpose = purpose
        self.goal_result = None
        self.nav_busy = True
        self.last_goal_time = time.monotonic()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.on_goal_response(f, seq))
        return True

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:                 # a newer goal replaced this one
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.goal_result = 'rejected'
            self.nav_busy = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return
        status = future.result().status
        self.goal_result = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                            GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, 'aborted')
        self.nav_busy = False
        self.goal_handle = None

    def cancel_goal(self):
        self.goal_seq += 1                       # results of the old goal are ignored from now on
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.nav_busy = False
        self.goal_result = None
        self.goal_purpose = None

    # ================= state helpers =================

    def set_state(self, state, detail):
        changed = state != self.state or detail != self.detail
        self.state, self.detail = state, detail
        if changed:
            self.get_logger().info(f'[{state}] {detail}')
            self.publish_state(force=True)

    def publish_state(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_state_pub < 2.0:
            return
        self.last_state_pub = now
        payload = {
            'state': self.state,
            'detail': self.detail,
            'target_person': self.target_id,
            'target_zone': self.target_zone,
            'patrol_waypoint': self.waypoints[self.patrol_index]['name'] if self.waypoints else None,
            'time': time.time(),
        }
        self.state_pub.publish(String(data=json.dumps(payload)))

    def target_position(self, now):
        """Fresh position of the current target, or None if not seen recently."""
        loc = self.locations.get(self.target_id)
        if loc is not None and now - loc[2] <= 1.5:
            self.target_last_seen = loc[2]
            return loc[0], loc[1]
        return None

    def last_known_target(self):
        loc = self.locations.get(self.target_id)
        if loc is not None:
            return loc[0], loc[1]
        info = self.intruders.get(self.target_id)
        return (info['x'], info['y']) if info else None

    def pick_target(self, now):
        candidates = [pid for pid in self.intruders
                      if self.ignored_until.get(pid, 0.0) < now and not self.near_ignored_spot(pid, now)]
        if not candidates:
            return None
        return max(candidates, key=lambda pid: self.locations.get(pid, (0, 0, self.intruders[pid]['t']))[2])

    def find_handover(self, ref, now):
        """Another person ID standing where the target was: the tracker renumbered them - same intruder."""
        if ref is None:
            return None
        best, best_d = None, self.handover_radius
        for pid, (x, y, t) in self.locations.items():
            if pid == self.target_id or now - t > 1.5:
                continue
            d = math.hypot(x - ref[0], y - ref[1])
            if d <= best_d:
                best, best_d = pid, d
        return best

    def try_handover(self, now):
        new = self.find_handover(self.last_known_target(), now)
        if new is None:
            return False
        old = self.target_id
        info = self.intruders.get(old) or {'zone': self.target_zone, 'x': 0.0, 'y': 0.0, 't': now}
        self.intruders.setdefault(new, dict(info))
        self.intruders.pop(old, None)
        self.target_id = new
        self.target_last_seen = now
        self.get_logger().info(f'Person #{old} is now tracked as Person #{new} (same spot) - same intruder, continuing')
        return True

    def near_ignored_spot(self, pid, now):
        self.ignored_spots = [spot for spot in self.ignored_spots if spot[2] > now]
        loc = self.locations.get(pid)
        info = self.intruders.get(pid)
        pos = (loc[0], loc[1]) if loc else ((info['x'], info['y']) if info else None)
        if pos is None:
            return False
        return any(math.hypot(pos[0] - sx, pos[1] - sy) <= self.handover_radius for sx, sy, _ in self.ignored_spots)

    def begin_investigation(self, pid, now):
        self.target_id = pid
        self.target_zone = self.intruders[pid]['zone']
        self.target_last_seen = now
        self.goal_person_pos = None
        self.approach_attempts = 0
        self.cancel_goal()
        self.get_logger().warn(f"Intruder Person #{pid} in '{self.target_zone}': pausing patrol to investigate")
        self.set_state('INVESTIGATE', f"approaching Person #{pid} in '{self.target_zone}'")

    def begin_search(self, now):
        last = self.last_known_target()
        if last is None:
            self.resume('intruder lost and no last known position')
            return
        self.search_since = now
        self.search_phase = 'goto'
        self.search_yaws = []
        self.cancel_goal()
        goal = self.approach_goal(last)
        if goal:
            self.send_goal(*goal, purpose='search')
        self.set_state('SEARCH', f'Person #{self.target_id} lost - checking last known position ({last[0]:.1f}, {last[1]:.1f})')

    def resume(self, reason, ignore=False):
        if self.target_id is not None and ignore:
            self.ignored_until[self.target_id] = time.monotonic() + self.ignore_after
            spot = self.last_known_target()
            if spot is not None:
                self.ignored_spots.append((spot[0], spot[1], time.monotonic() + self.ignore_after))
        self.get_logger().info(f'Resuming patrol: {reason}')
        self.target_id = None
        self.target_zone = None
        self.cancel_goal()
        self.set_state('PATROL', f"resuming at '{self.waypoints[self.patrol_index]['name']}'")

    def send_patrol_goal(self):
        wp = self.waypoints[self.patrol_index]
        if self.send_goal(wp['x'], wp['y'], math.radians(float(wp.get('yaw_deg', 0.0))), purpose='patrol'):
            self.set_state('PATROL', f"heading to '{wp['name']}' ({self.patrol_index + 1}/{len(self.waypoints)})")

    # ================= Nav2 health =================

    def check_nav(self):
        """Is Nav2's navigation actually ACTIVE? (Its action server can exist while it is shut down.)"""
        if self.nav_check_pending or not self.nav_state_client.service_is_ready():
            return
        self.nav_check_pending = True
        self.nav_state_client.call_async(GetState.Request()).add_done_callback(self.on_nav_state)

    def on_nav_state(self, future):
        self.nav_check_pending = False
        try:
            state = future.result().current_state
            active, label = state.id == 3, state.label
        except Exception:
            active, label = False, 'unknown'
        now = time.monotonic()
        if active:
            if not self.nav_active:
                self.get_logger().info('Nav2 navigation is active')
            self.nav_active = True
            self.nav_down_since = None
            return
        if self.nav_active or self.nav_down_since is None:
            self.get_logger().warn(f"Nav2 navigation is not active (state: {label}) - pausing decisions")
        self.nav_active = False
        if self.nav_down_since is None:
            self.nav_down_since = now
        if now - self.nav_down_since > 10.0 and now - self.last_nav_recovery > 30.0:
            self.recover_nav()

    def recover_nav(self):
        if not self.nav_manage_client.service_is_ready():
            return
        self.last_nav_recovery = time.monotonic()
        self.get_logger().warn('Restarting Nav2 navigation (RESET + STARTUP)')
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.RESET
        self.nav_manage_client.call_async(req).add_done_callback(lambda f: self.startup_nav())

    def startup_nav(self):
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        self.nav_manage_client.call_async(req)

    # ================= main loop =================

    def tick(self):
        now = time.monotonic()
        if self.state != 'STARTUP' and not self.nav_active:
            if self.goal_result == 'rejected':
                self.goal_result = None      # not the waypoint's fault - Nav2 is down
            if now - self.last_wait_log > 5.0:
                self.get_logger().warn(f'[{self.state}] paused - waiting for Nav2 navigation to become active')
                self.last_wait_log = now
            return
        if self.state == 'STARTUP':
            if self.nav_active and self.nav_client.server_is_ready() and self.robot_pose() is not None:
                self.set_state('PATROL', 'Nav2 ready - starting patrol')
                self.send_patrol_goal()
            elif now - self.last_wait_log > 5.0:
                self.get_logger().info('Waiting for Nav2 (navigate_to_pose) and localization (map -> base_link)...')
                self.last_wait_log = now
            return

        getattr(self, f'tick_{self.state.lower()}')(now)
        self.publish_state()

    def tick_patrol(self, now):
        target = self.pick_target(now)
        if target is not None:
            self.begin_investigation(target, now)
            return
        if self.nav_busy or now < self.patrol_pause_until:
            return
        if self.goal_purpose == 'patrol' and self.goal_result == 'succeeded':
            self.get_logger().info(f"Reached '{self.waypoints[self.patrol_index]['name']}'")
            self.advance_patrol()
            self.patrol_pause_until = now + self.pause_seconds
            return
        if self.goal_purpose == 'patrol' and self.goal_result in ('aborted', 'rejected'):
            self.get_logger().warn(f"Could not reach '{self.waypoints[self.patrol_index]['name']}' - skipping")
            self.advance_patrol()
            return
        if not self.loop and self.patrol_index >= len(self.waypoints):
            return
        self.send_patrol_goal()

    def advance_patrol(self):
        self.goal_result = None
        self.goal_purpose = None
        self.patrol_index += 1
        if self.patrol_index >= len(self.waypoints) and self.loop:
            self.patrol_index = 0

    def tick_investigate(self, now):
        if self.target_id not in self.intruders:
            self.resume(f'Person #{self.target_id} left the restricted zone')
            return
        pos = self.target_position(now)
        if pos is None:
            if self.try_handover(now):
                return
            if now - self.target_last_seen > self.lost_timeout:
                self.begin_search(now)
                return
            pos = self.last_known_target()
            if pos is None:
                return

        if not self.nav_busy and self.goal_purpose == 'approach' and self.goal_result == 'succeeded':
            self.observe_since = now
            self.goal_result = None
            self.set_state('OBSERVE', f"watching Person #{self.target_id} in '{self.target_zone}'")
            return

        need_goal = self.goal_person_pos is None
        if self.goal_person_pos is not None and now - self.last_goal_time > 2.0:
            moved = math.hypot(pos[0] - self.goal_person_pos[0], pos[1] - self.goal_person_pos[1])
            if moved > self.retarget:
                self.get_logger().info(f'Person #{self.target_id} moved {moved:.1f} m - following')
                need_goal = True
        if not self.nav_busy and self.goal_purpose == 'approach' and self.goal_result in ('aborted', 'rejected'):
            self.approach_attempts += 1
            self.goal_result = None
            if self.approach_attempts >= 3:
                self.get_logger().warn('Cannot reach a spot near the intruder - observing from here')
                self.observe_since = now
                self.set_state('OBSERVE', f"watching Person #{self.target_id} from a distance")
                return
            need_goal = True
        if not self.nav_busy and self.goal_result is None and self.goal_purpose != 'approach':
            need_goal = True

        if need_goal:
            goal = self.approach_goal(pos)
            if goal and self.send_goal(*goal, purpose='approach'):
                self.goal_person_pos = pos
                robot = self.robot_pose()
                dist = math.hypot(pos[0] - robot[0], pos[1] - robot[1]) if robot else float('nan')
                self.set_state('INVESTIGATE',
                               f"approaching Person #{self.target_id} in '{self.target_zone}' ({dist:.1f} m away)")

    def tick_observe(self, now):
        if self.target_id not in self.intruders:
            self.resume(f'Person #{self.target_id} left the restricted zone')
            return
        if now - self.observe_since > self.observe_max:
            self.get_logger().warn(
                f"Person #{self.target_id} still in '{self.target_zone}' after {self.observe_max:.0f} s - "
                f"logged as unresolved, returning to patrol")
            self.resume('observation time limit reached', ignore=True)
            return
        pos = self.target_position(now)
        if pos is None:
            if self.try_handover(now):
                return
            if now - self.target_last_seen > self.lost_timeout:
                self.begin_search(now)
            return
        if self.goal_person_pos is not None:
            moved = math.hypot(pos[0] - self.goal_person_pos[0], pos[1] - self.goal_person_pos[1])
            if moved > self.retarget:
                self.get_logger().info(f'Person #{self.target_id} moved {moved:.1f} m - following')
                self.goal_person_pos = None
                self.cancel_goal()
                self.set_state('INVESTIGATE', f"following Person #{self.target_id}")
                return
        robot = self.robot_pose()
        if robot and not self.nav_busy and now - self.last_goal_time > 3.0:
            want = math.atan2(pos[1] - robot[1], pos[0] - robot[0])
            if abs(wrap_angle(want - robot[2])) > 0.5:      # keep the person in front of the camera
                self.send_goal(robot[0], robot[1], want, purpose='face')

    def tick_search(self, now):
        if self.target_id not in self.intruders:
            self.resume(f'Person #{self.target_id} left the restricted zone')
            return
        if self.target_position(now) is None:
            self.try_handover(now)
        if self.target_position(now) is not None:
            self.get_logger().info(f'Person #{self.target_id} found again')
            self.goal_person_pos = None
            self.cancel_goal()
            self.set_state('INVESTIGATE', f"approaching Person #{self.target_id} in '{self.target_zone}'")
            return
        if now - self.search_since > self.search_timeout:
            self.resume(f'Person #{self.target_id} not found within {self.search_timeout:.0f} s', ignore=True)
            return
        if self.nav_busy:
            return
        if self.search_phase == 'goto':
            robot = self.robot_pose()
            if robot is None:
                return
            self.search_phase = 'look'
            self.search_yaws = [robot[2] + k * math.pi / 2.0 for k in (1, 2, 3, 4)]
            self.set_state('SEARCH', f'looking around for Person #{self.target_id}')
        if self.search_yaws:
            robot = self.robot_pose()
            if robot is not None:
                self.send_goal(robot[0], robot[1], self.search_yaws.pop(0), purpose='search')
        else:
            self.resume(f'Person #{self.target_id} not found after looking around', ignore=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SecurityEngine()
    except Exception as e:
        rclpy.logging.get_logger('security_engine').error(f'Could not start: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_goal()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
