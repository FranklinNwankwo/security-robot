"""Moves the 'walker' person along a route in Webots.

Runs from WSL as an extern supervisor controller (like the robot's own controller), and publishes the
walker's true position on /sim/walker_position so tracker estimates can be checked against ground truth."""
import glob
import math
import os

# Route in map/world coordinates: main hall -> east doorway -> inside the east room (restricted)
ROUTE = [(-0.6, -1.0), (0.6, 0.0), (1.5, 0.0), (2.6, -0.9)]
SPEED = 0.5      # m/s, a slow walk
PAUSE = 10.0     # s, standing still at each end of the route


def host_ip():
    """Windows host as seen from WSL (where Webots runs) - same lookup webots_ros2 uses."""
    try:
        with open('/etc/resolv.conf') as f:
            for line in f:
                if line.startswith('nameserver'):
                    return line.split()[1]
    except OSError:
        pass
    return '127.0.0.1'


def find_linux_webots_home():
    """Folder X such that X/lib/controller/libController.so is the Linux Webots controller library."""
    for lib in glob.glob('/opt/ros/jazzy/**/lib/controller/libController.so', recursive=True):
        return os.path.dirname(os.path.dirname(os.path.dirname(lib)))
    return None


def main():
    # Both must be set before the Webots controller library starts.
    os.environ.setdefault('WEBOTS_CONTROLLER_URL', f'tcp://{host_ip()}:1234/walker')
    # The global WEBOTS_HOME points at the *Windows* Webots (used to launch it). This Linux process needs the
    # Linux controller library shipped with webots_ros2, so point WEBOTS_HOME there - for this process only.
    linux_home = find_linux_webots_home()
    if linux_home is None:
        raise SystemExit('Linux libController.so not found under /opt/ros/jazzy')
    os.environ['WEBOTS_HOME'] = linux_home

    from controller import Supervisor
    import rclpy
    from geometry_msgs.msg import PointStamped

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    me = robot.getSelf()
    translation = me.getField('translation')
    rotation = me.getField('rotation')

    rclpy.init()
    node = rclpy.create_node('walker')
    pub = node.create_publisher(PointStamped, '/sim/walker_position', 10)
    log = node.get_logger()

    # One full cycle: walk the route, pause, walk it back, pause
    legs = list(ROUTE[1:]) + ['PAUSE'] + list(reversed(ROUTE[:-1])) + ['PAUSE']
    x, y = ROUTE[0]
    heading = 0.0
    leg = 0
    pause_left = PAUSE
    since_pub = 0.0
    translation.setSFVec3f([x, y, 0.0])
    log.info(f'Walker connected - walking {ROUTE[0]} -> {ROUTE[-1]} and back at {SPEED} m/s')

    while robot.step(timestep) != -1:
        dt = timestep / 1000.0
        target = legs[leg % len(legs)]

        if target == 'PAUSE':
            pause_left -= dt
            if pause_left <= 0.0:
                pause_left = PAUSE
                leg += 1
                log.info('Walker moving again')
        else:
            tx, ty = target
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            step = SPEED * dt
            if dist <= step:
                x, y = tx, ty
                leg += 1
                if legs[leg % len(legs)] == 'PAUSE':
                    where = 'inside the east room' if (x, y) == ROUTE[-1] else 'in the main hall'
                    log.info(f'Walker stopped {where} at ({x:.1f}, {y:.1f})')
            else:
                x += dx / dist * step
                y += dy / dist * step
                heading = math.atan2(dy, dx)
            translation.setSFVec3f([x, y, 0.0])
            rotation.setSFRotation([0.0, 0.0, 1.0, heading])

        since_pub += dt
        if since_pub >= 0.1:
            since_pub = 0.0
            msg = PointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x, msg.point.y = x, y
            pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
