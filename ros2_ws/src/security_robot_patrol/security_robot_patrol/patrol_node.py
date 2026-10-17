import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load_route(path):
    data = yaml.safe_load(Path(path).read_text())
    waypoints = data.get('waypoints', [])
    if not waypoints:
        raise ValueError(f'No waypoints in {path}')
    for wp in waypoints:
        for key in ('name', 'x', 'y'):
            if key not in wp:
                raise ValueError(f'Waypoint {wp} is missing "{key}"')
        wp.setdefault('yaw_deg', 0.0)
    return {
        'frame_id': data.get('frame_id', 'map'),
        'loop': bool(data.get('loop', True)),
        'pause_seconds': float(data.get('pause_seconds', 0.0)),
        'waypoints': waypoints,
    }


def make_pose(navigator, frame_id, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def go_to(navigator, route, wp):
    """Send one waypoint and block until Nav2 finishes. Returns the TaskResult."""
    goal = make_pose(navigator, route['frame_id'], wp['x'], wp['y'], wp['yaw_deg'])
    navigator.goToPose(goal)

    last_report = 0.0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.monotonic()
        if feedback and now - last_report > 3.0:
            navigator.get_logger().info(
                f'  -> {wp["name"]}: {feedback.distance_remaining:.2f} m remaining'
            )
            last_report = now
    return navigator.getResult()


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='patrol_navigator')
    log = navigator.get_logger()

    default_route = str(Path(get_package_share_directory('security_robot_patrol')) / 'config' / 'patrol_route.yaml')
    route_file = navigator.declare_parameter('route_file', default_route).value

    try:
        route = load_route(route_file)
    except Exception as e:
        log.error(f'Could not load route: {e}')
        navigator.destroy_node()
        rclpy.shutdown()
        return

    names = ' -> '.join(wp['name'] for wp in route['waypoints'])
    log.info(f'Loaded route from {route_file}: {names}{" (looping)" if route["loop"] else ""}')

    # Wait for Nav2 to be fully up. AMCL already has its initial pose from nav2_params.yaml,
    # so we wait on the controller instead of letting this call reset the robot's pose.
    log.info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active(localizer='controller_server')
    log.info('Nav2 active - starting patrol')

    lap = 0
    try:
        while rclpy.ok():
            lap += 1
            reached = 0
            log.info(f'=== Patrol lap {lap} ===')

            for wp in route['waypoints']:
                log.info(f'Heading to "{wp["name"]}" ({wp["x"]:.2f}, {wp["y"]:.2f})')
                result = go_to(navigator, route, wp)

                if result == TaskResult.SUCCEEDED:
                    reached += 1
                    log.info(f'Reached "{wp["name"]}"')
                    if route['pause_seconds'] > 0.0:
                        time.sleep(route['pause_seconds'])
                elif result == TaskResult.CANCELED:
                    log.warn(f'Goal "{wp["name"]}" was cancelled - stopping patrol')
                    return
                else:
                    log.warn(f'Could not reach "{wp["name"]}" - skipping to next waypoint')

            log.info(f'Lap {lap} complete: {reached}/{len(route["waypoints"])} waypoints reached')

            if reached == 0:
                log.error('No waypoint reachable this lap - stopping patrol. Check Nav2 and the route.')
                break
            if not route['loop']:
                break
    except KeyboardInterrupt:
        log.info('Patrol interrupted - cancelling current goal')
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
