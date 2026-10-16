import os

import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('security_robot_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'security_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'security_robot_world.wbt')
    )

    security_robot_driver = WebotsController(
        robot_name='security_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ],
    )

    return launch.LaunchDescription([
        webots,
        security_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
