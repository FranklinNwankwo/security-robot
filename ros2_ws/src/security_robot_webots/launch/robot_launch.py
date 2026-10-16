import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('security_robot_webots')

    webots_urdf_path = os.path.join(package_dir, 'resource', 'security_robot.urdf')
    description_urdf_path = os.path.join(package_dir, 'resource', 'security_robot_description.urdf')
    with open(description_urdf_path, 'r') as f:
        robot_description = f.read()

    rviz_config_path = os.path.join(package_dir, 'rviz', 'security_robot.rviz')
    rviz_arguments = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    use_rviz = LaunchConfiguration('rviz')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'security_robot_world.wbt')
    )

    security_robot_driver = WebotsController(
        robot_name='security_robot',
        parameters=[
            {'robot_description': webots_urdf_path},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_arguments,
        condition=IfCondition(use_rviz),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2 alongside the simulation'),
        webots,
        security_robot_driver,
        robot_state_publisher,
        rviz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
