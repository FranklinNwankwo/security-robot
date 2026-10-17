import os

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('security_robot_webots')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    webots_urdf_path = os.path.join(package_dir, 'resource', 'security_robot.urdf')
    description_urdf_path = os.path.join(package_dir, 'resource', 'security_robot_description.urdf')
    with open(description_urdf_path, 'r') as f:
        robot_description = f.read()

    slam_params_path = os.path.join(package_dir, 'config', 'slam_params.yaml')

    rviz_config_path = os.path.join(package_dir, 'rviz', 'security_robot.rviz')
    rviz_slam_config_path = os.path.join(package_dir, 'rviz', 'security_robot_slam.rviz')
    rviz_arguments = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    rviz_slam_arguments = ['-d', rviz_slam_config_path] if os.path.exists(rviz_slam_config_path) else rviz_arguments

    use_rviz = LaunchConfiguration('rviz')
    use_slam = LaunchConfiguration('slam')

    rviz_without_slam = PythonExpression(["'", use_rviz, "' == 'true' and '", use_slam, "' != 'true'"])
    rviz_with_slam = PythonExpression(["'", use_rviz, "' == 'true' and '", use_slam, "' == 'true'"])

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

    scan_normalizer = Node(
        package='security_robot_webots',
        executable='scan_normalizer',
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(use_slam),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_arguments,
        condition=IfCondition(rviz_without_slam),
    )

    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_slam_arguments,
        condition=IfCondition(rviz_with_slam),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2 alongside the simulation'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Start SLAM Toolbox to build a map'),
        webots,
        security_robot_driver,
        robot_state_publisher,
        scan_normalizer,
        slam,
        rviz,
        rviz_slam,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
