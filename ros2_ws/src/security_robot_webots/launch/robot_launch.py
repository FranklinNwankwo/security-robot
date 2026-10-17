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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    webots_urdf_path = os.path.join(package_dir, 'resource', 'security_robot.urdf')
    description_urdf_path = os.path.join(package_dir, 'resource', 'security_robot_description.urdf')
    with open(description_urdf_path, 'r') as f:
        robot_description = f.read()

    slam_params_path = os.path.join(package_dir, 'config', 'slam_params.yaml')
    nav2_params_path = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    map_path = os.path.join(package_dir, 'maps', 'room_map.yaml')

    rviz_config_path = os.path.join(package_dir, 'rviz', 'security_robot.rviz')
    rviz_slam_config_path = os.path.join(package_dir, 'rviz', 'security_robot_slam.rviz')
    rviz_nav_config_path = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_arguments = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    rviz_slam_arguments = ['-d', rviz_slam_config_path] if os.path.exists(rviz_slam_config_path) else rviz_arguments
    rviz_nav_arguments = ['-d', rviz_nav_config_path]

    use_rviz = LaunchConfiguration('rviz')
    use_slam = LaunchConfiguration('slam')
    use_nav = LaunchConfiguration('nav')

    rviz_plain_condition = PythonExpression(
        ["'", use_rviz, "' == 'true' and '", use_slam, "' != 'true' and '", use_nav, "' != 'true'"])
    rviz_slam_condition = PythonExpression(
        ["'", use_rviz, "' == 'true' and '", use_slam, "' == 'true'"])
    rviz_nav_condition = PythonExpression(
        ["'", use_rviz, "' == 'true' and '", use_nav, "' == 'true' and '", use_slam, "' != 'true'"])

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'security_robot_world.wbt')
    )

    security_robot_driver = WebotsController(
        robot_name='security_robot',
        parameters=[
            {'robot_description': webots_urdf_path},
        ],
        remappings=[('/camera/image_color', '/camera/image_raw')],
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_params_path,
            'use_sim_time': 'False',
            'autostart': 'True',
            'slam': 'False',
        }.items(),
        condition=IfCondition(use_nav),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_arguments,
        condition=IfCondition(rviz_plain_condition),
    )

    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_slam_arguments,
        condition=IfCondition(rviz_slam_condition),
    )

    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_nav_arguments,
        condition=IfCondition(rviz_nav_condition),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2 alongside the simulation'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Start SLAM Toolbox to build a map'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Start Nav2 with the saved map'),
        webots,
        security_robot_driver,
        robot_state_publisher,
        scan_normalizer,
        slam,
        nav2,
        rviz,
        rviz_slam,
        rviz_nav,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
