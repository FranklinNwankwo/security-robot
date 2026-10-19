import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('security_robot_perception'), 'launch', 'perception_launch.py')
        )
    )
    return LaunchDescription([
        perception,
        Node(package='security_robot_security', executable='zone_monitor', output='screen'),
        Node(package='security_robot_security', executable='security_engine', output='screen'),
    ])
