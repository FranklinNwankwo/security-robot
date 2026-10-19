from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='security_robot_perception', executable='detector_node', output='screen'),
        Node(package='security_robot_perception', executable='tracker_node', output='screen'),
    ])
