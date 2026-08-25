from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='star_nose_ros2', executable='ur5_node', name='ur5_node', output='screen'),
        Node(package='star_nose_ros2', executable='data_logger_node', name='data_logger_node', output='screen'),
        Node(package='star_nose_ros2', executable='ui_node', name='ui_node', output='screen'),
    ])
