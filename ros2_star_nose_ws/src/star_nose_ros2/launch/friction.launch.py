"""Bring up the friction rig: UR5 friction control node + terminal UI +
data logger. ur5_friction_node owns RTDE and the trajectory/force loop; the UI
publishes /friction/command; the logger records pose + sensor.

Params you may want to override, e.g.:
  ros2 launch star_nose_ros2 friction.launch.py robot_ip:=127.0.0.1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='177.22.22.2'),
        Node(package='star_nose_ros2', executable='ur5_friction_node',
             name='ur5_friction_node', output='screen',
             parameters=[{'robot_ip': robot_ip}]),
        Node(package='star_nose_ros2', executable='friction_ui_node',
             name='friction_ui_node', output='screen', emulate_tty=True),
        Node(package='star_nose_ros2', executable='data_logger_node',
             name='data_logger_node', output='screen'),
    ])
