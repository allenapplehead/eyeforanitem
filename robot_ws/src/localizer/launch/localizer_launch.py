import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localizer',
            executable='localizer',
            name='localizer',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='localizer',
            executable='model_odom',
            name='model_odom',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='localizer',
            executable='tag_odom',
            name='tag_odom',
            output='screen',
            emulate_tty=True,
        )
    ])