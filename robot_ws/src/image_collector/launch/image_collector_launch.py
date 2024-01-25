import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Use the absolute path to the YAML file in the source directory
    config_file_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'image_collector_config.yaml')
    )
    return LaunchDescription([
        Node(
            package='image_collector',
            executable='collect',
            name='image_collector_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
        )
    ])