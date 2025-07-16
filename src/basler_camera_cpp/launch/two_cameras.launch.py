# two_cameras.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('basler_camera_cpp')
    return LaunchDescription([
        Node(
            package='basler_camera_cpp',
            executable='basler_camera_node',
            name='camera_43',
            output='screen',
            parameters=[{'camera_id': 43}],
        ),
        Node(
            package='basler_camera_cpp',
            executable='basler_camera_node',
            name='camera_44',
            output='screen',
            parameters=[{'camera_id': 44}],
        ),
    ])
