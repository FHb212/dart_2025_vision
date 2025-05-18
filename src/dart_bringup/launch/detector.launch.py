
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
            package='dart_detector',
            executable='dart_detector_node',
            output='both',
            parameters=[os.path.join(get_package_share_directory(
                    'dart_detector'), "config", "settings.yaml")],),
        ],
    )