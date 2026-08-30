import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vision_recognition_pkg')
    return LaunchDescription([
        Node(
            package='vision_recognition_pkg',
            executable='face_detector',
            name='face_detector'
        ),

        Node(
            package='vision_recognition_pkg',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[
                os.path.join(package_share, 'config', 'object_detection.yaml')
            ],
        ),

    ])
