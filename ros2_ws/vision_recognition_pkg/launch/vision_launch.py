from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_recognition_pkg',
            executable='face_detector',
            name='face_detector'
        ),

        Node(
            package='vision_recognition_pkg',
            executable='object_detector',
            name='object_detector'
        ),

    ])
