import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "ros_cassandra_description"

    camera_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "camera_params.yaml"
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output="screen",
        parameters=[camera_params_file],
    )

    return LaunchDescription(
        [
            camera_node,
        ]
    )
