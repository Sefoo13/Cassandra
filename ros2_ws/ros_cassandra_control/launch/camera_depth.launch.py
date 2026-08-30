#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        "device_type": "D435",
        "enable_depth": True,
        "enable_color": True,
        "enable_sync": True,
        "align_depth.enable": True,
        "depth_module.profile": "640x480x30",
        "rgb_camera.profile": "640x480x30",
        "depth_fps": 30,
        "color_fps": 30,
        "frame_id": "camera_link",
        "enable_infra1": False,
        "enable_infra2": False,
        "pointcloud.enable": False,
    }

    return LaunchDescription(
        [
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense2_camera",
                output="screen",
                parameters=[params],
            )
        ]
    )
