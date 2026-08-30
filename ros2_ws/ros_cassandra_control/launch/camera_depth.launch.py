#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        "device_type": "D435",
        "enable_depth": True,
        "enable_color": True,
        # Keep depth pixels registered to the color image used by the detector.
        "enable_sync": True,
        "align_depth.enable": True,
        "depth_module.emitter_enabled": 1,
        "depth_module.profile": "640x480x15",
        "rgb_camera.profile": "640x480x15",
        "depth_fps": 15,
        "color_fps": 15,
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
