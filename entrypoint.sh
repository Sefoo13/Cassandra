#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
    echo "Starting Foxglove Bridge on ws://0.0.0.0:8765"
    exec ros2 run foxglove_bridge foxglove_bridge
else
    exec "$@"
fi
