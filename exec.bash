#!/bin/bash
xhost -local:root

docker exec -it ros_cassandra bash -c "
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec bash
"
