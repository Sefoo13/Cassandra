xhost +local:root

SCRIPT_PATH=$(readlink -f "$0")
SCRIPT_DIR_PATH=$(dirname "$SCRIPT_PATH")
WS_DIR_PATH=$(realpath "$SCRIPT_DIR_PATH")

docker run -it --rm \
    --privileged \
    --runtime="nvidia" \
    --device="/dev/ttyACM0" \
    --device="/dev/ttyUSB0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth" \
    --volume="$WS_DIR_PATH/ros2_ws/ros_cassandra_control:/ros2_ws/src/ros_cassandra_control" \
    --volume="$WS_DIR_PATH/ros2_ws/ros_cassandra_description:/ros2_ws/src/ros_cassandra_description" \
    --volume="$WS_DIR_PATH/ros2_ws/diffdrive_arduino:/ros2_ws/src/diffdrive_cassandra" \
    --volume="$WS_DIR_PATH/ros2_ws/vision_recognition_pkg:/ros2_ws/src/vision_recognition_pkg" \
    --env="QT_X11_NO_MITSHM=0" \
    --env="XAUTHORITY=/tmp/.docker.xauth" \
    --name="ros_cassandra" \
    --network="host" \
    cassandra_ros  \
    bash

xhost -local:root