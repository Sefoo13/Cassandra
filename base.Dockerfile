FROM arm64v8/ros:humble-ros-base

ENV ROS_WS=/ros2_ws
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

WORKDIR $ROS_WS/src

RUN apt-get update && \
    apt-get install -y \
    git \
    vim \
    less \
    xterm \
    nano \
    python3-pip \
    python3-setuptools \
    python3-pydantic \
    python3-empy \
    python3-numpy \
    build-essential \
    cmake \
    libwebsocketpp-dev \
    libasio-dev \
    nlohmann-json3-dev \
    rapidjson-dev \
    v4l-utils \
    libv4l-dev \
    libportaudio2 \
    portaudio19-dev \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install \
    colcon-common-extensions \
    vcstool \
    wget \
    setuptools \
    rosdep \
    adafruit-circuitpython-pca9685 \
    Jetson.GPIO \
    lewansoul-lx16a \
    openai \
    sounddevice \
    vosk

RUN rosdep init || true
RUN rosdep update

WORKDIR $ROS_WS

RUN rosdep install --from-paths src --ignore-src -r -y || echo "Ignoring missing ARM64 packages"
RUN rm -rf build install log

COPY ./ros2_ws/ros_cassandra_control ./src/ros_cassandra_control
COPY ./ros2_ws/dc_motor_controller ./src/dc_motor_controller
COPY ./ros2_ws/cassandra_servo_controller ./src/cassandra_servo_controller
COPY ./ros2_ws/ros_cassandra_description ./src/ros_cassandra_description

## клонуємо xacro
#RUN cd src && git clone https://github.com/ros/xacro.git && \
#              git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

#RUN cd .. && echo "source /ros2_ws/install/setup.bash"

#RUN cd src && \
#    colcon build --packages-select xacro && \
#    colcon build --packages-select ros_cassandra_control ros_cassandra_description

RUN cd .. && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["sleep", "infinity"]
