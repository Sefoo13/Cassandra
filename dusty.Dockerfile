FROM dustynv/ros:humble-desktop-l4t-r32.7.1

ENV ROS_WS=/ros2_ws
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

WORKDIR $ROS_WS/src

RUN \
    #apt-get update &&
    apt-get install -y \
    git \
#    vim \
#    less \
    xterm \
    nano \
    python3-pip \
    python3-setuptools \
 #   python3-pydantic \
    python3-empy \
    python3-numpy \
    build-essential \
    cmake \
 #   libwebsocketpp-dev \
    libasio-dev \
#    nlohmann-json3-dev \
#    rapidjson-dev \
#    v4l-utils \
#    libv4l-dev \
    #ros-${ROS_DISTRO}-xacro \
    #ros-${ROS_DISTRO}-usb-cam \
    #ros-${ROS_DISTRO}-ros2-control \
    #ros-${ROS_DISTRO}-ros2-controllers \
    #ros-${ROS_DISTRO}-hardware-interface \
    #ros-${ROS_DISTRO}-joint-state-publisher-gui \
    #ros-${ROS_DISTRO}-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install \
    colcon-common-extensions \
    vcstool \
    wget \
    setuptools \
    rosdep \
    pycuda==2020.1

RUN rosdep init || true

WORKDIR $ROS_WS

RUN rosdep install --from-paths src --ignore-src -r -y || echo "Ignoring missing ARM64 packages"
RUN rm -rf build install log

COPY ./ros2_ws/ros_cassandra_control ./src/ros_cassandra_control
COPY ./ros2_ws/vision_recognition_pkg ./src/vision_recognition_pkg
#COPY ./ros2_ws/diffdrive_cassandra ros2_ws/src/diffdrive_cassandra
COPY ./ros2_ws/ros_cassandra_description ./src/ros_cassandra_description

# клонуємо xacro
RUN cd src && git clone https://github.com/ros/xacro.git && \
              git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development


#RUN ./ros_entrypoint.sh && \
 #   colcon build --packages-select ros_cassandra_control ros_cassandra_description

RUN cd .. && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["sleep", "infinity"]
