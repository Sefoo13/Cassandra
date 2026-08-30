# Cassandra text-to-speech

## Microphone speech recognition

`speech_recognition_node.py` continuously captures mono microphone audio,
recognizes completed phrases with an offline Vosk model, and publishes them as
`std_msgs/msg/String` messages on `/voice/transcript`.

Unpack a Vosk model on the host (for Ukrainian, use a Ukrainian model) and set
its directory before starting the container:

```bash
export VOSK_MODEL_DIR=/absolute/path/to/unpacked-vosk-model
docker compose build ros_cassandra
docker compose up
```

After building and sourcing the ROS workspace, verify recognized phrases with:

```bash
ros2 topic echo /voice/transcript
```

The node starts from `foxglove.launch.py`. Its parameters are stored in
`config/speech_recognition.yaml`. By default it opens the system microphone at
16 kHz and reads the model from `/models/vosk`.

For microphone diagnostics, `playback_recognized_audio` replays the captured
PCM audio after Vosk detects the end of a non-empty phrase. Microphone capture
is ignored during playback and for the configured listening cooldown to avoid
an audio feedback loop. `playback_output_device` selects a SoundDevice output.
When it is empty, the node first searches for a ReSpeaker/XVF3800 output and
falls back to the system default only if none is available. Disable this option
after microphone testing to avoid adding playback latency to normal voice
interaction. `playback_gain` changes only diagnostic playback volume and does
not alter the audio sent to the recognizer.

## Short ChatGPT responses

`chat_response_node.py` subscribes to `/voice/transcript`, sends each phrase to
the OpenAI Responses API, publishes the result on `/chat/response`, and calls
`/speak`. Export the API key before creating the ROS container:

```bash
export OPENAI_API_KEY="your-api-key"
docker compose up -d --build --force-recreate ros_cassandra
```

The model, prompt, conversation memory, and response limits are configured in
`config/chat_response.yaml`. The prompt asks for short Ukrainian plain text,
and the node additionally truncates every response to `max_words` before it is
published or spoken.

Test the chat path without using the microphone:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String \
  "{data: 'Що ти вмієш?'}"

ros2 topic echo /chat/response
```

## Voice control

`voice_command_node.py` converts a bounded set of Ukrainian phrases from
`/voice/command` into named servo poses or short `/cmd_vel` commands.
Configuration and safety limits are stored in `config/voice_command.yaml`.

Normal speech is published on `/voice/transcript` and repeated through Piper.
Commands require the wake word `команда`. With persistent command mode enabled,
the robot announces that command mode is active and routes subsequent phrases
to `/voice/command` until it hears `вихід`, `вийти`, or
`завершити режим команд`. It then announces that command mode is off. Both an
inline first command and separate commands are supported:

```text
Команда, вперед

Команда
(robot: Режим команд увімкнено)
поверни голову ліворуч
підніми руки
вихід
(robot: Режим команд вимкнено)
```

Supported examples:

```text
поверни голову ліворуч / праворуч
підніми голову / опусти голову
постав голову рівно
поверни корпус ліворуч / праворуч
склади руки
підніми руки
початкове положення
вперед / їдь вперед
назад / їдь назад
поверни ліворуч / праворуч
стоп / стій / зупинись
```

Movement uses fixed low speeds for one second by default. `стоп` is always
accepted and immediately publishes a zero velocity. Execution status is
published on `/voice/command_status` for Foxglove.

Before testing on physical wheels, raise the robot or otherwise prevent it
from moving unexpectedly. Commands can be tested by publishing text directly:

```bash
ros2 topic pub --once /voice/command std_msgs/msg/String \
  "{data: 'поверни голову ліворуч'}"

ros2 topic pub --once /voice/command std_msgs/msg/String \
  "{data: 'їдь вперед'}"

ros2 topic pub --once /voice/command std_msgs/msg/String "{data: 'стоп'}"
```

Text-to-speech is exposed as the ROS 2 service `/speak`, so it can be called
from Foxglove's **Service Call** panel. The ROS node forwards each request to
the Piper HTTP API. Piper generates a temporary WAV file and plays it through
PulseAudio inside its own container.

The launch file starts two bridges. Foxglove Bridge remains available on port
8765 for high-performance visualization. Rosbridge runs on port 9090 and is
the recommended connection for `/speak`, because its JSON service transport
preserves non-ASCII text such as Ukrainian.

When Foxglove is connected through Rosbridge it cannot resolve `package://`
mesh URLs. The controller launch file automatically detects the robot's
default-route IP and publishes HTTP mesh URLs using the built-in asset server.
For robots with multiple network interfaces, the detected address can be
overridden:

```bash
export CASSANDRA_MESH_BASE_URL=http://ROBOT_IP:8080/meshes
```

## Start

Put the model at:

```text
$HOME/piper-voices/uk_UA-lada-x_low.onnx
```

Then rebuild and start the containers:

```bash
docker compose up -d --build piper
docker compose build ros_cassandra
docker compose run --rm ros_cassandra \
  bash -lc "source /ros2_ws/install/setup.bash && \
  ros2 launch ros_cassandra_control foxglove.launch.py"
```

Both containers currently use Docker host networking because the robot stack
already relies on it. Therefore the ROS container reaches Piper at
`http://127.0.0.1:8000/speak`. If the stack is moved to a normal Compose
network, change `piper_api_url` in `config/tts_service.yaml` to
`http://piper:8000/speak` and set `PIPER_API_HOST` to `0.0.0.0`. With the
current configuration, the unauthenticated Piper API is intentionally
reachable only through the host's loopback interface.

## Call from Foxglove

1. Select a **Rosbridge** connection and connect to `ws://ROBOT_IP:9090`.
2. Add the **Service Call** panel.
3. Select `/speak`.
4. Send:

```json
{
  "text": "Привіт Ханка"
}
```

The response contains `success` and `message`. Requests are played one at a
time, so concurrent callers do not overlap audio.

## Diagnostics

```bash
curl http://127.0.0.1:8000/health

curl -X POST http://127.0.0.1:8000/speak \
  -H 'Content-Type: application/json' \
  -d '{"text":"Привіт Ханка"}'

ros2 service call /speak ros_cassandra_control/srv/Speak \
  "{text: 'Привіт Ханка'}"
```

To use a different PulseAudio output, set `PULSE_SINK` before starting
Compose, for example:

```bash
PULSE_SINK=1 docker compose up -d piper
```

Before every playback it runs `pactl set-default-sink 0`. The PulseAudio
socket is mounted from `/run/user/1000/pulse` by default, and `paplay` sends
the generated WAV to that selected sink.

If the desktop user has a different UID or PulseAudio runtime directory, start
it with:

```bash
LOCAL_UID="$(id -u)" \
LOCAL_GID="$(id -g)" \
PULSE_RUNTIME_DIR="${XDG_RUNTIME_DIR}/pulse" \
PULSE_SINK=0 \
docker compose up -d --build piper
```


sudo python3 move_slow.py 


sudo python3 movement.py

sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9 g++-9

cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9-linux-aarch64.tar.gz
tar -xzf cmake-3.27.9-linux-aarch64.tar.gz
export PATH=$HOME/cmake-3.27.9-linux-aarch64/bin:$PATH
cmake --version

git clone https://github.com/rhasspy/piper.git
cd ~/piper
mkdir build
cd build
export CC=gcc-9
export CXX=g++-9
cmake ..
make -j1

pip3 install --user scikit-build
pip3 install --user setuptools wheel

pip3 install --user cmake

sudo apt install espeak-ng espeak-ng-data

nano src/ros_cassandra_description/urdf/base_urdf.xacro
colcon build && source install/setup.sh && ros2 launch ros_cassandra_control cassandra.xml 
ros2 run vision_recognition_pkg face_detector
source install/setup.sh && ros2 run vision_recognition_pkg object_detector

The configurable YOLO detector can also be started with:

```bash
ros2 launch vision_recognition_pkg object_detection.launch.py
```

It subscribes to the raw RealSense color stream, runs the existing YOLOv5n
TensorRT engine, publishes an annotated raw `sensor_msgs/Image` stream on
`/vision/objects`, and publishes JSON detection records on
`/vision/detections`. Confidence, IoU, tracking, and maximum processing FPS are
configured in `config/object_detection.yaml`. The overlay includes FPS,
inference latency, Jetson load and temperature, object count, tracking IDs,
centers, directions, and the selected person target. When aligned RealSense
depth is enabled and available, detections also include ``. Depth and
alignment are disabled by default on the Jetson Nano to keep the RealSense
color stream stable. This backend uses TensorRT and PyCUDA directly and does
not require PyTorch or Ultralytics.

Person following starts disabled. Saying the single word `слідкуй` enables it;
`зупинись` disables it and also stops wheel movement. The detector keeps the
current person tracking ID and publishes smoothly filtered yaw positions for
servo 10 on `/ros_cassandra/servos/raw_commands`. A center dead zone, minimum
step, and command interval prevent rapid oscillation. If the person is lost,
the head returns to center after two seconds.

On the Jetson Nano/L4T R32, run this node in the `dusty` container, whose base
image provides the matching L4T TensorRT runtime. Both containers use host ROS
networking, so its topics are visible to Cassandra:

```bash
docker compose build dusty
docker compose up -d --force-recreate dusty
```

The `dusty` service performs the incremental package build and launches the
detector automatically whenever the container starts. Follow its output with:

```bash
docker compose logs -f dusty
```

Inspect detections with:

```bash
ros2 topic echo /vision/detections
```

cd .. && source opt/ros/humble/setup.sh && cd ros2_ws/ && ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p pixel_format:=yuyv2rgb

docker run --rm --net=host --device /dev/video2:/dev/video2 -it foxglove_bridge

# Tank ros project

Tank launch project packages:
- ros_tank_logic
- ros_tank_gazebo
- ros_tank_navigation
- ros_tank_description

![Selection_096](https://user-images.githubusercontent.com/23004657/209576988-321a2a82-18bd-4550-98bb-9a9118b5310c.png)


Then change value param for tank launcher:
<param name="video_device" value="/dev/video4" />

Commands for starting the project:

1. download file - arduino/ros_tank.ino to arduino

2. Connect via ssh to jetson 
   ssh name@192.168.0.140
 exec following commands:
   cd ros_tank
   ./run_jetson.bash
   ros2 launch ros_tank_control ros_tank_control_diff.launch.py

 Command to check:
   available cameras:
     ls /dev | grep video*
   available lidar:
     ls -l /dev |grep ttyUSB
 
 lsusb - get list USB devices
 sudo chmod 666 /dev/ttyUSB0 or /dev/ttyACM0

3. In PC terminal
./run.bash
ros2 launch ros_tank_logic ros_tank_rviz.launch.py

4. Another PC terminal 
./exec.bash 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped


_________________________________________________________________________
_________________________________________________________________________
For simulation:
_________________________________________________________________________
_________________________________________________________________________

[//]: # (sourcing Gazebo's setup)
. /usr/share/gazebo/setup.sh

ros2 launch ros_tank_logic ros_tank_sim.launch.xml

<!-- Spawn world in gazebo running sim -->
- ros2 launch ros_tank_gazebo start_world.launch.py

<!-- Publish URDF file in robot_description topic and launch rviz -->
- ros2 launch ros_tank_logic ros_tank_rviz_sim.launch.py

<!-- Read robot_description and spawn in gazebo running sim -->
- ros2 launch ros_tank_gazebo spawn_robot.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard 

_________________________________________________________________________
_________________________________________________________________________


run joint state publisher node:
ros2 run joint_state_publisher_gui joint_state_publisher_gui

run lidar node:
ros2 launch rplidar_ros view_rplidar.launch.py 
ros2 launch rplidar_ros rplidar.launch.py 

ros2 launch ros_tank_navigation rplidar.launch.py
ros2 launch ros_cassandra_control camera.launch.py

ros2 launch ros_tank_control ros_tank.xml

checking camera:
sudo apt-get install v4l-utils
v4l2-ctl --list-devices

run camera node:
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /ros2_ws/src/ros_cassandra_control/config/camera-params.yaml


ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "
layout:
 dim: []
 data_offset: 0
data:
 - 1
 - 1"
