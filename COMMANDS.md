

sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL https://github.com/docker/compose/releases/download/v2.24.6/docker-compose-linux-aarch64 \
-o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
docker compose version
docker compose up -d



pactl set-default-sink 0

docker run --rm -i \
  -v "$HOME/piper-voices:/voices" \
  piper-ai \
  --model /voices/uk_UA-lada-x_low.onnx \
  --output_file /voices/voice.wav \
  <<< "Привіт, я AI голос" && \
aplay "$HOME/piper-voices/voice.wav"

ros2 launch ros_cassandra_control controllers.launch.py

# Перевірка ros2_control без доступу до I2C/PCA9685
ros2 launch ros_cassandra_control controllers.launch.py dc_motor_dry_run:=true

# Виклик TTS через ROS 2 (цей самий service доступний у Foxglove як /speak)
ros2 service call /speak ros_cassandra_control/srv/Speak \
  "{text: 'Привіт Ханка'}"

# Пряма перевірка Piper HTTP API
curl -X POST http://127.0.0.1:8000/speak \
  -H 'Content-Type: application/json' \
  -d '{"text":"Привіт Ханка"}'

# Piper API виконує це автоматично перед кожним відтворенням:
pactl set-default-sink 0

sudo chown root:gpio /dev/gpiochip0
sudo chmod 660 /dev/gpiochip0

python3 servo_movement.py

python3.10 -m pip install adafruit-circuitpython-pca9685
python3.10 -m pip install Jetson.GPIO
python3.10 test_move.py
jtop


pip3 install onnx==1.10.2
python3 export.py --weights yolov5n.pt --include onnx (create yolov5n.onnx)
sudo find / -name trtexec 2>/dev/null (search trtexec)
/usr/src/tensorrt/bin/trtexec --onnx=yolov5s.onnx --saveEngine=yolov5s.engine (build .engine files)

# Download file from jetson
scp username@hostname:/path/to/remote/file /path/to/local/file

# For running mediapipe
pip3.10 uninstall tensorflow -y
pip3.10 uninstall mediapipe -y
pip3.10 uninstall protobuf -y
pip3.10 install protobuf==3.20.3
pip3.10 install mediapipe==0.10.14

pycuda-2020.1


# Install xacro
cd ~/ros2_ws/src
git clone -b humble https://github.com/ros/xacro.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select xacro

# Install realsense 
git clone -b humble https://github.com/ros/diagnostics.git
colcon build --packages-select diagnostic_updater

colcon build --packages-select realsense2_camera_msgs
colcon build --packages-select realsense2_camera

# Set up mp4 for start 
nano ~/.config/autostart/gif.desktop

[Desktop Entry]
Type=Application
Name=Kiosk Video
Exec=bash -c "sleep 5 && env DISPLAY=:0 mpv --fs --loop /home/xxx/code/Cassandra/media/cassandra.mp4"
X-GNOME-Autostart-enabled=true
NoDisplay=false

# Format gif to mp4 
ffmpeg -i input.gif \
  -movflags faststart \
  -pix_fmt yuv420p \
  output.mp4
