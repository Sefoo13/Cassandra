# cassandra_servo_controller

Lifecycle-aware ROS 2 Python driver for Cassandra's thirteen LewanSoul LX-16A
servos. Servo IDs, home positions, and named poses were derived from
`test_scripts/servo_movement.py`.

## Lifecycle and safety

- `on_configure`: validates parameters and opens `/dev/ttyUSB0`.
- `on_activate`: starts the command and state loops.
- `write`: performs one synchronized `move_prepare` / `move_start`.
- `read`: publishes measured servo positions as ROS joint radians.
- `on_deactivate`: issues `move_stop`; optional torque release is configurable.
- `on_cleanup`, `on_shutdown`, and `on_error`: stop and close the serial port.

The node ignores commands unless its lifecycle state is `ACTIVE`. It does not
automatically move to the home pose on startup unless
`move_to_home_on_activate` is enabled.

## Install hardware dependencies

```bash
python3 -m pip install pyserial lewansoul-lx16a
```

The driver uses the same `lewansoul_lx16a.ServoController` API as the source
test script.

## Build and launch

```bash
cd /Users/sviat/PycharmProjects/Cassandra/ros2_ws
colcon build --packages-select cassandra_servo_controller --symlink-install
source install/setup.bash
ros2 launch cassandra_servo_controller servo_controller.launch.py
```

Test the lifecycle and topics without opening the serial port:

```bash
ros2 launch cassandra_servo_controller servo_controller.launch.py dry_run:=true
```

The launch file automatically performs `configure` followed by `activate`.

## forward_position_controller bridge

The driver subscribes to `/forward_position_controller/commands` and maps the
controller's ordered `Float64MultiArray` directly to all thirteen physical
servos, including head yaw and head pitch. The `forward_joint_names` parameter
must stay in the same order as `forward_position_controller.joints` in
`ros_cassandra_control/config/controllers.yaml`.

```bash
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, -0.15]}"
```

The final two values control `head_yaw_joint` and `head_pitch_joint`.

## Named poses

Available atomic poses are:

```text
base, pray_pose, pray, pray_right, pray_left, whole,
head_right_down, head_left_up, head_left, head_right, head_center,
head_up, head_down, crest, torso_left, torso_center, torso_right
```

Example:

```bash
ros2 topic pub --once /ros_cassandra/servos/pose std_msgs/msg/String \
  "{data: base}"
```

## Raw commands

For calibration, publish pairs of `servo ID, raw position`:

```bash
ros2 topic pub --once /ros_cassandra/servos/raw_commands \
  std_msgs/msg/Int32MultiArray "{data: [10, 300, 11, 550]}"
```

Raw values are clamped to `raw_min..raw_max`, initially `0..1000`.

Measured states are published at 20 Hz on the standard joint-state topic so
`robot_state_publisher` and Foxglove animate the physical intermediate
positions instead of jumping to the final command:

```bash
ros2 topic echo /joint_states
```

Review `config/servos.yaml` on the physical robot. In particular, verify every
servo ID, home position, direction, and raw limit before commanding large
movements. The four right-arm servos use direction `-1` because their physical
installation is mirrored relative to the left arm.
