# dc_motor_controller

`ros2_control` system hardware plugin for Cassandra's four DC motors and the
PCA9685 PWM controller.

The plugin is loaded by `controller_manager` as:

```xml
<plugin>dc_motor_controller/DCMotorSystemHardware</plugin>
```

It exports one velocity command interface and position/velocity state
interfaces for each wheel. `diff_drive_controller` writes wheel angular
velocities directly to those command interfaces; there is no ROS topic bridge
or second motor-control lifecycle node.

## Hardware behavior

- `on_configure` opens the configured Linux I2C device, selects the PCA9685,
  and configures its PWM frequency.
- `on_activate` initializes every command and forces all motor outputs to zero.
- `write` scales each wheel velocity against `max_wheel_speed_rad_s`, clamps it,
  applies the joint's electrical direction, and writes the two H-bridge
  channels.
- `on_deactivate` and `on_cleanup` stop all motors.
- I2C/PCA9685 failures return an error to `controller_manager`.

Cassandra currently has no wheel encoders. The exported states are therefore
calibrated estimates. The common wheel component used for forward/backward
motion is multiplied by `linear_velocity_state_scale`; the differential
component used for turning is preserved.

## Configuration

Hardware-wide parameters are declared in
`ros_cassandra_description/urdf/control_cassandra.xacro`:

- `i2c_device` (normally `/dev/i2c-1`)
- `i2c_address` (normally `0x40`)
- `pwm_frequency`
- `min_pwm_percent` — minimum duty cycle for a non-zero command, used to
  overcome motor static friction
- `max_pwm_percent`
- `max_wheel_speed_rad_s`
- `command_deadband_rad_s`
- `linear_velocity_state_scale` — multiplier for estimated forward/backward
  velocity and odometry; it does not scale pure rotation
- `dry_run`

Each wheel joint declares:

- `forward_channel`
- `reverse_channel`
- `direction` (`1` or `-1`)

The channel mapping preserves the wiring used by the previous Python driver:

| Joint | PCA9685 pair |
| --- | --- |
| `joint_front_left_wheel` | 4, 5 |
| `joint_rear_left_wheel` | 8, 9 |
| `joint_front_right_wheel` | 7, 6 |
| `joint_rear_right_wheel` | 11, 10 |

## Build and run

```bash
cd /Users/sviat/PycharmProjects/Cassandra/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dc_motor_controller ros_cassandra_description ros_cassandra_control
source install/setup.bash
ros2 launch ros_cassandra_control controllers.launch.py
```

For controller testing without opening I2C:

```bash
ros2 launch ros_cassandra_control controllers.launch.py dc_motor_dry_run:=true
```

Send commands to the standard diff-drive input:

```bash
ros2 topic pub -r 10 /diff_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```
