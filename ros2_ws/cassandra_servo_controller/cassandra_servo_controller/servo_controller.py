"""Lifecycle-aware ROS 2 driver for LewanSoul LX-16A servos."""

from __future__ import annotations

import math
from typing import Any, Sequence

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String

from .model import LX16A_RADIANS_PER_UNIT, POSES, clamp_raw
from .model import radians_to_raw, raw_to_radians


DEFAULT_SERVO_IDS = tuple(range(1, 12))
DEFAULT_JOINT_NAMES = (
    "left_shoulder_joint",
    "left_shoulder_low_joint",
    "left_bicep_joint",
    "left_arm_joint",
    "upper_torso_joint",
    "right_shoulder_joint",
    "right_shoulder_low_joint",
    "right_bicep_joint",
    "right_arm_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)
DEFAULT_HOME_POSITIONS = (450, 730, 500, 450, 400, 660, 100, 470, 200, 460, 550)
DEFAULT_DIRECTIONS = (
    1.0,   # left_shoulder_joint
    1.0,   # left_shoulder_low_joint
    1.0,   # left_bicep_joint
    1.0,   # left_arm_joint
    1.0,   # upper_torso_joint
    -1.0,  # right_shoulder_joint
    -1.0,  # right_shoulder_low_joint
    -1.0,  # right_bicep_joint
    -1.0,  # right_arm_joint
    1.0,   # head_yaw_joint
    1.0,   # head_pitch_joint
)
DEFAULT_FORWARD_JOINT_NAMES = (
    "right_shoulder_joint",
    "left_shoulder_joint",
    "right_shoulder_low_joint",
    "left_shoulder_low_joint",
    "right_bicep_joint",
    "left_bicep_joint",
    "left_arm_joint",
    "right_arm_joint",
    "upper_torso_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)


class LX16AServoBus:
    """Serial hardware abstraction around the lewansoul_lx16a library."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float,
        servo_ids: Sequence[int],
        dry_run: bool = False,
    ) -> None:
        self.servo_ids = tuple(int(servo_id) for servo_id in servo_ids)
        self.dry_run = dry_run
        self.serial: Any | None = None
        self.controller: Any | None = None
        self.servos: dict[int, Any] = {}
        self.last_positions: dict[int, int] = {}

        if not dry_run:
            try:
                import serial
                import lewansoul_lx16a
            except ImportError as exc:
                raise RuntimeError(
                    "Install pyserial and lewansoul-lx16a, or use dry_run:=true"
                ) from exc

            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            self.controller = lewansoul_lx16a.ServoController(self.serial)
            self.servos = {
                servo_id: self.controller.servo(servo_id)
                for servo_id in self.servo_ids
            }

    def move_sync(self, positions: dict[int, int], time_ms: int) -> None:
        """Prepare a synchronized move and start all selected servos."""
        unknown = set(positions).difference(self.servo_ids)
        if unknown:
            raise ValueError(f"Unknown servo IDs: {sorted(unknown)}")
        self.last_positions.update(positions)
        if self.dry_run:
            return
        assert self.controller is not None
        for servo_id, position in positions.items():
            self.servos[servo_id].move_prepare(position, time_ms)
        self.controller.move_start()

    def stop(self) -> None:
        """Stop any move currently in progress."""
        if not self.dry_run and self.controller is not None:
            self.controller.move_stop()

    def set_torque(self, enabled: bool) -> None:
        """Enable or disable servo torque."""
        if self.dry_run:
            return
        for servo in self.servos.values():
            if enabled:
                servo.motor_on()
            else:
                servo.motor_off()

    def read_positions(self) -> dict[int, int]:
        """Read raw positions from all configured servos."""
        if self.dry_run:
            return dict(self.last_positions)
        positions = {
            servo_id: int(servo.get_position())
            for servo_id, servo in self.servos.items()
        }
        self.last_positions.update(positions)
        return positions

    def close(self) -> None:
        """Stop movement and release the serial port."""
        self.stop()
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        self.controller = None
        self.servos = {}


class CassandraServoController(LifecycleNode):
    """Lifecycle node controlling Cassandra's eleven LX-16A servos."""

    def __init__(self) -> None:
        super().__init__("cassandra_servo_controller")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("servo_ids", list(DEFAULT_SERVO_IDS))
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("home_positions", list(DEFAULT_HOME_POSITIONS))
        self.declare_parameter("directions", list(DEFAULT_DIRECTIONS))
        self.declare_parameter("raw_min", 0)
        self.declare_parameter("raw_max", 1000)
        self.declare_parameter("radians_per_unit", LX16A_RADIANS_PER_UNIT)
        self.declare_parameter("move_time_ms", 500)
        self.declare_parameter("pose_move_time_ms", 1000)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("state_publish_rate_hz", 20.0)
        self.declare_parameter("read_positions", True)
        self.declare_parameter("move_to_home_on_activate", False)
        self.declare_parameter("torque_off_on_deactivate", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter(
            "forward_command_topic", "/forward_position_controller/commands"
        )
        self.declare_parameter(
            "forward_joint_names", list(DEFAULT_FORWARD_JOINT_NAMES)
        )
        self.declare_parameter(
            "raw_command_topic", "/ros_cassandra/servos/raw_commands"
        )
        self.declare_parameter("pose_topic", "/ros_cassandra/servos/pose")
        self.declare_parameter(
            "joint_state_topic", "/joint_states"
        )

        self.bus: LX16AServoBus | None = None
        self.forward_subscription = None
        self.raw_subscription = None
        self.pose_subscription = None
        self.state_publisher = None
        self.write_timer = None
        self.read_timer = None
        self.active = False
        self.pending_positions: dict[int, int] | None = None
        self.pending_time_ms = 0

        self.servo_ids: tuple[int, ...] = ()
        self.joint_names: tuple[str, ...] = ()
        self.home_positions: tuple[int, ...] = ()
        self.directions: tuple[float, ...] = ()
        self.forward_joint_names: tuple[str, ...] = ()
        self.name_to_index: dict[str, int] = {}
        self.raw_min = 0
        self.raw_max = 1000
        self.radians_per_unit = LX16A_RADIANS_PER_UNIT
        self.move_time_ms = 500
        self.pose_move_time_ms = 1000
        self.read_positions_enabled = True
        self.torque_off_on_deactivate = False

        self.get_logger().info(
            "Created in UNCONFIGURED state; configure and activate before use"
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate configuration, open serial, and create ROS interfaces."""
        del state
        try:
            self._load_and_validate_parameters()
            self.bus = LX16AServoBus(
                port=str(self.get_parameter("serial_port").value),
                baudrate=int(self.get_parameter("baudrate").value),
                timeout=float(self.get_parameter("serial_timeout").value),
                servo_ids=self.servo_ids,
                dry_run=bool(self.get_parameter("dry_run").value),
            )
            self.forward_subscription = self.create_subscription(
                Float64MultiArray,
                str(self.get_parameter("forward_command_topic").value),
                self.on_forward_command,
                10,
            )
            self.raw_subscription = self.create_subscription(
                Int32MultiArray,
                str(self.get_parameter("raw_command_topic").value),
                self.on_raw_command,
                10,
            )
            self.pose_subscription = self.create_subscription(
                String,
                str(self.get_parameter("pose_topic").value),
                self.on_pose_command,
                10,
            )
            self.state_publisher = self.create_publisher(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                10,
            )

            control_rate = float(self.get_parameter("control_rate_hz").value)
            state_rate = float(
                self.get_parameter("state_publish_rate_hz").value
            )
            if control_rate <= 0.0 or state_rate <= 0.0:
                raise ValueError("Control and state rates must be positive")
            self.write_timer = self.create_timer(
                1.0 / control_rate, self.write
            )
            self.read_timer = self.create_timer(
                1.0 / state_rate, self.read
            )
            self.write_timer.cancel()
            self.read_timer.cancel()
            self.get_logger().info(
                f"Configured {len(self.servo_ids)} servos on "
                f"{self.get_parameter('serial_port').value}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Configuration failed: {exc}")
            self._release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Enable callbacks and start the read/write loops."""
        del state
        if self.bus is None or self.write_timer is None or self.read_timer is None:
            return TransitionCallbackReturn.FAILURE
        try:
            self.pending_positions = None
            self.active = True
            self.write_timer.reset()
            self.read_timer.reset()
            if bool(self.get_parameter("move_to_home_on_activate").value):
                self._queue_positions(
                    dict(zip(self.servo_ids, self.home_positions)),
                    self.pose_move_time_ms,
                )
            self.get_logger().info("Servo controller is ACTIVE")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.active = False
            self.get_logger().error(f"Activation failed: {exc}")
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop current movement and optionally release servo torque."""
        del state
        self.active = False
        self._cancel_timers()
        try:
            if self.bus is not None:
                self.bus.stop()
                if self.torque_off_on_deactivate:
                    self.bus.set_torque(False)
            self.pending_positions = None
            self.get_logger().info("Servo controller is INACTIVE")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Deactivation failed: {exc}")
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Close serial and return to UNCONFIGURED."""
        del state
        try:
            self.active = False
            self._release_resources()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Cleanup failed: {exc}")
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Safely release hardware from any lifecycle state."""
        del state
        return self._shutdown_transition()

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Stop and release hardware after a lifecycle error."""
        del state
        return self._shutdown_transition()

    def on_forward_command(self, message: Float64MultiArray) -> None:
        """Bridge forward_position_controller commands to physical servos."""
        if not self.active:
            return
        if len(message.data) != len(self.forward_joint_names):
            self.get_logger().warning(
                "forward_position_controller command requires exactly "
                f"{len(self.forward_joint_names)} positions, got "
                f"{len(message.data)}"
            )
            return
        try:
            positions: dict[int, int] = {}
            for name, radians in zip(
                self.forward_joint_names, message.data
            ):
                index = self.name_to_index[name]
                positions[self.servo_ids[index]] = radians_to_raw(
                    radians=float(radians),
                    home_raw=self.home_positions[index],
                    direction=self.directions[index],
                    radians_per_unit=self.radians_per_unit,
                    raw_min=self.raw_min,
                    raw_max=self.raw_max,
                )
            self._queue_positions(positions, self.move_time_ms)
        except (KeyError, ValueError) as exc:
            self.get_logger().warning(f"Invalid forward command: {exc}")

    def on_raw_command(self, message: Int32MultiArray) -> None:
        """Accept `[servo_id, raw_position, ...]` calibration commands."""
        if not self.active:
            return
        if not message.data or len(message.data) % 2:
            self.get_logger().warning(
                "Raw command must contain servo_id/position pairs"
            )
            return
        try:
            positions = {
                int(message.data[index]): clamp_raw(
                    int(message.data[index + 1]), self.raw_min, self.raw_max
                )
                for index in range(0, len(message.data), 2)
            }
            unknown = set(positions).difference(self.servo_ids)
            if unknown:
                raise ValueError(f"Unknown servo IDs: {sorted(unknown)}")
            self._queue_positions(positions, self.move_time_ms)
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def on_pose_command(self, message: String) -> None:
        """Queue an atomic named pose from servo_movement.py."""
        if not self.active:
            return
        pose_name = message.data.strip().lower()
        if pose_name not in POSES:
            self.get_logger().warning(
                f"Unknown pose '{pose_name}'. Available: {', '.join(POSES)}"
            )
            return
        positions = {
            servo_id: clamp_raw(position, self.raw_min, self.raw_max)
            for servo_id, position in POSES[pose_name].items()
            if servo_id in self.servo_ids
        }
        self._queue_positions(positions, self.pose_move_time_ms)

    def write(self) -> None:
        """Send the latest queued command as one synchronized move."""
        if not self.active or self.bus is None or self.pending_positions is None:
            return
        positions = self.pending_positions
        time_ms = self.pending_time_ms
        self.pending_positions = None
        try:
            self.bus.move_sync(positions, time_ms)
        except Exception as exc:
            self.get_logger().error(f"Servo write failed: {exc}")
            self.active = False
            self._cancel_timers()
            try:
                self.bus.stop()
            except Exception as stop_exc:
                self.get_logger().error(f"Servo stop also failed: {stop_exc}")

    def read(self) -> None:
        """Read servo positions and publish them as JointState radians."""
        if (
            not self.active
            or not self.read_positions_enabled
            or self.bus is None
            or self.state_publisher is None
        ):
            return
        try:
            raw_positions = self.bus.read_positions()
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self.joint_names)
            message.position = [
                raw_to_radians(
                    raw_positions.get(servo_id, self.home_positions[index]),
                    self.home_positions[index],
                    self.directions[index],
                    self.radians_per_unit,
                )
                for index, servo_id in enumerate(self.servo_ids)
            ]
            self.state_publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"Servo read failed: {exc}")

    def _load_and_validate_parameters(self) -> None:
        self.servo_ids = tuple(
            int(value) for value in self.get_parameter("servo_ids").value
        )
        self.joint_names = tuple(
            str(value) for value in self.get_parameter("joint_names").value
        )
        self.home_positions = tuple(
            int(value) for value in self.get_parameter("home_positions").value
        )
        self.directions = tuple(
            float(value) for value in self.get_parameter("directions").value
        )
        self.forward_joint_names = tuple(
            str(value)
            for value in self.get_parameter("forward_joint_names").value
        )
        lengths = {
            len(self.servo_ids),
            len(self.joint_names),
            len(self.home_positions),
            len(self.directions),
        }
        if len(lengths) != 1 or not self.servo_ids:
            raise ValueError(
                "servo_ids, joint_names, home_positions and directions "
                "must have the same non-zero length"
            )
        if len(set(self.servo_ids)) != len(self.servo_ids):
            raise ValueError("servo_ids must be unique")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")
        if any(direction not in (-1.0, 1.0) for direction in self.directions):
            raise ValueError("Every direction must be -1.0 or 1.0")

        self.raw_min = int(self.get_parameter("raw_min").value)
        self.raw_max = int(self.get_parameter("raw_max").value)
        if not 0 <= self.raw_min < self.raw_max <= 1000:
            raise ValueError("Require 0 <= raw_min < raw_max <= 1000")
        if any(
            not self.raw_min <= position <= self.raw_max
            for position in self.home_positions
        ):
            raise ValueError("Every home position must be within raw limits")

        self.radians_per_unit = float(
            self.get_parameter("radians_per_unit").value
        )
        if not math.isfinite(self.radians_per_unit) or self.radians_per_unit <= 0:
            raise ValueError("radians_per_unit must be finite and positive")
        self.move_time_ms = int(self.get_parameter("move_time_ms").value)
        self.pose_move_time_ms = int(
            self.get_parameter("pose_move_time_ms").value
        )
        if not 0 <= self.move_time_ms <= 30000:
            raise ValueError("move_time_ms must be between 0 and 30000")
        if not 0 <= self.pose_move_time_ms <= 30000:
            raise ValueError("pose_move_time_ms must be between 0 and 30000")

        self.read_positions_enabled = bool(
            self.get_parameter("read_positions").value
        )
        self.torque_off_on_deactivate = bool(
            self.get_parameter("torque_off_on_deactivate").value
        )
        self.name_to_index = {
            name: index for index, name in enumerate(self.joint_names)
        }
        if not self.forward_joint_names:
            raise ValueError("forward_joint_names must not be empty")
        unknown_forward_joints = set(self.forward_joint_names).difference(
            self.name_to_index
        )
        if unknown_forward_joints:
            raise ValueError(
                "forward_joint_names contains unknown joints: "
                f"{sorted(unknown_forward_joints)}"
            )
        if len(set(self.forward_joint_names)) != len(
            self.forward_joint_names
        ):
            raise ValueError("forward_joint_names must be unique")

    def _queue_positions(
        self, positions: dict[int, int], time_ms: int
    ) -> None:
        if positions:
            self.pending_positions = dict(positions)
            self.pending_time_ms = time_ms

    def _cancel_timers(self) -> None:
        if self.write_timer is not None:
            self.write_timer.cancel()
        if self.read_timer is not None:
            self.read_timer.cancel()

    def _release_resources(self) -> None:
        self._cancel_timers()
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        for attribute in (
            "write_timer",
            "read_timer",
        ):
            timer = getattr(self, attribute)
            if timer is not None:
                self.destroy_timer(timer)
                setattr(self, attribute, None)
        for attribute in (
            "forward_subscription",
            "raw_subscription",
            "pose_subscription",
        ):
            subscription = getattr(self, attribute)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, attribute, None)
        if self.state_publisher is not None:
            self.destroy_publisher(self.state_publisher)
            self.state_publisher = None
        self.pending_positions = None

    def _shutdown_transition(self) -> TransitionCallbackReturn:
        self.active = False
        try:
            self._release_resources()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Shutdown failed: {exc}")
            return TransitionCallbackReturn.ERROR

    def emergency_shutdown(self) -> None:
        """Best-effort process-level cleanup."""
        self.active = False
        try:
            self._release_resources()
        except Exception as exc:
            self.get_logger().error(f"Emergency shutdown failed: {exc}")


def main(args: list[str] | None = None) -> None:
    """Run the lifecycle servo controller."""
    rclpy.init(args=args)
    node: CassandraServoController | None = None
    try:
        node = CassandraServoController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.emergency_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
