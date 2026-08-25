#!/usr/bin/env python3
"""Execute a small, bounded set of Ukrainian voice commands."""

import re
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__(
            "voice_command",
            automatically_declare_parameters_from_overrides=True,
        )

        self._enabled = bool(self.get_parameter("enabled").value)
        transcript_topic = str(self.get_parameter("transcript_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        servo_pose_topic = str(self.get_parameter("servo_pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._servo_enabled = bool(
            self.get_parameter("servo_commands_enabled").value
        )
        self._movement_enabled = bool(
            self.get_parameter("movement_commands_enabled").value
        )
        self._linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self._reverse_speed = abs(
            float(self.get_parameter("reverse_speed").value)
        )
        self._angular_speed = abs(
            float(self.get_parameter("angular_speed").value)
        )
        self._movement_duration = max(
            0.1,
            float(self.get_parameter("movement_duration_seconds").value),
        )
        publish_rate = float(
            self.get_parameter("command_publish_rate_hz").value
        )
        if publish_rate <= 0.0:
            raise ValueError("command_publish_rate_hz must be positive")

        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._servo_publisher = self.create_publisher(
            String,
            servo_pose_topic,
            10,
        )
        self._velocity_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._subscription = self.create_subscription(
            String,
            transcript_topic,
            self._on_transcript,
            10,
        )
        self._movement_command = Twist()
        self._movement_deadline = 0.0
        self._movement_timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_movement,
        )
        self.get_logger().info(
            f"Voice commands are {'enabled' if self._enabled else 'disabled'}"
        )

    def _on_transcript(self, message):
        text = self._normalize(message.data)
        if not text:
            return

        if self._contains_any(text, ("стоп", "стій", "зупинись", "зупинися")):
            self._stop_movement("stop")
            return
        if not self._enabled:
            return

        servo_pose = self._match_servo_pose(text)
        if servo_pose is not None:
            if not self._servo_enabled:
                self._publish_status("rejected:servo_commands_disabled")
                return
            self._publish_servo_pose(servo_pose)
            return

        movement = self._match_movement(text)
        if movement is not None:
            if not self._movement_enabled:
                self._publish_status("rejected:movement_commands_disabled")
                return
            linear_x, angular_z, name = movement
            self._start_movement(linear_x, angular_z, name)

    @staticmethod
    def _normalize(text):
        normalized = text.lower().replace("’", "'").replace("`", "'")
        normalized = re.sub(r"[^а-яіїєґa-z0-9'\s-]", " ", normalized)
        return re.sub(r"\s+", " ", normalized).strip()

    @staticmethod
    def _contains_any(text, phrases):
        return any(phrase in text for phrase in phrases)

    def _match_servo_pose(self, text):
        is_head = "голов" in text
        is_torso = "торс" in text or "корпус" in text

        if is_head and self._contains_any(text, ("ліворуч", "вліво", "ліво")):
            return "head_left"
        if is_head and self._contains_any(text, ("праворуч", "вправо", "право")):
            return "head_right"
        if is_head and self._contains_any(
            text,
            ("вгору", "догори", "верх", "підніми", "підняти"),
        ):
            return "head_up"
        if is_head and self._contains_any(
            text,
            ("вниз", "донизу", "низ", "опусти", "опустити"),
        ):
            return "head_down"
        if is_head and self._contains_any(text, ("прямо", "рівно", "центр")):
            return "head_center"

        if is_torso and self._contains_any(text, ("ліворуч", "вліво", "ліво")):
            return "torso_left"
        if is_torso and self._contains_any(text, ("праворуч", "вправо", "право")):
            return "torso_right"
        if is_torso and self._contains_any(text, ("прямо", "рівно", "центр")):
            return "torso_center"

        if self._contains_any(text, ("склади руки", "скласти руки")):
            return "pray"
        if self._contains_any(text, ("підніми руки", "руки вгору")):
            return "crest"
        if self._contains_any(
            text,
            ("початкове положення", "базове положення", "рівно"),
        ):
            return "base"
        return None

    def _match_movement(self, text):
        if self._contains_any(
            text,
            ("назад", "їдь назад", "здай назад", "рухайся назад"),
        ):
            return -self._reverse_speed, 0.0, "backward"
        if self._contains_any(
            text,
            ("вперед", "уперед", "їдь вперед", "їдь уперед", "рухайся прямо"),
        ):
            return self._linear_speed, 0.0, "forward"
        if self._contains_any(text, ("ліворуч", "вліво", "поверни наліво")):
            return 0.0, self._angular_speed, "turn_left"
        if self._contains_any(text, ("праворуч", "вправо", "поверни направо")):
            return 0.0, -self._angular_speed, "turn_right"
        return None

    def _publish_servo_pose(self, pose):
        message = String()
        message.data = pose
        self._servo_publisher.publish(message)
        self._publish_status(f"executed:servo:{pose}")
        self.get_logger().info(f"Voice servo command: {pose}")

    def _start_movement(self, linear_x, angular_z, name):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self._movement_command = command
        self._movement_deadline = time.monotonic() + self._movement_duration
        self._velocity_publisher.publish(command)
        self._publish_status(f"executing:movement:{name}")
        self.get_logger().info(
            f"Voice movement command: {name} for {self._movement_duration:.1f}s"
        )

    def _publish_movement(self):
        if self._movement_deadline <= 0.0:
            return
        if time.monotonic() >= self._movement_deadline:
            self._stop_movement("completed")
            return
        self._velocity_publisher.publish(self._movement_command)

    def _stop_movement(self, reason):
        self._movement_deadline = 0.0
        self._movement_command = Twist()
        self._velocity_publisher.publish(self._movement_command)
        self._publish_status(f"stopped:movement:{reason}")
        self.get_logger().info(f"Movement stopped: {reason}")

    def _publish_status(self, status):
        message = String()
        message.data = status
        self._status_publisher.publish(message)

    def destroy_node(self):
        self._stop_movement("shutdown")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
