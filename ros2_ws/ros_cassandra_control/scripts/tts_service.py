#!/usr/bin/env python3
"""ROS 2 service that forwards speech requests to the Piper HTTP API."""

import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from ros_cassandra_control.srv import Speak


class TextToSpeechService(Node):
    def __init__(self):
        super().__init__(
            "text_to_speech",
            automatically_declare_parameters_from_overrides=True,
        )
        speaking_topic = str(self.get_parameter("speaking_topic").value)
        self._speaking_publisher = self.create_publisher(Bool, speaking_topic, 10)
        self._service = self.create_service(Speak, "speak", self._speak)
        self.get_logger().info("Text-to-speech service is ready at /speak")

    def _speak(self, request, response):
        api_url = self.get_parameter("piper_api_url").value
        timeout = self.get_parameter("request_timeout_seconds").value
        text = request.text

        self.get_logger().info(
            f"Received speech request ({len(text)} characters): {text!r}"
        )
        body = json.dumps({"text": text}, ensure_ascii=False).encode("utf-8")
        http_request = Request(
            api_url,
            data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )

        self._publish_speaking(True)
        try:
            with urlopen(http_request, timeout=timeout) as http_response:
                result = json.loads(http_response.read().decode("utf-8"))
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "No message from Piper"))
        except HTTPError as error:
            try:
                result = json.loads(error.read().decode("utf-8"))
                detail = result.get("message", str(error))
            except (UnicodeDecodeError, json.JSONDecodeError):
                detail = str(error)
            response.success = False
            response.message = f"Piper API error: {detail}"
        except (URLError, TimeoutError, json.JSONDecodeError) as error:
            response.success = False
            response.message = f"Cannot call Piper API: {error}"
        finally:
            self._publish_speaking(False)

        if response.success:
            self.get_logger().info("Speech request completed")
        else:
            self.get_logger().error(response.message)
        return response

    def _publish_speaking(self, speaking):
        message = Bool()
        message.data = speaking
        self._speaking_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
