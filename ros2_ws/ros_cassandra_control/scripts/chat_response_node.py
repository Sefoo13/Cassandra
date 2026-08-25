#!/usr/bin/env python3
"""Generate short conversational responses and send them to speech synthesis."""

import os
import queue
import re
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros_cassandra_control.srv import Speak


class ChatResponseNode(Node):
    def __init__(self):
        super().__init__(
            "chat_response",
            automatically_declare_parameters_from_overrides=True,
        )

        self._enabled = bool(self.get_parameter("enabled").value)
        self._stop_event = threading.Event()
        self._worker = None
        if not self._enabled:
            self.get_logger().info("Chat responses are disabled by configuration")
            return

        from openai import OpenAI

        api_key = os.getenv("OPENAI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not configured")

        input_topic = str(self.get_parameter("input_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        speak_service = str(self.get_parameter("speak_service").value)
        self._model = str(self.get_parameter("model").value)
        self._reasoning_effort = str(
            self.get_parameter("reasoning_effort").value
        )
        self._max_words = int(self.get_parameter("max_words").value)
        self._max_output_tokens = int(
            self.get_parameter("max_output_tokens").value
        )
        self._remember_conversation = bool(
            self.get_parameter("remember_conversation").value
        )
        configured_instructions = str(self.get_parameter("instructions").value)
        self._instructions = (
            f"{configured_instructions}\n"
            f"Відповідай не більше ніж {self._max_words} словами."
        )

        self._client = OpenAI(api_key=api_key)
        self._response_publisher = self.create_publisher(String, response_topic, 10)
        self._speak_client = self.create_client(Speak, speak_service)
        self._subscription = self.create_subscription(
            String,
            input_topic,
            self._on_transcript,
            10,
        )
        self._requests = queue.Queue(maxsize=5)
        self._previous_response_id = None
        self._worker = threading.Thread(target=self._process_requests, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Chat responses are ready: {input_topic} -> {response_topic}"
        )

    def _on_transcript(self, message):
        text = message.data.strip()
        if not text:
            return
        try:
            self._requests.put_nowait(text)
        except queue.Full:
            self.get_logger().warning("Chat request queue is full; dropping text")

    def _process_requests(self):
        while not self._stop_event.is_set():
            try:
                transcript = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                response = self._client.responses.create(
                    model=self._model,
                    instructions=self._instructions,
                    input=self._format_input(transcript),
                    reasoning={"effort": self._reasoning_effort},
                    max_output_tokens=self._max_output_tokens,
                    previous_response_id=(
                        self._previous_response_id
                        if self._remember_conversation
                        else None
                    ),
                )
                answer = self._limit_words(response.output_text.strip())
                if not answer:
                    raise RuntimeError("OpenAI returned an empty response")
                if self._remember_conversation:
                    self._previous_response_id = response.id
            except Exception as error:
                self.get_logger().error(f"OpenAI request failed: {error}")
                continue

            message = String()
            message.data = answer
            self._response_publisher.publish(message)
            self.get_logger().info(f"Chat response: {answer!r}")
            self._speak(answer)

    @staticmethod
    def _format_input(transcript):
        return (
            "Розпізнаний голосовий запит користувача:\n"
            f"<user_request>{transcript}</user_request>\n"
            "Дай коротку відповідь, придатну для озвучення роботом."
        )

    def _limit_words(self, text):
        normalized = re.sub(r"\s+", " ", text).strip()
        words = normalized.split(" ")
        if len(words) <= self._max_words:
            return normalized
        return " ".join(words[: self._max_words]).rstrip(".,;:!?") + "."

    def _speak(self, text):
        if not self._speak_client.service_is_ready():
            self.get_logger().error("The /speak service is not available")
            return
        request = Speak.Request()
        request.text = text
        future = self._speak_client.call_async(request)
        future.add_done_callback(self._on_speech_finished)

    def _on_speech_finished(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"Text-to-speech failed: {response.message}"
                )
        except Exception as error:
            self.get_logger().error(f"Cannot call /speak: {error}")

    def destroy_node(self):
        self._stop_event.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ChatResponseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"chat_response: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
