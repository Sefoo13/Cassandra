#!/usr/bin/env python3
"""Continuously recognize microphone speech and publish completed phrases."""

import json
import os
import queue
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ros_cassandra_control.srv import Speak

try:
    import sounddevice as sd
    from vosk import KaldiRecognizer, Model
except ImportError as error:
    raise RuntimeError(
        "Missing speech recognition dependency. Install 'sounddevice' and 'vosk'."
    ) from error


class SpeechRecognitionNode(Node):
    """Capture mono audio from a microphone and publish Vosk transcripts."""

    def __init__(self):
        super().__init__(
            "speech_recognition",
            automatically_declare_parameters_from_overrides=True,
        )

        model_path = str(self.get_parameter("model_path").value)
        if not os.path.isdir(model_path):
            raise RuntimeError(
                f"Vosk model was not found at {model_path!r}. "
                "Set model_path in config/speech_recognition.yaml to an "
                "unpacked Vosk model directory."
            )

        self._sample_rate = int(self.get_parameter("sample_rate").value)
        self._block_size = int(self.get_parameter("block_size").value)
        topic = str(self.get_parameter("topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        configured_device = str(self.get_parameter("audio_device").value).strip()
        self._audio_device = configured_device or None
        self._input_channels = max(
            1,
            int(self.get_parameter("input_channels").value),
        )
        self._selected_channel = int(
            self.get_parameter("selected_channel").value
        )
        if not 0 <= self._selected_channel < self._input_channels:
            raise RuntimeError(
                "selected_channel must be between 0 and input_channels - 1"
            )
        self._configure_mixer()
        self._input_gain = max(
            0.1,
            float(self.get_parameter("input_gain").value),
        )
        self._speak_recognized_text = bool(
            self.get_parameter("speak_recognized_text").value
        )
        self._playback_recognized_audio = bool(
            self.get_parameter("playback_recognized_audio").value
        )
        configured_output_device = str(
            self.get_parameter("playback_output_device").value
        ).strip()
        self._playback_output_device = configured_output_device or None
        self._playback_max_seconds = max(
            1.0,
            float(self.get_parameter("playback_max_seconds").value),
        )
        speak_service = str(self.get_parameter("speak_service").value)
        speaking_topic = str(self.get_parameter("speaking_topic").value)
        self._listen_cooldown = float(
            self.get_parameter("listen_cooldown_seconds").value
        )
        self._wake_word_enabled = bool(
            self.get_parameter("wake_word_enabled").value
        )
        self._wake_words = tuple(
            str(word).lower().strip()
            for word in self.get_parameter("wake_words").value
            if str(word).strip()
        )
        self._wake_acknowledgement = str(
            self.get_parameter("wake_acknowledgement").value
        ).strip()
        self._command_mode_persistent = bool(
            self.get_parameter("command_mode_persistent").value
        )
        self._command_exit_words = tuple(
            str(word).lower().strip()
            for word in self.get_parameter("command_exit_words").value
            if str(word).strip()
        )
        self._command_exit_acknowledgement = str(
            self.get_parameter("command_exit_acknowledgement").value
        ).strip()
        self._command_window = max(
            0.1,
            float(self.get_parameter("command_window_seconds").value),
        )
        self._always_active_commands = tuple(
            str(command).lower().strip()
            for command in self.get_parameter("always_active_commands").value
            if str(command).strip()
        )

        self._publisher = self.create_publisher(String, topic, 10)
        self._command_publisher = self.create_publisher(String, command_topic, 10)
        self._speak_client = self.create_client(Speak, speak_service)
        self._speaking_subscription = self.create_subscription(
            Bool,
            speaking_topic,
            self._on_speaking_state,
            10,
        )
        self._recognizer = KaldiRecognizer(Model(model_path), self._sample_rate)
        self._audio_queue = queue.Queue(maxsize=20)
        self._stop_event = threading.Event()
        self._speaking = threading.Event()
        self._playing_back = threading.Event()
        self._external_speaking = threading.Event()
        self._pending_command_window = False
        self._resume_listening_at = 0.0
        self._command_mode_until = 0.0
        self._stream = None
        self._worker = threading.Thread(target=self._recognize, daemon=True)

        try:
            self._stream = sd.RawInputStream(
                samplerate=self._sample_rate,
                blocksize=self._block_size,
                device=self._audio_device,
                dtype="int16",
                channels=self._input_channels,
                callback=self._on_audio,
            )
            self._stream.start()
        except Exception as error:
            raise RuntimeError(f"Cannot open the microphone: {error}") from error

        self._worker.start()
        device_name = self._audio_device or "system default"
        self.get_logger().info(
            f"Listening on {device_name}, input channel "
            f"{self._selected_channel + 1}/{self._input_channels}; "
            f"completed phrases publish to {topic}"
        )

    def _configure_mixer(self):
        """Set the configured ALSA mixer level before opening the microphone."""
        if not bool(self.get_parameter("mixer_enabled").value):
            return

        card = max(0, int(self.get_parameter("mixer_card").value))
        control = str(self.get_parameter("mixer_control").value).strip()
        element = max(0, int(self.get_parameter("mixer_element").value))
        volume = min(
            100,
            max(0, int(self.get_parameter("mixer_volume_percent").value)),
        )
        if not control:
            self.get_logger().warning(
                "ALSA mixer setup skipped: mixer_control is empty"
            )
            return

        mixer_name = f"{control},{element}"
        command = [
            "amixer",
            "-c",
            str(card),
            "sset",
            mixer_name,
            f"{volume}%",
        ]
        try:
            subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
            )
        except (OSError, subprocess.CalledProcessError) as error:
            details = getattr(error, "stderr", "") or str(error)
            self.get_logger().warning(
                f"Cannot set ALSA mixer {mixer_name!r}: {details.strip()}"
            )
            return

        self.get_logger().info(
            f"ALSA card {card} mixer {mixer_name!r} set to {volume}%"
        )

    def _on_audio(self, input_data, _frames, _time_info, status):
        if status:
            self.get_logger().warning(f"Microphone status: {status}")
        if (
            self._speaking.is_set()
            or self._playing_back.is_set()
            or self._external_speaking.is_set()
            or time.monotonic() < self._resume_listening_at
        ):
            return
        try:
            samples = np.frombuffer(input_data, dtype=np.int16)
            if self._input_channels > 1:
                complete_frames = samples.size // self._input_channels
                samples = samples[: complete_frames * self._input_channels]
                samples = samples.reshape(-1, self._input_channels)[
                    :, self._selected_channel
                ]
            if self._input_gain != 1.0:
                samples = samples.astype(np.float32) * self._input_gain
                samples = np.clip(samples, -32768, 32767).astype(np.int16)
            audio = np.ascontiguousarray(samples).tobytes()
            self._audio_queue.put_nowait(audio)
        except queue.Full:
            self.get_logger().warning("Audio queue is full; dropping an audio block")

    def _recognize(self):
        phrase_audio = bytearray()
        max_audio_bytes = int(
            self._sample_rate * 2 * self._playback_max_seconds
        )
        while not self._stop_event.is_set():
            try:
                audio = self._audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            phrase_audio.extend(audio)
            if len(phrase_audio) > max_audio_bytes:
                del phrase_audio[: len(phrase_audio) - max_audio_bytes]

            if not self._recognizer.AcceptWaveform(audio):
                continue

            try:
                transcript = json.loads(self._recognizer.Result()).get("text", "")
            except json.JSONDecodeError as error:
                self.get_logger().warning(f"Invalid Vosk result: {error}")
                continue

            transcript = transcript.strip()
            if not transcript:
                phrase_audio.clear()
                continue

            captured_audio = bytes(phrase_audio)
            phrase_audio.clear()
            if self._playback_recognized_audio:
                self._play_audio(captured_audio)

            if self._route_wake_command(transcript):
                continue

            self._publish_text(self._publisher, transcript)
            self.get_logger().info(f"Recognized: {transcript!r}")
            if self._speak_recognized_text:
                self._speak(transcript)

    def _play_audio(self, audio):
        """Play captured mono PCM while temporarily ignoring microphone input."""
        if not audio:
            return

        self._playing_back.set()
        self._clear_audio_queue()
        try:
            output_device = self._resolve_playback_output_device()
            output_info = sd.query_devices(
                output_device,
                "output",
            )
            output_sample_rate = int(
                round(float(output_info["default_samplerate"]))
            )
            playback_audio = self._resample_audio(
                audio,
                self._sample_rate,
                output_sample_rate,
            )
            with sd.RawOutputStream(
                samplerate=output_sample_rate,
                device=output_device,
                dtype="int16",
                channels=1,
            ) as stream:
                stream.write(playback_audio)
            output_name = str(output_info["name"])
            self.get_logger().info(
                f"Played captured phrase on {output_name} at "
                f"{output_sample_rate} Hz"
            )
        except Exception as error:
            self.get_logger().warning(
                f"Cannot play captured microphone audio: {error}"
            )
        finally:
            self._resume_listening_at = (
                time.monotonic() + self._listen_cooldown
            )
            self._playing_back.clear()

    def _resolve_playback_output_device(self):
        """Prefer an XVF3800 output when no playback device is configured."""
        if self._playback_output_device is not None:
            return self._playback_output_device

        devices = sd.query_devices()
        for index, device in enumerate(devices):
            name = str(device.get("name", "")).lower()
            if (
                int(device.get("max_output_channels", 0)) > 0
                and ("xvf3800" in name or "respeaker" in name)
            ):
                self.get_logger().info(
                    f"Automatically selected playback device: "
                    f"{device['name']} (index {index})"
                )
                return index

        self.get_logger().warning(
            "No ReSpeaker/XVF3800 output found; using the system default"
        )
        return None

    @staticmethod
    def _resample_audio(audio, source_rate, target_rate):
        """Linearly resample mono signed 16-bit PCM for diagnostic playback."""
        if source_rate == target_rate:
            return audio

        samples = np.frombuffer(audio, dtype=np.int16)
        if samples.size < 2:
            return audio

        output_size = max(
            1,
            int(round(samples.size * target_rate / source_rate)),
        )
        source_positions = np.arange(samples.size, dtype=np.float64)
        target_positions = np.linspace(
            0,
            samples.size - 1,
            output_size,
            dtype=np.float64,
        )
        resampled = np.interp(
            target_positions,
            source_positions,
            samples,
        )
        return np.clip(resampled, -32768, 32767).astype(np.int16).tobytes()

    def _clear_audio_queue(self):
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

    def _route_wake_command(self, transcript):
        if not self._wake_word_enabled:
            return False

        now = time.monotonic()
        normalized = transcript.lower().strip()
        if (
            now <= self._command_mode_until
            and normalized in self._command_exit_words
        ):
            self._command_mode_until = 0.0
            self._pending_command_window = False
            self.get_logger().info("Command mode deactivated")
            if self._command_exit_acknowledgement:
                self._speak(self._command_exit_acknowledgement)
            return True

        if normalized in self._always_active_commands:
            self._publish_command(normalized)
            if not self._command_mode_persistent:
                self._command_mode_until = 0.0
            return True

        for wake_word in self._wake_words:
            match = re.search(rf"(?<!\w){re.escape(wake_word)}(?!\w)", normalized)
            if match is None:
                continue

            command = (normalized[: match.start()] + normalized[match.end() :])
            command = re.sub(r"^[\s,.:;!?-]+|[\s,.:;!?-]+$", "", command)
            self.get_logger().info(f"Wake word detected: {wake_word!r}")
            if command:
                if self._wake_acknowledgement:
                    self._speak(self._wake_acknowledgement)
                self._publish_command(command)
                self._activate_command_window()
            elif self._wake_acknowledgement:
                self._command_mode_until = float("inf")
                self._pending_command_window = True
                if not self._speak(self._wake_acknowledgement):
                    self._pending_command_window = False
                    self._activate_command_window()
            else:
                self._activate_command_window()
            return True

        if now <= self._command_mode_until:
            self._publish_command(transcript)
            if not self._command_mode_persistent:
                self._command_mode_until = 0.0
            return True

        self._command_mode_until = 0.0
        return False

    def _publish_command(self, command):
        self._publish_text(self._command_publisher, command)
        self.get_logger().info(f"Voice command: {command!r}")

    def _activate_command_window(self):
        if self._command_mode_persistent:
            self._command_mode_until = float("inf")
            self.get_logger().info(
                "Command mode activated; waiting for an exit phrase"
            )
        else:
            self._command_mode_until = time.monotonic() + self._command_window
            self.get_logger().info(
                f"Waiting for one voice command for {self._command_window:.1f}s"
            )

    @staticmethod
    def _publish_text(publisher, text):
        message = String()
        message.data = text
        publisher.publish(message)

    def _on_speaking_state(self, message):
        if message.data:
            self._external_speaking.set()
            return
        self._resume_listening_at = time.monotonic() + self._listen_cooldown
        self._external_speaking.clear()
        if self._pending_command_window:
            self._pending_command_window = False
            self._activate_command_window()

    def _speak(self, text):
        if self._speaking.is_set():
            self.get_logger().warning("Speech is already in progress; skipping text")
            return False
        if not self._speak_client.service_is_ready():
            self.get_logger().error("The /speak service is not available")
            return False

        self._speaking.set()
        self._clear_audio_queue()

        request = Speak.Request()
        request.text = text
        future = self._speak_client.call_async(request)
        future.add_done_callback(self._on_speech_finished)
        return True

    def _on_speech_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Speech request queued")
            else:
                self.get_logger().error(
                    f"Text-to-speech failed: {response.message}"
                )
                if self._pending_command_window:
                    self._pending_command_window = False
                    self._activate_command_window()
        except Exception as error:
            self.get_logger().error(f"Cannot call /speak: {error}")
            if self._pending_command_window:
                self._pending_command_window = False
                self._activate_command_window()
        finally:
            self._speaking.clear()

    def destroy_node(self):
        self._stop_event.set()
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"speech_recognition: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
