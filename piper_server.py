#!/usr/bin/env python3
"""Small HTTP wrapper around Piper with serialized ALSA playback."""

import json
import os
import subprocess
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HOST = os.getenv("PIPER_API_HOST", "127.0.0.1")
PORT = int(os.getenv("PIPER_API_PORT", "8000"))
PIPER_BIN = os.getenv("PIPER_BIN", "/app/piper/piper")
PIPER_MODEL = os.getenv("PIPER_MODEL", "/voices/uk_UA-lada-x_low.onnx")
PULSE_SINK = os.getenv("PULSE_SINK", "0")
MAX_TEXT_LENGTH = int(os.getenv("PIPER_MAX_TEXT_LENGTH", "1000"))
PIPER_TIMEOUT = float(os.getenv("PIPER_TIMEOUT_SECONDS", "120"))

playback_lock = threading.Lock()


class PiperRequestHandler(BaseHTTPRequestHandler):
    server_version = "CassandraPiper/1.0"

    def _json_response(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != "/health":
            self._json_response(404, {"success": False, "message": "Not found"})
            return

        healthy = os.path.isfile(PIPER_BIN) and os.path.isfile(PIPER_MODEL)
        self._json_response(
            200 if healthy else 503,
            {
                "success": healthy,
                "piper": PIPER_BIN,
                "model": PIPER_MODEL,
            },
        )

    def do_POST(self):
        if self.path != "/speak":
            self._json_response(404, {"success": False, "message": "Not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > 16_384:
                raise ValueError("Invalid request size")
            payload = json.loads(self.rfile.read(content_length))
            if not isinstance(payload, dict):
                raise ValueError("Request body must be a JSON object")
            text = payload.get("text", "")
            if not isinstance(text, str):
                raise ValueError("'text' must be a string")
            # Piper treats each stdin line as a separate utterance. With one
            # --output_file that can leave only the first line in the WAV, so
            # turn multiline requests from Foxglove into one utterance.
            text = " ".join(text.split())
            if not text:
                raise ValueError("'text' must not be empty")
            if len(text) > MAX_TEXT_LENGTH:
                raise ValueError(
                    f"'text' exceeds the {MAX_TEXT_LENGTH} character limit"
                )
            if text[-1] not in ".!?…":
                text += "."
        except (ValueError, json.JSONDecodeError) as error:
            self._json_response(400, {"success": False, "message": str(error)})
            return

        output_path = None
        try:
            with playback_lock:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as output:
                    output_path = output.name

                piper = subprocess.run(
                    [
                        PIPER_BIN,
                        "--model",
                        PIPER_MODEL,
                        "--json-input",
                    ],
                    input=(
                        json.dumps(
                            {"text": text, "output_file": output_path},
                            ensure_ascii=False,
                        )
                        + "\n"
                    ).encode("utf-8"),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=PIPER_TIMEOUT,
                    check=False,
                )
                if piper.returncode != 0:
                    detail = piper.stderr.decode("utf-8", errors="replace").strip()
                    raise RuntimeError(f"Piper failed: {detail or piper.returncode}")

                self.log_message(
                    "synthesized %d characters",
                    len(text),
                )

                pulse = subprocess.run(
                    ["pactl", "set-default-sink", PULSE_SINK],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=10,
                    check=False,
                )
                if pulse.returncode != 0:
                    detail = pulse.stderr.decode("utf-8", errors="replace").strip()
                    raise RuntimeError(
                        f"pactl set-default-sink {PULSE_SINK} failed: "
                        f"{detail or pulse.returncode}"
                    )

                player = subprocess.run(
                    ["paplay", output_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=PIPER_TIMEOUT,
                    check=False,
                )
                if player.returncode != 0:
                    detail = player.stderr.decode("utf-8", errors="replace").strip()
                    raise RuntimeError(f"paplay failed: {detail or player.returncode}")

            self._json_response(
                200,
                {
                    "success": True,
                    "message": f"Speech played: {len(text)} characters",
                },
            )
        except subprocess.TimeoutExpired:
            self._json_response(
                504, {"success": False, "message": "Speech operation timed out"}
            )
        except (OSError, RuntimeError) as error:
            self._json_response(500, {"success": False, "message": str(error)})
        finally:
            if output_path:
                try:
                    os.unlink(output_path)
                except FileNotFoundError:
                    pass

    def log_message(self, format_string, *args):
        print(
            f"{self.address_string()} - {format_string % args}",
            flush=True,
        )


if __name__ == "__main__":
    print(
        f"Piper API listening on http://{HOST}:{PORT}; "
        f"model={PIPER_MODEL}; PulseAudio sink={PULSE_SINK}",
        flush=True,
    )
    ThreadingHTTPServer((HOST, PORT), PiperRequestHandler).serve_forever()
