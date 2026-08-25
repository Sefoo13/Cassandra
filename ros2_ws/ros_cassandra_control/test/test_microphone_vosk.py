#!/usr/bin/env python3
"""Test microphone input and Vosk recognition without ROS 2."""

import argparse
import json
import os
import queue
import sys
import time

import numpy as np
import sounddevice as sd
from vosk import KaldiRecognizer, Model, SetLogLevel


def resolve_model_path(configured_path):
    def is_model(path):
        return os.path.isfile(os.path.join(path, "am", "final.mdl"))

    for current_path, directories, _files in os.walk(configured_path):
        relative_path = os.path.relpath(current_path, configured_path)
        depth = 0 if relative_path == "." else relative_path.count(os.sep) + 1
        if is_model(current_path):
            return current_path
        if depth >= 2:
            directories.clear()
    raise RuntimeError(
        f"No Vosk model found under {configured_path!r}; expected am/final.mdl"
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/models/vosk")
    parser.add_argument("--device", default="USB PnP Audio Device")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--list-devices", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_devices:
        print(sd.query_devices())
        return

    model_path = resolve_model_path(args.model)
    device_info = sd.query_devices(args.device, "input")
    print(f"Model: {model_path}")
    print(f"Input device: {device_info['name']}")
    print(f"Default sample rate: {device_info['default_samplerate']}")
    print(f"Testing for {args.duration:g} seconds. Speak, then pause.")

    SetLogLevel(-1)
    recognizer = KaldiRecognizer(Model(model_path), args.sample_rate)
    audio_queue = queue.Queue()

    def audio_callback(input_data, _frames, _time_info, status):
        if status:
            print(f"Audio status: {status}", file=sys.stderr)
        audio_queue.put(bytes(input_data))

    deadline = time.monotonic() + args.duration
    last_level_report = 0.0
    last_partial = ""

    with sd.RawInputStream(
        samplerate=args.sample_rate,
        blocksize=4000,
        device=args.device,
        dtype="int16",
        channels=1,
        callback=audio_callback,
    ):
        while time.monotonic() < deadline:
            try:
                audio = audio_queue.get(timeout=0.5)
            except queue.Empty:
                print("No audio block received", file=sys.stderr)
                continue

            samples = np.frombuffer(audio, dtype=np.int16)
            now = time.monotonic()
            if now - last_level_report >= 1.0:
                peak = int(np.max(np.abs(samples))) if samples.size else 0
                rms = float(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))
                print(f"Audio level: peak={peak:5d}, rms={rms:8.1f}")
                last_level_report = now

            if recognizer.AcceptWaveform(audio):
                text = json.loads(recognizer.Result()).get("text", "").strip()
                if text:
                    print(f"FINAL: {text}")
                last_partial = ""
            else:
                partial = json.loads(recognizer.PartialResult()).get(
                    "partial", ""
                ).strip()
                if partial and partial != last_partial:
                    print(f"PARTIAL: {partial}")
                    last_partial = partial

    final_text = json.loads(recognizer.FinalResult()).get("text", "").strip()
    if final_text:
        print(f"FINAL: {final_text}")
    print("Test finished")


if __name__ == "__main__":
    main()
