#!/usr/bin/env python3
"""Stream microphone speech to OpenVLA through /vla_instruction.

Flow:
    microphone PCM frames -> DashScope streaming ASR -> final sentence text
    -> ros2 topic pub /vla_instruction -> vla_gazebo_bridge.py -> OpenVLA

Run this on a machine that has a microphone/audio input device.
"""

from __future__ import annotations

import argparse
import os
import queue
import signal
import sys
import time
from pathlib import Path
from typing import Any

from asr_file_to_vla import (
    BEIJING_WS_URL,
    DEFAULT_MODEL,
    DEFAULT_ROS_SETUP,
    DEFAULT_SAMPLE_RATE,
    DEFAULT_TOPIC,
    INTERNATIONAL_WS_URL,
    build_ros_pub_command,
    publish_instruction,
    resolve_ros_setup,
)

FORMAT_PCM = "pcm"
DEFAULT_CHANNELS = 1
DEFAULT_BLOCK_SIZE = 3200


def import_pyaudio():
    try:
        import pyaudio
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing dependency: pyaudio. Install PortAudio and PyAudio first, for example:\n"
            "  sudo apt install portaudio19-dev python3-pyaudio\n"
            "  pip install pyaudio"
        ) from exc
    return pyaudio


def list_input_devices() -> None:
    pyaudio = import_pyaudio()
    mic = pyaudio.PyAudio()
    try:
        print("Available audio input devices:")
        found = False
        for index in range(mic.get_device_count()):
            info = mic.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) <= 0:
                continue
            found = True
            print(
                f"  [{index}] {info.get('name')} "
                f"channels={info.get('maxInputChannels')} "
                f"default_rate={info.get('defaultSampleRate')}"
            )
        if not found:
            print("  <none>")
    finally:
        mic.terminate()


def sentence_text(sentence: Any) -> str:
    if not isinstance(sentence, dict):
        return ""
    return str(sentence.get("text", "")).strip()


class VLARecognitionCallback:
    def __init__(self, result_queue: "queue.Queue[str]", stop_event, verbose_partial: bool) -> None:
        from dashscope.audio.asr import RecognitionCallback

        class Callback(RecognitionCallback):
            def on_open(inner_self) -> None:
                print("[ASR] recognition opened")

            def on_close(inner_self) -> None:
                print("[ASR] recognition closed")

            def on_complete(inner_self) -> None:
                print("[ASR] recognition completed")

            def on_error(inner_self, message) -> None:
                print(f"[ASR] error request_id={getattr(message, 'request_id', None)}")
                print(f"[ASR] error message={getattr(message, 'message', message)}")
                stop_event.set()

            def on_event(inner_self, result) -> None:
                from dashscope.audio.asr import RecognitionResult

                sentence = result.get_sentence()
                text = sentence_text(sentence)
                if not text:
                    return

                if verbose_partial:
                    print(f"[ASR] partial: {text}")

                if RecognitionResult.is_sentence_end(sentence):
                    print(f"[ASR] final: {text}")
                    result_queue.put(text)

        self.callback = Callback()


def open_input_stream(args: argparse.Namespace):
    pyaudio = import_pyaudio()
    mic = pyaudio.PyAudio()
    kwargs = {
        "format": pyaudio.paInt16,
        "channels": args.channels,
        "rate": args.sample_rate,
        "input": True,
        "frames_per_buffer": args.block_size,
    }
    if args.input_device_index is not None:
        kwargs["input_device_index"] = args.input_device_index

    try:
        stream = mic.open(**kwargs)
    except Exception:
        mic.terminate()
        raise
    return mic, stream


def drain_and_publish(
    result_queue: "queue.Queue[str]",
    *,
    topic: str,
    setup_file: Path | None,
    timeout: float,
    no_publish: bool,
    print_command: bool,
) -> None:
    while True:
        try:
            text = result_queue.get_nowait()
        except queue.Empty:
            break

        if no_publish:
            print(f"[VLA] no-publish: {text}")
            continue

        if print_command:
            print("[ROS] command:")
            print(build_ros_pub_command(text, topic, setup_file))

        publish_instruction(text, topic=topic, setup_file=setup_file, timeout=timeout)
        print(f"[VLA] published to {topic}: {text}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream microphone ASR results to OpenVLA via /vla_instruction."
    )
    parser.add_argument(
        "--api-key",
        default=os.environ.get("DASHSCOPE_API_KEY"),
        help="DashScope API key. Prefer DASHSCOPE_API_KEY.",
    )
    parser.add_argument(
        "--region",
        choices=("beijing", "international"),
        default="beijing",
        help="DashScope endpoint region matching your API key.",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help="DashScope ASR model name.")
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE, help="Audio sample rate in Hz.")
    parser.add_argument("--channels", type=int, default=DEFAULT_CHANNELS, help="Input channel count.")
    parser.add_argument("--block-size", type=int, default=DEFAULT_BLOCK_SIZE, help="Frames per audio chunk.")
    parser.add_argument("--input-device-index", type=int, default=None, help="PyAudio input device index.")
    parser.add_argument("--list-devices", action="store_true", help="List audio input devices and exit.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="ROS2 String topic consumed by vla_gazebo_bridge.")
    parser.add_argument(
        "--ros-setup",
        default=None,
        help=f"Path to base_robot_ws/install/setup.bash. Defaults to {DEFAULT_ROS_SETUP}.",
    )
    parser.add_argument("--no-publish", action="store_true", help="Only print final ASR text; do not publish to ROS.")
    parser.add_argument("--print-command", action="store_true", help="Print each ros2 topic pub command before executing it.")
    parser.add_argument("--publish-timeout", type=float, default=20.0, help="ROS publish command timeout in seconds.")
    parser.add_argument("--verbose-partial", action="store_true", help="Print non-final partial ASR text.")
    parser.add_argument(
        "--semantic-punctuation",
        action="store_true",
        help="Enable semantic punctuation in streaming ASR results.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_devices:
        list_input_devices()
        return

    if not args.api_key:
        raise SystemExit(
            "Missing DashScope API key. Set it first, for example:\n"
            "  export DASHSCOPE_API_KEY='sk-...'"
        )

    try:
        import dashscope
        from dashscope.audio.asr import Recognition
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing dependency: dashscope. Run this script in the openvla conda "
            "environment and install it with `pip install dashscope` if needed."
        ) from exc

    dashscope.api_key = args.api_key
    dashscope.base_websocket_api_url = (
        BEIJING_WS_URL if args.region == "beijing" else INTERNATIONAL_WS_URL
    )

    setup_file = None if args.no_publish else resolve_ros_setup(args.ros_setup)
    if setup_file is not None:
        print(f"[ROS] setup: {setup_file}")
    else:
        print("[ROS] no-publish mode enabled")

    stop_event = signal_stop_event()
    result_queue: "queue.Queue[str]" = queue.Queue()
    callback = VLARecognitionCallback(result_queue, stop_event, args.verbose_partial).callback

    recognition = Recognition(
        model=args.model,
        format=FORMAT_PCM,
        sample_rate=args.sample_rate,
        semantic_punctuation_enabled=args.semantic_punctuation,
        callback=callback,
    )

    mic = None
    stream = None

    def stop(_sig=None, _frame=None) -> None:
        print("\n[Main] stopping microphone recognition...")
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        mic, stream = open_input_stream(args)
        print(
            "[Mic] opened "
            f"sample_rate={args.sample_rate}, channels={args.channels}, "
            f"block_size={args.block_size}, device={args.input_device_index}"
        )

        recognition.start()
        print("[Main] speak into the microphone. Press Ctrl+C to stop.")

        while not stop_event.is_set():
            data = stream.read(args.block_size, exception_on_overflow=False)
            recognition.send_audio_frame(data)
            drain_and_publish(
                result_queue,
                topic=args.topic,
                setup_file=setup_file,
                timeout=args.publish_timeout,
                no_publish=args.no_publish,
                print_command=args.print_command,
            )
            time.sleep(0.001)

    finally:
        try:
            recognition.stop()
        except Exception:
            pass

        drain_and_publish(
            result_queue,
            topic=args.topic,
            setup_file=setup_file,
            timeout=args.publish_timeout,
            no_publish=args.no_publish,
            print_command=args.print_command,
        )

        if stream is not None:
            stream.stop_stream()
            stream.close()
        if mic is not None:
            mic.terminate()

        print("[Main] stopped")


def signal_stop_event():
    import threading

    return threading.Event()


if __name__ == "__main__":
    main()
