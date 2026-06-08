#!/usr/bin/env python3
"""Transcribe a local audio file and send the text to OpenVLA in Gazebo.

This script is intentionally independent from rclpy so it can run inside the
`openvla` conda environment where DashScope is installed. It publishes the final
text by invoking `ros2 topic pub` in a sourced ROS2 workspace shell.

Example:
    conda activate openvla
    export DASHSCOPE_API_KEY="sk-..."
    python3 asr_file_to_vla.py Chinese.mp3 \
        --ros-setup /root/autodl-tmp/VLA_ROBOT/base_robot_ws/install/setup.bash
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
from http import HTTPStatus
from pathlib import Path
from typing import Any

DEFAULT_MODEL = "fun-asr-realtime"
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_TOPIC = "/vla_instruction"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROS_SETUP = PROJECT_ROOT / "base_robot_ws" / "install" / "setup.bash"

BEIJING_WS_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"
INTERNATIONAL_WS_URL = "wss://dashscope-intl.aliyuncs.com/api-ws/v1/inference"


def _extract_text(sentence: Any) -> str:
    if isinstance(sentence, dict):
        return str(sentence.get("text", "")).strip()

    if isinstance(sentence, list):
        texts = []
        for item in sentence:
            if isinstance(item, dict) and item.get("text"):
                texts.append(str(item["text"]).strip())
            elif isinstance(item, str):
                texts.append(item.strip())
        return "".join(texts).strip()

    return str(sentence).strip()


def transcribe_file(audio_path: Path, *, model: str, sample_rate: int) -> str:
    if not audio_path.exists():
        raise FileNotFoundError(f"Audio file not found: {audio_path}")

    audio_format = audio_path.suffix.lower().lstrip(".")
    if not audio_format:
        raise ValueError(f"Cannot infer audio format from: {audio_path}")

    try:
        from dashscope.audio.asr import Recognition
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing dependency: dashscope. Run this script in the openvla conda "
            "environment and install it with `pip install dashscope` if needed."
        ) from exc

    recognition = Recognition(
        model=model,
        format=audio_format,
        sample_rate=sample_rate,
        callback=None,
    )

    result = recognition.call(str(audio_path))
    if result.status_code != HTTPStatus.OK:
        message = getattr(result, "message", None) or recognition.get_response()
        raise RuntimeError(
            f"ASR failed for {audio_path.name}: status={result.status_code}, message={message}"
        )

    text = _extract_text(result.get_sentence())
    if not text:
        raise RuntimeError(f"ASR returned empty text for {audio_path.name}")
    return text


def resolve_ros_setup(explicit_setup: str | None) -> Path:
    """Resolve the ROS2 setup file used before running `ros2 topic pub`.

    By default this intentionally uses the setup file inside the same VLA_ROBOT
    project tree as this script. That avoids accidentally publishing with a
    setup.bash from another copy of the project.
    """
    setup = Path(explicit_setup).expanduser() if explicit_setup else DEFAULT_ROS_SETUP
    if not setup.exists():
        raise FileNotFoundError(
            "ROS setup file not found: "
            f"{setup}\n"
            "Build and source the ROS workspace first, for example:\n"
            f"  cd {PROJECT_ROOT / 'base_robot_ws'}\n"
            "  colcon build --symlink-install\n"
            "Or pass the correct file with --ros-setup /path/to/install/setup.bash"
        )
    return setup

def build_ros_pub_command(text: str, topic: str, setup_file: Path | None) -> str:
    yaml_text = text.replace("'", "''")
    payload = "{data: '" + yaml_text + "'}"
    ros_cmd = " ".join(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            shlex.quote(topic),
            "std_msgs/msg/String",
            shlex.quote(payload),
        ]
    )

    if setup_file is None:
        return ros_cmd
    return f"source {shlex.quote(str(setup_file))} && {ros_cmd}"


def publish_instruction(text: str, *, topic: str, setup_file: Path | None, timeout: float) -> None:
    command = build_ros_pub_command(text, topic, setup_file)
    completed = subprocess.run(
        ["bash", "-lc", command],
        check=False,
        text=True,
        capture_output=True,
        timeout=timeout,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "Failed to publish ROS instruction.\n"
            f"Command: {command}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )

    if completed.stdout.strip():
        print(completed.stdout.strip())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transcribe a local audio file and publish it to /vla_instruction."
    )
    parser.add_argument(
        "audio_file",
        help="Audio file path. Relative paths are resolved from this script directory first.",
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
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="ROS2 String topic consumed by vla_gazebo_bridge.")
    parser.add_argument(
        "--ros-setup",
        default=None,
        help=f"Path to base_robot_ws/install/setup.bash. Defaults to {DEFAULT_ROS_SETUP}.",
    )
    parser.add_argument(
        "--no-publish",
        action="store_true",
        help="Only print the transcription; do not publish to ROS.",
    )
    parser.add_argument(
        "--print-command",
        action="store_true",
        help="Print the ros2 topic pub command before executing it.",
    )
    parser.add_argument("--timeout", type=float, default=20.0, help="ROS publish command timeout in seconds.")
    return parser.parse_args()


def resolve_audio_path(audio_file: str) -> Path:
    audio_path = Path(audio_file).expanduser()
    if audio_path.is_absolute() or audio_path.exists():
        return audio_path

    script_relative = Path(__file__).resolve().parent / audio_path
    if script_relative.exists():
        return script_relative
    return audio_path


def main() -> None:
    args = parse_args()
    if not args.api_key:
        raise SystemExit(
            "Missing DashScope API key. Set it first, for example:\n"
            "  export DASHSCOPE_API_KEY='sk-...'"
        )

    try:
        import dashscope
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing dependency: dashscope. Run this script in the openvla conda "
            "environment and install it with `pip install dashscope` if needed."
        ) from exc

    dashscope.api_key = args.api_key
    dashscope.base_websocket_api_url = (
        BEIJING_WS_URL if args.region == "beijing" else INTERNATIONAL_WS_URL
    )

    audio_path = resolve_audio_path(args.audio_file)
    setup_file = resolve_ros_setup(args.ros_setup) if not args.no_publish else None

    print(f"[ASR] audio: {audio_path}")
    text = transcribe_file(audio_path, model=args.model, sample_rate=args.sample_rate)
    print(f"[ASR] text: {text}")

    if args.no_publish:
        return

    print(f"[ROS] setup: {setup_file}")

    if args.print_command:
        print("[ROS] command:")
        print(build_ros_pub_command(text, args.topic, setup_file))

    publish_instruction(text, topic=args.topic, setup_file=setup_file, timeout=args.timeout)
    print(f"[ROS] published to {args.topic}")


if __name__ == "__main__":
    main()
