#!/usr/bin/env python3
"""Verify audio-file instructions without Gazebo.

Flow:
    local audio file -> DashScope ASR text -> OpenVLA /act with a dummy image

This proves the transcribed text can enter the OpenVLA server even when Gazebo
cannot provide a real camera frame.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Sequence

from asr_file_to_vla import (
    BEIJING_WS_URL,
    DEFAULT_MODEL,
    DEFAULT_SAMPLE_RATE,
    INTERNATIONAL_WS_URL,
    resolve_audio_path,
    transcribe_file,
)

DEFAULT_SERVER_URL = "http://127.0.0.1:8000/act"


def parse_rgb(value: str) -> tuple[int, int, int]:
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("RGB color must be formatted as R,G,B")

    try:
        rgb = tuple(int(part.strip()) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("RGB color values must be integers") from exc

    if any(channel < 0 or channel > 255 for channel in rgb):
        raise argparse.ArgumentTypeError("RGB color values must be in [0, 255]")
    return rgb  # type: ignore[return-value]


def make_dummy_image(width: int, height: int, rgb: Sequence[int]) -> list[list[list[int]]]:
    pixel = [int(rgb[0]), int(rgb[1]), int(rgb[2])]
    row = [pixel[:] for _ in range(width)]
    return [[px[:] for px in row] for _ in range(height)]


def call_openvla_server(
    *,
    server_url: str,
    instruction: str,
    image: list[list[list[int]]],
    timeout: float,
) -> dict:
    try:
        import requests
    except ModuleNotFoundError as exc:
        raise RuntimeError("Missing dependency: requests. Install it with `pip install requests`.") from exc

    response = requests.post(
        server_url,
        json={"image": image, "instruction": instruction},
        timeout=timeout,
    )
    response.raise_for_status()
    return response.json()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transcribe an audio file and send the text directly to OpenVLA /act without Gazebo."
    )
    parser.add_argument("audio_file", help="Audio file path. Relative paths are resolved from this script directory first.")
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
    parser.add_argument("--server-url", default=DEFAULT_SERVER_URL, help="OpenVLA /act endpoint URL.")
    parser.add_argument("--image-width", type=int, default=256, help="Dummy image width.")
    parser.add_argument("--image-height", type=int, default=256, help="Dummy image height.")
    parser.add_argument("--image-color", type=parse_rgb, default=(128, 128, 128), help="Dummy image RGB color, e.g. 128,128,128.")
    parser.add_argument("--timeout", type=float, default=30.0, help="OpenVLA HTTP request timeout in seconds.")
    parser.add_argument("--asr-only", action="store_true", help="Only print ASR text; do not call OpenVLA.")
    return parser.parse_args()


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
    print(f"[ASR] audio: {audio_path}")
    instruction = transcribe_file(audio_path, model=args.model, sample_rate=args.sample_rate)
    print(f"[ASR] text: {instruction}")

    if args.asr_only:
        return

    image = make_dummy_image(args.image_width, args.image_height, args.image_color)
    print(f"[OpenVLA] POST {args.server_url}")
    print(f"[OpenVLA] dummy image: {args.image_width}x{args.image_height}, color={args.image_color}")

    result = call_openvla_server(
        server_url=args.server_url,
        instruction=instruction,
        image=image,
        timeout=args.timeout,
    )

    print(f"[OpenVLA] status: {result.get('status', '<no status field>')}")
    if "instruction" in result:
        print(f"[OpenVLA] echoed instruction: {result['instruction']}")
    if "prompt" in result:
        print(f"[OpenVLA] echoed prompt: {result['prompt']}")
    if "action" in result:
        print(f"[OpenVLA] action: {result['action']}")
    else:
        print(f"[OpenVLA] raw response: {result}")


if __name__ == "__main__":
    main()
