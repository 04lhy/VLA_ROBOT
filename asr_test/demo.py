#!/usr/bin/env python3
"""Transcribe local test audio files with DashScope Fun-ASR.

Usage:
    export DASHSCOPE_API_KEY="sk-..."
    python3 demo.py
    python3 demo.py Chinese.mp3 english.mp3
"""

from __future__ import annotations

import argparse
import os
from http import HTTPStatus
from pathlib import Path
from typing import Any
import dashscope
from dashscope.audio.asr import Recognition

DEFAULT_AUDIO_FILES = ("Chinese.mp3", "english.mp3")
DEFAULT_MODEL = "fun-asr-realtime"
DEFAULT_SAMPLE_RATE = 16000

# Beijing region. Use the international endpoint only if your API key belongs to
# the Singapore/international region.
BEIJING_WS_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"
INTERNATIONAL_WS_URL = "wss://dashscope-intl.aliyuncs.com/api-ws/v1/inference"


def _extract_text(sentence: Any) -> str:
    """Handle both dict and list sentence shapes returned by the SDK."""
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


def transcribe_file(
    audio_path: Path,
    *,
    model: str = DEFAULT_MODEL,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
) -> str:
    if not audio_path.exists():
        raise FileNotFoundError(f"Audio file not found: {audio_path}")

    audio_format = audio_path.suffix.lower().lstrip(".")
    if not audio_format:
        raise ValueError(f"Cannot infer audio format from: {audio_path}")


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

    return _extract_text(result.get_sentence())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transcribe local mp3 files with DashScope Fun-ASR.")
    parser.add_argument(
        "files",
        nargs="*",
        default=list(DEFAULT_AUDIO_FILES),
        help="Audio files to transcribe. Relative paths are resolved from this script directory.",
    )
    parser.add_argument(
        "--api-key",
        default=os.environ.get("DASHSCOPE_API_KEY"),
        help="DashScope API key. Prefer setting DASHSCOPE_API_KEY instead of passing it on the command line.",
    )
    parser.add_argument(
        "--region",
        choices=("beijing", "international"),
        default="beijing",
        help="DashScope endpoint region for the API key.",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help="ASR model name.")
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE, help="Audio sample rate in Hz.")
    parser.add_argument(
        "--save-text",
        action="store_true",
        help="Also write each transcription to a .txt file next to the audio file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.api_key:
        raise SystemExit(
            "Missing DashScope API key. Set it first, for example:\n"
            "  export DASHSCOPE_API_KEY='sk-...'"
        )


    dashscope.api_key = args.api_key
    dashscope.base_websocket_api_url = (
        BEIJING_WS_URL if args.region == "beijing" else INTERNATIONAL_WS_URL
    )

    script_dir = Path(__file__).resolve().parent
    for item in args.files:
        audio_path = Path(item)
        if not audio_path.is_absolute():
            audio_path = script_dir / audio_path

        print(f"\n[ASR] {audio_path.name}")
        text = transcribe_file(
            audio_path,
            model=args.model,
            sample_rate=args.sample_rate,
        )
        print(f"转写文本: {text}")

        if args.save_text:
            output_path = audio_path.with_suffix(".txt")
            output_path.write_text(text + "\n", encoding="utf-8")
            print(f"已保存: {output_path}")


if __name__ == "__main__":
    main()
