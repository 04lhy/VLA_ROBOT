#!/usr/bin/env python3
"""Independent, one-command audio input using DashScope SDK 2.x Fun-ASR."""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import tempfile
import wave
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from contextlib import closing
from pathlib import Path
from typing import Any, Callable, Iterable

DEFAULT_MODEL = "fun-asr-realtime"
DEFAULT_SAMPLE_RATE = 16000
AUDIO_FORMATS = {"wav", "mp3", "pcm", "opus", "speex", "aac", "amr"}


class AudioInputError(RuntimeError):
    """An input/configuration/recognition failure; no instruction is available."""


def _collect_transcript(
    events: Iterable[dict[str, Any]], *, on_update: Callable[[str, bool], None] | None = None,
) -> str:
    """Collect final sentences from one completed task, never partial text."""
    sentences: dict[tuple, str] = {}
    completed = False
    for event in events:
        header = event.get("header") or {}
        kind = header.get("event")
        if kind == "task-failed":
            raise AudioInputError(
                f"ASR task failed: {header.get('error_code', 'unknown')}: "
                f"{header.get('error_message', 'no error message')}"
            )
        if kind == "task-finished":
            completed = True
            break
        if kind != "result-generated":
            continue
        sentence = ((event.get("payload") or {}).get("output") or {}).get("sentence") or {}
        if sentence.get("heartbeat"):
            continue
        final = sentence.get("sentence_end", sentence.get("end_time") is not None)
        text = sentence.get("text")
        if not isinstance(text, str) or not text.strip():
            continue
        if on_update is not None:
            on_update(text.strip(), bool(final))
        if not final:
            continue
        # Deduplicate final updates by sentence identity, preserving repeated speech.
        if sentence.get("sentence_id") is not None:
            key = ("id", sentence["sentence_id"])
        elif sentence.get("begin_time") is not None:
            key = ("begin", sentence["begin_time"])
        else:
            key = ("event", len(sentences))
        sentences[key] = text.strip()
    if not completed:
        raise AudioInputError("ASR ended before task-finished; refusing incomplete input.")
    text = " ".join(sentences.values()).strip()
    if not text:
        raise AudioInputError("No speech recognized. Please provide an audible command.")
    return text


def _print_update(text: str, final: bool) -> None:
    print(f"[ASR] {'final' if final else 'partial'}: {text}", file=sys.stderr, flush=True)


def _enter_pressed() -> bool:
    """Poll canonical terminal input without a blocking keyboard thread."""
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return False
    if not sys.stdin.readline():
        raise AudioInputError("Terminal input closed; command canceled.")
    return True


def _pyaudio():
    try:
        import pyaudio
    except ImportError as exc:
        raise AudioInputError(
            "Microphone input needs PyAudio: install PortAudio development headers, "
            "then run `python3 -m pip install -r agent_demo/requirements-mic.txt`."
        ) from exc
    return pyaudio


def list_input_devices() -> list[tuple[int, str]]:
    audio = _pyaudio().PyAudio()
    try:
        return [
            (index, str(info.get("name", "unknown")))
            for index in range(audio.get_device_count())
            if (info := audio.get_device_info_by_index(index)).get("maxInputChannels", 0) > 0
        ]
    except OSError as exc:
        raise AudioInputError(f"Cannot enumerate microphone devices: {exc}") from exc
    finally:
        audio.terminate()


class SpeechInput:
    """Transcribe a local file or one microphone recording, without agent/ROS imports."""

    def __init__(
        self, *, api_key: str | None = None, workspace: str | None = None,
        region: str | None = None, model: str = DEFAULT_MODEL,
        sample_rate: int | None = None, timeout: float = 30.0,
    ):
        self._api_key = api_key or os.environ.get("DASHSCOPE_API_KEY")
        self.workspace = workspace or os.environ.get("DASHSCOPE_WORKSPACE")
        self.region = region or os.environ.get("DASHSCOPE_REGION", "cn-beijing")
        if not self._api_key:
            raise AudioInputError("Set DASHSCOPE_API_KEY to your Alibaba Cloud Model Studio API key.")
        if not self.workspace:
            raise AudioInputError("Set DASHSCOPE_WORKSPACE to your Workspace ID, or use --asr-workspace.")
        if sample_rate is not None and sample_rate <= 0:
            raise AudioInputError("ASR sample rate must be positive.")
        if not math.isfinite(timeout) or timeout <= 0:
            raise AudioInputError("ASR timeout must be a finite positive number.")
        self.model = model
        self.sample_rate = sample_rate
        self.timeout = timeout

    def _new_client(self):
        try:
            from dashscope import Client
        except ImportError as exc:
            raise AudioInputError(
                "Audio input requires DashScope SDK 2.x. Run "
                "`python3 -m pip install -r agent_demo/requirements.txt`."
            ) from exc
        return Client(
            api_key=self._api_key, workspace=self.workspace, region=self.region,
            timeout=self.timeout, max_retries=0,
        )

    def transcribe_file(self, audio_file: str | Path) -> str:
        """Use WAV metadata; compressed/raw audio defaults to 16 kHz unless specified."""
        path = Path(audio_file).expanduser().resolve()
        try:
            if not path.is_file():
                raise AudioInputError(f"Audio file not found: {path}")
            if path.stat().st_size == 0:
                raise AudioInputError(f"Audio file is empty: {path}")
            audio_format = path.suffix.lower().lstrip(".")
            if audio_format not in AUDIO_FORMATS:
                raise AudioInputError(f"Unsupported audio format .{audio_format}; use {', '.join(sorted(AUDIO_FORMATS))}.")
            sample_rate = self.sample_rate or DEFAULT_SAMPLE_RATE
            if audio_format == "wav":
                with wave.open(str(path), "rb") as audio:
                    if audio.getnchannels() != 1 or audio.getsampwidth() != 2:
                        raise AudioInputError("WAV input must be mono, 16-bit PCM. Convert the file first.")
                    if audio.getnframes() == 0:
                        raise AudioInputError(f"WAV file contains no audio frames: {path}")
                    sample_rate = audio.getframerate()
                    if self.sample_rate is not None and self.sample_rate != sample_rate:
                        raise AudioInputError("--asr-sample-rate does not match the WAV header.")
            with self._new_client() as client:
                with closing(client.audio.asr.recognize_file(
                    str(path), model=self.model, format=audio_format,
                    sample_rate=sample_rate, timeout=self.timeout,
                )) as events:
                    return _collect_transcript(events)
        except AudioInputError:
            raise
        except Exception as exc:
            # SDK/network exceptions may contain request details; do not print the key.
            message = str(exc).replace(self._api_key, "[redacted]")
            raise AudioInputError(f"Audio transcription failed: {message}") from exc

    def transcribe_microphone(self, *, seconds: float = 5.0, device_index: int | None = None) -> str:
        """Record one finite utterance to a temporary WAV, then transcribe it once."""
        if not math.isfinite(seconds) or seconds <= 0:
            raise AudioInputError("Recording duration must be a finite positive number.")
        pyaudio = _pyaudio()
        rate = self.sample_rate or DEFAULT_SAMPLE_RATE
        with tempfile.TemporaryDirectory(prefix="agent_audio_") as directory:
            path = Path(directory) / "command.wav"
            audio = pyaudio.PyAudio()
            try:
                with closing(audio.open(
                    format=pyaudio.paInt16, channels=1, rate=rate, input=True,
                    input_device_index=device_index, frames_per_buffer=1024,
                )) as stream, wave.open(str(path), "wb") as output:
                    output.setnchannels(1)
                    output.setsampwidth(2)
                    output.setframerate(rate)
                    print(f"[Mic] Recording {seconds:g}s now...", file=sys.stderr, flush=True)
                    remaining = max(1, round(seconds * rate))
                    while remaining:
                        count = min(1024, remaining)
                        output.writeframesraw(stream.read(count, exception_on_overflow=True))
                        remaining -= count
            except (OSError, ValueError) as exc:
                raise AudioInputError(f"Microphone recording failed: {exc}") from exc
            finally:
                audio.terminate()
            print("[Mic] Recording complete; transcribing...", file=sys.stderr, flush=True)
            return self.transcribe_file(path)

    def transcribe_microphone_stream(
        self, *, device_index: int | None = None,
        on_update: Callable[[str, bool], None] | None = None,
    ) -> str:
        """Upload live PCM while receiving results; Enter submits one whole command."""
        if not sys.stdin.isatty():
            raise AudioInputError(
                "Streaming microphone input needs an interactive terminal to submit with Enter. "
                "Use --microphone --record-seconds for timed recording instead."
            )
        pyaudio = _pyaudio()
        rate = self.sample_rate or DEFAULT_SAMPLE_RATE
        block_size = max(1, rate // 10)  # About 100 ms per PCM frame.
        try:
            with self._new_client() as client, client.audio.asr.recognize(
                model=self.model, format="pcm", sample_rate=rate,
                parameters={"heartbeat": True}, timeout=self.timeout,
            ) as recognition:
                audio = pyaudio.PyAudio()
                try:
                    with ThreadPoolExecutor(max_workers=1, thread_name_prefix="agent-asr") as pool:
                        receiver = pool.submit(
                            _collect_transcript, recognition.events(),
                            on_update=_print_update if on_update is None else on_update,
                        )
                        try:
                            with closing(audio.open(
                                format=pyaudio.paInt16, channels=1, rate=rate, input=True,
                                input_device_index=device_index, frames_per_buffer=block_size,
                            )) as stream:
                                print(
                                    "[Mic] Streaming now. Speak, then press Enter to submit; Ctrl+C cancels.",
                                    file=sys.stderr, flush=True,
                                )
                                while True:
                                    if receiver.done():
                                        receiver.result()  # Propagate any ASR error immediately.
                                        raise AudioInputError("ASR stream ended before command submission.")
                                    if _enter_pressed():
                                        break
                                    recognition.send_audio_frame(stream.read(
                                        block_size, exception_on_overflow=True,
                                    ))
                            print("[Mic] Input ended; waiting for final ASR results...", file=sys.stderr, flush=True)
                            recognition.stop()
                            try:
                                return receiver.result(timeout=self.timeout)
                            except FutureTimeoutError as exc:
                                raise AudioInputError(
                                    f"Timed out after {self.timeout:g}s waiting for final ASR results."
                                ) from exc
                        finally:
                            # Closing before the pool joins unblocks a receiving socket on cancel/error.
                            recognition.close()
                finally:
                    audio.terminate()
        except AudioInputError:
            raise
        except Exception as exc:
            message = str(exc).replace(self._api_key, "[redacted]")
            raise AudioInputError(f"Streaming audio input failed: {message}") from exc


def add_audio_arguments(parser: argparse.ArgumentParser) -> None:
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--audio-file", type=Path, help="Transcribe a local audio file (relative to the working directory).")
    source.add_argument("--microphone", action="store_true", help="Record one command, then transcribe it.")
    source.add_argument("--list-input-devices", action="store_true", help="List microphone devices and exit; no API key needed.")
    parser.add_argument("--stream", action="store_true", help="With --microphone: upload live audio and show partial results; Enter submits.")
    parser.add_argument("--record-seconds", type=float, help="Timed microphone recording duration (default: 5 seconds); cannot combine with --stream.")
    parser.add_argument("--input-device-index", type=int, help="PyAudio microphone device index.")
    parser.add_argument("--asr-model", default=os.environ.get("AGENT_ASR_MODEL", DEFAULT_MODEL))
    parser.add_argument("--asr-workspace", help="Model Studio Workspace ID (default: DASHSCOPE_WORKSPACE).")
    parser.add_argument("--asr-region", help="API key region, e.g. cn-beijing / ap-southeast-1 (default: DASHSCOPE_REGION or cn-beijing).")
    parser.add_argument("--asr-sample-rate", type=int, help="WAV: validate header; other formats/microphone: sample rate (default: 16000).")
    parser.add_argument("--asr-timeout", type=float, default=30.0, help="SDK connection/read timeout and streaming final-result wait, in seconds (default: 30).")


def validate_audio_arguments(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.stream and not args.microphone:
        parser.error("--stream requires --microphone")
    if args.stream and args.record_seconds is not None:
        parser.error("--stream ends on Enter; do not combine it with --record-seconds")


def read_audio_instruction(args: argparse.Namespace) -> str:
    speech = SpeechInput(
        workspace=args.asr_workspace, region=args.asr_region, model=args.asr_model,
        sample_rate=args.asr_sample_rate, timeout=args.asr_timeout,
    )
    if args.audio_file is not None:
        return speech.transcribe_file(args.audio_file)
    if args.microphone:
        if args.stream:
            return speech.transcribe_microphone_stream(device_index=args.input_device_index)
        seconds = 5.0 if args.record_seconds is None else args.record_seconds
        return speech.transcribe_microphone(seconds=seconds, device_index=args.input_device_index)
    raise AudioInputError("Choose --audio-file or --microphone.")


def print_input_devices() -> None:
    devices = list_input_devices()
    for index, name in devices:
        print(f"[{index}] {name}")
    if not devices:
        print("No microphone input devices found.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Transcribe one audio input to text; does not invoke the agent.")
    add_audio_arguments(parser)
    args = parser.parse_args()
    validate_audio_arguments(parser, args)
    if not (args.audio_file is not None or args.microphone or args.list_input_devices):
        parser.error("choose --audio-file, --microphone or --list-input-devices")
    try:
        if args.list_input_devices:
            print_input_devices()
        else:
            print(read_audio_instruction(args))
    except AudioInputError as exc:
        raise SystemExit(f"[Audio] {exc}") from exc
    except KeyboardInterrupt:
        raise SystemExit(130)


if __name__ == "__main__":
    main()
