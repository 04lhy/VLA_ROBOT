#!/usr/bin/env python3
"""One manually submitted native Omni Realtime audio command."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from contextlib import closing
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
from typing import Any, Callable
import wave

from audio_input import _enter_pressed, _pyaudio
from omni_planner import (
    AUDIO_PLAN_SCHEMA, OMNI_PROMPT, TRANSCRIPT_SCHEMA, OmniClient, OmniError, validate_transcript,
)

DEFAULT_REALTIME_MODEL = "qwen3.8-omni-flash-realtime"
SAMPLE_RATE = 16000
BLOCK_FRAMES = 1600


def _expect_event(session, kind: str) -> dict[str, Any]:
    event = session.recv_event()
    if not event or event.get("type") != kind:
        raise OmniError(f"Omni Realtime expected {kind}; session did not initialize correctly.")
    return event


def _receive_result(session, submitted: threading.Event, function_name: str) -> Any:
    response_id = None
    progress_shown = False
    for event in session.events():
        kind = event.get("type")
        if kind == "response.created":
            if not submitted.is_set() or response_id is not None:
                raise OmniError("Unexpected Omni response before submission or duplicate response.")
            response_id = (event.get("response") or {}).get("id")
            if not isinstance(response_id, str) or not response_id:
                raise OmniError("Omni Realtime response has no ID.")
        elif kind in {"response.function_call_arguments.delta", "response.text.delta"}:
            if not submitted.is_set() or response_id is None:
                raise OmniError("Omni output arrived before command submission.")
            if not progress_shown:
                print("[Omni Realtime] Receiving model output...", file=sys.stderr, flush=True)
                progress_shown = True
        elif kind == "response.done":
            result = event.get("response") or {}
            if not submitted.is_set() or response_id is None or result.get("id") != response_id:
                raise OmniError("Omni Realtime returned an unrelated response.")
            if result.get("status") != "completed":
                raise OmniError(f"Omni Realtime response was {result.get('status')}; no task submitted.")
            outputs = result.get("output") or []
            calls = [item for item in outputs if item.get("type") == "function_call"]
            if len(calls) != 1 or calls[0].get("name") != function_name:
                raise OmniError("Omni must return exactly one expected plan/transcript function call.")
            if calls[0].get("status") != "completed":
                raise OmniError("Omni function arguments are incomplete; no task submitted.")
            if any(item.get("type") not in {"function_call", "message"} for item in outputs):
                raise OmniError("Unexpected Omni output item; no task submitted.")
            try:
                return json.loads(calls[0]["arguments"])
            except (KeyError, TypeError, json.JSONDecodeError) as exc:
                raise OmniError("Omni Realtime returned invalid function JSON.") from exc
        elif kind == "error":
            raise OmniError("Omni Realtime returned a server error.")
        elif kind == "session.finished":
            break
    raise OmniError("Omni Realtime disconnected before a complete response; no task submitted.")


class OmniRealtimeClient:
    def __init__(
        self, *, model: str = DEFAULT_REALTIME_MODEL, ws_url: str | None = None,
        timeout: float = 60.0, http_model: str = "qwen3.8-omni-flash", http_base_url: str | None = None,
    ):
        self._api_key = os.environ.get("DASHSCOPE_API_KEY")
        self.workspace = os.environ.get("DASHSCOPE_WORKSPACE")
        self.region = os.environ.get("DASHSCOPE_REGION", "cn-beijing")
        if not self._api_key or not self.workspace:
            raise OmniError("Omni Realtime requires DASHSCOPE_API_KEY and DASHSCOPE_WORKSPACE.")
        if not math.isfinite(timeout) or timeout <= 0:
            raise OmniError("Omni Realtime timeout must be a finite positive number.")
        self.model, self.ws_url, self.timeout = model, ws_url or os.environ.get("DASHSCOPE_REALTIME_URL"), timeout
        self.http_model, self.http_base_url = http_model, http_base_url
        self.last_seconds = 0.0  # From commit until the complete model response.

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass  # Each command owns and closes its socket and SDK client.

    def _new_client(self):
        try:
            from dashscope import Client
        except ImportError as exc:
            raise OmniError("Omni Realtime requires DashScope SDK 2.x; install requirements.txt.") from exc
        return Client(api_key=self._api_key, workspace=self.workspace, region=self.region, timeout=self.timeout)

    def _request(self, send_audio: Callable, *, context: str | None) -> Any:
        function_name = "submit_robot_plan" if context is not None else "submit_transcript"
        instructions = OMNI_PROMPT + f"\nContext:\n{context}" if context is not None else (
            "Transcribe the spoken command verbatim in its original language. "
            "If speech is unintelligible or silent, return an empty transcript."
        )
        instructions += (
            f"\nAlways call {function_name} exactly once with the complete result as arguments, "
            "including when asking for clarification. Do not answer with ordinary text JSON. "
            "Do not execute robot tools yourself."
        )
        schema = AUDIO_PLAN_SCHEMA if context is not None else TRANSCRIPT_SCHEMA
        submitted = threading.Event()
        try:
            with self._new_client() as client, client.audio.realtime.connect(
                model=self.model, ws_url=self.ws_url, timeout=self.timeout,
            ) as session:
                _expect_event(session, "session.created")
                session.update_session(
                    output_modalities=["text"], enable_turn_detection=False,
                    enable_input_audio_transcription=False,
                    input_audio_config={"type": "pcm", "sample_rate": SAMPLE_RATE},
                    instructions=instructions,
                    tools=[{"type": "function", "function": {
                        "name": function_name, "description": "Return the complete command result to the local application.",
                        "parameters": schema,
                    }}],
                )
                ack = _expect_event(session, "session.updated").get("session") or {}
                if ack.get("turn_detection") is not None:
                    raise OmniError("Omni did not disable automatic turn detection; command canceled.")
                with ThreadPoolExecutor(max_workers=1, thread_name_prefix="agent-omni") as pool:
                    receiver = pool.submit(_receive_result, session, submitted, function_name)
                    try:
                        send_audio(session, receiver)
                        if receiver.done():
                            receiver.result()
                            raise OmniError("Omni session finished before submission.")
                        print("[Omni Realtime] Input ended; generating result...", file=sys.stderr, flush=True)
                        started = time.monotonic()
                        submitted.set()
                        session.commit()
                        session.create_response()
                        try:
                            result = receiver.result(timeout=self.timeout)
                        except FutureTimeoutError as exc:
                            raise OmniError("Timed out waiting for Omni Realtime; no task submitted.") from exc
                        self.last_seconds = time.monotonic() - started
                        return result
                    finally:
                        session.close()  # Unblock receive before joining the thread on cancel/error.
        except OmniError:
            raise
        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}".replace(self._api_key, "[redacted]")
            message = re.sub(r'"audio"\s*:\s*"[^"]*"', '"audio":"[redacted]"', message)
            raise OmniError(f"Omni Realtime failed: {message[:700]}") from exc

    def microphone_payload(self, *, context: str | None = None, device_index: int | None = None) -> Any:
        if not sys.stdin.isatty():
            raise OmniError("Omni Realtime microphone input needs an interactive terminal; Enter submits.")

        def capture(session, receiver):
            pyaudio = _pyaudio()
            audio = pyaudio.PyAudio()
            try:
                with closing(audio.open(
                    format=pyaudio.paInt16, channels=1, rate=SAMPLE_RATE, input=True,
                    input_device_index=device_index, frames_per_buffer=BLOCK_FRAMES,
                )) as stream:
                    print("[Mic] Omni Realtime streaming now. Speak, then press Enter; Ctrl+C cancels.", file=sys.stderr, flush=True)
                    sent = 0
                    while True:
                        if receiver.done():
                            receiver.result()
                            raise OmniError("Omni stream ended before submission.")
                        if _enter_pressed():
                            break
                        session.append_audio(stream.read(BLOCK_FRAMES, exception_on_overflow=True))
                        sent += BLOCK_FRAMES
                    if sent < SAMPLE_RATE // 10:
                        raise OmniError("No microphone audio captured; command canceled.")
            finally:
                audio.terminate()
        return self._request(capture, context=context)

    def audio_payload(self, audio_file: str | Path, *, context: str | None = None) -> Any:
        """Replay a 16 kHz mono PCM WAV through the same streaming protocol."""
        path = Path(audio_file).expanduser().resolve()
        try:
            with wave.open(str(path), "rb") as audio:
                if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate()) != (1, 2, SAMPLE_RATE):
                    raise OmniError("Realtime file replay requires 16 kHz mono 16-bit PCM WAV.")
                if audio.getnframes() < SAMPLE_RATE // 10:
                    raise OmniError("Realtime WAV must contain at least 100 ms of audio.")

                def replay(session, receiver):
                    deadline = time.monotonic()
                    while True:
                        if receiver.done():
                            receiver.result()
                            raise OmniError("Omni stream ended before file submission.")
                        data = audio.readframes(BLOCK_FRAMES)
                        if not data:
                            break
                        session.append_audio(data)
                        deadline += len(data) / (2 * SAMPLE_RATE)
                        time.sleep(max(0, deadline - time.monotonic()))
                return self._request(replay, context=context)
        except (OSError, wave.Error, EOFError) as exc:
            raise OmniError(f"Cannot read Realtime WAV (convert MP3 to PCM WAV first): {exc}") from exc

    def transcribe_microphone(self, *, device_index: int | None = None) -> str:
        return validate_transcript(self.microphone_payload(device_index=device_index))

    def transcribe_file(self, audio_file: str | Path) -> str:
        return validate_transcript(self.audio_payload(audio_file))

    def chat(self, messages: list[dict[str, Any]], *, temperature: float = 0.1) -> str:
        # Recovery planning uses the already validated transcript/history over HTTP Omni.
        with OmniClient(api_key=self._api_key, model=self.http_model, base_url=self.http_base_url, timeout=self.timeout) as client:
            return client.chat(messages, temperature=temperature)
