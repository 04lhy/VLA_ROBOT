#!/usr/bin/env python3
"""Native Qwen-Omni file input, with the existing plan/executor contract."""

from __future__ import annotations

import base64
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import re
import time
from typing import Any
import wave

from agent_types import AgentState, PlanStep
from audio_input import AudioInputError
from planner import PLANNER_SYSTEM_PROMPT, Planner, known_context
from tools import DOCK_POSES

DEFAULT_OMNI_MODEL = "qwen3.8-omni-flash"
MAX_BASE64_BYTES = 10_000_000  # Model Studio requires encoded audio smaller than 10 MB.
OMNI_PROMPT = PLANNER_SYSTEM_PROMPT + """
The user's instruction is in the attached audio. Understand that audio directly.
Extend the JSON object above with a required "transcript" string containing the
spoken instruction in its original language. Do not replace transcription with a summary.
Return exactly: transcript, needs_clarification, clarification_question, steps.
Each step must contain step_id, description, tool, args, verify.
verify must be null or an object with tool and args, using a verify_* tool.
If no intelligible command is audible, set transcript="", needs_clarification=true,
clarification_question="请重新说出指令。", steps=[]. Do not invent a command.
If clarification is needed, steps must be empty. Otherwise provide a nonempty plan.
"""


class OmniError(AudioInputError):
    """Configuration, API, or native audio-plan validation error."""


def _audio_item(audio_file: str | Path) -> dict[str, Any]:
    path = Path(audio_file).expanduser().resolve()
    try:
        if not path.is_file() or path.stat().st_size == 0:
            raise OmniError(f"Audio file is missing or empty: {path}")
        audio_format = path.suffix.lower().lstrip(".")
        if audio_format not in {"wav", "mp3"}:
            raise OmniError("Omni file input currently supports .wav and .mp3; convert other formats first.")
        if 4 * ((path.stat().st_size + 2) // 3) >= MAX_BASE64_BYTES:
            raise OmniError("Omni audio exceeds the local Base64 limit of less than 10 MB.")
        if audio_format == "wav":
            with wave.open(str(path), "rb") as audio:
                if audio.getnframes() == 0:
                    raise OmniError("WAV file contains no audio frames.")
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        if len(encoded) >= MAX_BASE64_BYTES:
            raise OmniError("Omni audio exceeds the local Base64 limit of less than 10 MB.")
    except OmniError:
        raise
    except (OSError, wave.Error, EOFError) as exc:
        raise OmniError(f"Cannot read Omni audio input: {exc}") from exc
    mime = "audio/wav" if audio_format == "wav" else "audio/mpeg"
    return {"type": "input_audio", "input_audio": {
        "data": f"data:{mime};base64,{encoded}", "format": audio_format,
    }}


def _validate_schema(value: Any, schema: dict[str, Any]) -> None:
    try:
        from jsonschema import Draft7Validator
    except ImportError as exc:
        raise OmniError("Omni validation requires jsonschema; install agent_demo/requirements.txt.") from exc
    error = next(Draft7Validator(schema).iter_errors(value), None)
    if error is not None:
        path = ".".join(str(item) for item in error.absolute_path) or "response"
        raise OmniError(f"Invalid Omni plan at {path}: {error.message[:350]}")


TEXT = {"type": "string", "minLength": 1, "pattern": r"\S"}
TOOL_ARGUMENTS = {
    "navigate_to": ({"location": TEXT}, ["location"]),
    "detect_object": ({"object_name": TEXT}, ["object_name"]),
    "detect_color": ({"location": {"type": "string"}}, []),
    "vla_manipulate": ({
        "instruction": TEXT,
        "dock": {"enum": [None, *sorted(DOCK_POSES)]},
        "wait_timeout": {"type": ["number", "null"], "exclusiveMinimum": 0},
    }, ["instruction"]),
    "verify_navigation": ({"location": TEXT}, ["location"]),
    "verify_object_visible": ({"object_name": TEXT}, ["object_name"]),
    "verify_grasp": ({"object_name": {"type": ["string", "null"]}}, []),
    "verify_condition": ({"condition": TEXT}, ["condition"]),
}
VERIFY_SCHEMA = {
    "type": "object", "required": ["tool", "args"], "additionalProperties": False,
    "properties": {
        "tool": {"enum": [name for name in TOOL_ARGUMENTS if name.startswith("verify_")]},
        "args": {"type": "object"},
    },
}
STEPS_SCHEMA = {"type": "array", "items": {
    "type": "object", "required": ["step_id", "description", "tool", "args", "verify"],
    "additionalProperties": False,
    "properties": {
        "step_id": TEXT, "description": {"type": "string"},
        "tool": {"enum": list(TOOL_ARGUMENTS)}, "args": {"type": "object"},
        "verify": {"anyOf": [{"type": "null"}, VERIFY_SCHEMA]},
    },
}}
AUDIO_PLAN_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["transcript", "needs_clarification", "clarification_question", "steps"],
    "properties": {
        "transcript": {"type": "string"}, "needs_clarification": {"type": "boolean"},
        "clarification_question": {"type": "string"}, "steps": STEPS_SCHEMA,
    },
}
TRANSCRIPT_SCHEMA = {
    "type": "object", "required": ["transcript"], "additionalProperties": False,
    "properties": {"transcript": {"type": "string"}},
}


def validate_transcript(payload: Any) -> str:
    _validate_schema(payload, TRANSCRIPT_SCHEMA)
    text = payload["transcript"].strip()
    if not text:
        raise OmniError("No intelligible speech recognized by Omni.")
    return text


def validate_steps(steps: Any, known_locations: list[str]) -> None:
    _validate_schema(steps, STEPS_SCHEMA)
    ids = set()
    for step in steps:
        if step["step_id"] in ids:
            raise OmniError(f"Duplicate Omni step_id: {step['step_id']}")
        ids.add(step["step_id"])
        for call in (step, step["verify"]):
            if call is None:
                continue
            properties, required = TOOL_ARGUMENTS[call["tool"]]
            _validate_schema(call["args"], {
                "type": "object", "properties": properties, "required": required,
                "additionalProperties": False,
            })
            location = call["args"].get("location")
            if location and location not in known_locations:
                raise OmniError(f"Unknown Omni plan location: {location}")


def validate_audio_plan(payload: Any, known_locations: list[str]) -> str:
    _validate_schema(payload, AUDIO_PLAN_SCHEMA)
    if not payload["transcript"].strip():
        raise OmniError("No intelligible speech recognized by Omni; no task submitted.")
    if payload["needs_clarification"]:
        if payload["steps"]:
            raise OmniError("Omni requested clarification but also returned executable steps.")
        raise OmniError(f"Omni needs clarification: {payload['clarification_question'] or '请补充指令。'}")
    if not payload["steps"]:
        raise OmniError("Omni returned an empty plan; no task submitted.")
    validate_steps(payload["steps"], known_locations)
    return payload["transcript"].strip()


class OmniClient:
    def __init__(
        self, *, api_key: str | None = None, base_url: str | None = None,
        model: str = DEFAULT_OMNI_MODEL, timeout: float = 60.0,
    ):
        self._api_key = api_key or os.environ.get("DASHSCOPE_API_KEY")
        if not self._api_key:
            raise OmniError("Set DASHSCOPE_API_KEY for native Omni input.")
        base_url = base_url or os.environ.get("DASHSCOPE_BASE_URL")
        if not base_url:
            workspace = os.environ.get("DASHSCOPE_WORKSPACE", "")
            region = os.environ.get("DASHSCOPE_REGION", "cn-beijing")
            if not re.fullmatch(r"[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?", workspace):
                raise OmniError("Set DASHSCOPE_WORKSPACE, or provide --omni-base-url / DASHSCOPE_BASE_URL.")
            if not re.fullmatch(r"[a-z0-9-]+", region):
                raise OmniError("Invalid DASHSCOPE_REGION.")
            base_url = f"https://{workspace}.{region}.maas.aliyuncs.com/compatible-mode/v1"
        if not math.isfinite(timeout) or timeout <= 0:
            raise OmniError("Omni timeout must be a finite positive number.")
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise OmniError("Install agent_demo/requirements.txt to use Omni.") from exc
        self.model = model
        self.last_seconds = 0.0
        self.client = OpenAI(api_key=self._api_key, base_url=base_url, timeout=timeout, max_retries=0)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.client.close()

    def chat(self, messages: list[dict[str, Any]], *, temperature: float = 0.1) -> str:
        started = time.monotonic()
        try:
            response = self.client.chat.completions.create(
                model=self.model, messages=messages, temperature=temperature,
                modalities=["text"], response_format={"type": "json_object"},
                reasoning_effort="none", stream=False, max_tokens=4096,
            )
            self.last_seconds = time.monotonic() - started
            if not response.choices or response.choices[0].finish_reason != "stop":
                raise OmniError("Omni response did not complete normally; no task submitted.")
            text = response.choices[0].message.content
            if not isinstance(text, str) or not text.strip():
                raise OmniError("Omni returned no text; no task submitted.")
            return text
        except OmniError:
            raise
        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}"
            if exc.__cause__ is not None:
                message += f"; cause: {type(exc.__cause__).__name__}: {exc.__cause__}"
            message = message.replace(self._api_key, "[redacted]")
            message = re.sub(r"data:audio/[^\s\"']+", "[audio data redacted]", message)
            raise OmniError(f"Omni request failed: {message[:700]}") from exc

    def audio_payload(self, audio_file: str | Path, *, context: str | None = None) -> Any:
        audio = _audio_item(audio_file)  # Validate locally before contacting the model.
        system = OMNI_PROMPT if context is not None else (
            'Transcribe the attached audio in its original language. Return JSON only: '
            '{"transcript":"..."}. Return an empty transcript if speech is unintelligible. '
            'Do not invent or execute an instruction.'
        )
        user = f"Context:\n{context}\n\nPlan the instruction spoken in the audio." if context is not None else "Transcribe this audio."
        raw = self.chat([
            {"role": "system", "content": system},
            {"role": "user", "content": [{"type": "text", "text": user}, audio]},
        ])
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            raise OmniError("Omni returned invalid JSON; no task submitted.") from exc

    def transcribe_file(self, audio_file: str | Path) -> str:
        return validate_transcript(self.audio_payload(audio_file))


class OmniPlanner(Planner):
    def plan_audio(self, state: AgentState, audio_file: str | Path) -> list[PlanStep]:
        payload = self.llm.audio_payload(audio_file, context=known_context(state.known_locations))
        return self._accept_audio_payload(state, payload)

    def plan_microphone(self, state: AgentState, *, device_index: int | None = None) -> list[PlanStep]:
        payload = self.llm.microphone_payload(context=known_context(state.known_locations), device_index=device_index)
        return self._accept_audio_payload(state, payload)

    def _accept_audio_payload(self, state: AgentState, payload: Any) -> list[PlanStep]:
        transcript = validate_audio_plan(payload, state.known_locations)
        plan = super()._parse_plan_payload(payload)
        state.user_instruction = transcript
        return plan

    def replan(self, state: AgentState, failure_summary: str) -> list[PlanStep]:
        try:
            plan = super().replan(state, failure_summary)
            validate_steps([asdict(step) for step in plan], state.known_locations)
            return plan
        except OmniError:
            raise
        except (RuntimeError, ValueError, TypeError, KeyError) as exc:
            raise OmniError(f"Invalid Omni recovery plan: {exc}") from exc
