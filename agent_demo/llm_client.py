#!/usr/bin/env python3
"""DeepSeek/OpenAI-compatible chat client used by the planner."""

from __future__ import annotations

import json
import os
from typing import Any


DEFAULT_MODEL = os.environ.get("AGENT_LLM_MODEL", "deepseek-v4-pro")
DEFAULT_BASE_URL = os.environ.get("AGENT_LLM_BASE_URL", "https://api.deepseek.com")


class LLMClient:
    def __init__(self, *, api_key: str | None = None, base_url: str = DEFAULT_BASE_URL, model: str = DEFAULT_MODEL):
        try:
            from openai import OpenAI
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Missing dependency: openai. Install it with `pip install openai`, "
                "or run with --mock-planner."
            ) from exc

        self.model = model
        self.client = OpenAI(
            api_key=api_key or os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("OPENAI_API_KEY"),
            base_url=base_url,
        )

    def chat(self, messages: list[dict[str, Any]], *, temperature: float = 0.1) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=temperature,
        )
        return response.choices[0].message.content or ""


def extract_json_object(text: str) -> dict[str, Any]:
    """Extract a JSON object from plain text or fenced markdown."""
    stripped = text.strip()
    if stripped.startswith("```"):
        lines = stripped.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        stripped = "\n".join(lines).strip()
        if stripped.startswith("json"):
            stripped = stripped[4:].strip()

    try:
        return json.loads(stripped)
    except json.JSONDecodeError:
        start = stripped.find("{")
        end = stripped.rfind("}")
        if start == -1 or end == -1 or end <= start:
            raise
        return json.loads(stripped[start : end + 1])
