#!/usr/bin/env python3
"""DeepSeek/OpenAI-compatible chat client used by the planner."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


def _load_dotenv() -> None:
    """Load KEY=VALUE lines from agent_demo/.env (git-ignored secret store).

    已存在的环境变量优先（不会覆盖 export 的值）。密钥永远不进仓库。"""
    env_file = Path(__file__).resolve().parent / ".env"
    if not env_file.is_file():
        return
    for line in env_file.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


_load_dotenv()

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
