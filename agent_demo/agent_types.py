#!/usr/bin/env python3
"""Shared data structures for the plan-execute agent demo."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

StepStatus = Literal["pending", "running", "success", "failed", "skipped"]


@dataclass
class PlanStep:
    step_id: str
    tool: str
    args: dict[str, Any] = field(default_factory=dict)
    verify: dict[str, Any] | None = None
    description: str = ""


@dataclass
class ToolResult:
    tool: str
    success: bool
    message: str
    observations: dict[str, Any] = field(default_factory=dict)
    suggested_recovery: list[str] = field(default_factory=list)


@dataclass
class StepRecord:
    step: PlanStep
    status: StepStatus
    result: ToolResult | None = None
    verify_result: ToolResult | None = None


@dataclass
class AgentState:
    user_instruction: str
    plan: list[PlanStep] = field(default_factory=list)
    history: list[StepRecord] = field(default_factory=list)
    known_locations: list[str] = field(default_factory=list)
    max_replans: int = 1
    replans_used: int = 0
