#!/usr/bin/env python3
"""Plan generation and replanning."""

from __future__ import annotations

import json
from typing import Any

from agent_types import AgentState, PlanStep
from llm_client import LLMClient, extract_json_object

PLANNER_SYSTEM_PROMPT = """You are a robot task planner.
Convert the user's instruction into a short executable JSON plan.

Available tools:
- navigate_to(location: string): move the robot to a known location.
- detect_object(object_name: string): locate a target object in the current scene.
- vla_manipulate(instruction: string): run a local manipulation instruction through OpenVLA.
- verify_navigation(location: string): verify the robot reached a location.
- verify_object_visible(object_name: string): verify an object is visible.
- verify_grasp(object_name?: string): verify the robot has grasped an object.
- verify_condition(condition: string): generic verification placeholder.

Rules:
- Return JSON only. No markdown, no extra prose.
- Use only known locations provided in context. If a location is unknown, ask for clarification by returning needs_clarification=true.
- Keep each step atomic.
- Add verify for steps whose success matters.
- Do not invent low-level joint or velocity commands.

JSON schema:
{
  "needs_clarification": false,
  "clarification_question": "",
  "steps": [
    {
      "step_id": "1",
      "description": "go to kitchen",
      "tool": "navigate_to",
      "args": {"location": "kitchen"},
      "verify": {"tool": "verify_navigation", "args": {"location": "kitchen"}}
    }
  ]
}
"""


def known_context(known_locations: list[str]) -> str:
    return json.dumps(
        {
            "known_locations": known_locations,
            "planning_style": "plan_execute",
        },
        ensure_ascii=False,
        indent=2,
    )


class Planner:
    def __init__(self, llm: LLMClient | None = None, *, mock: bool = False):
        self.llm = llm
        self.mock = mock

    def plan(self, state: AgentState) -> list[PlanStep]:
        if self.mock:
            return self._mock_plan(state.user_instruction)
        if self.llm is None:
            raise RuntimeError("LLM client is required when mock planner is disabled")

        messages = [
            {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
            {"role": "user", "content": f"Context:\n{known_context(state.known_locations)}\n\nUser instruction:\n{state.user_instruction}"},
        ]
        raw = self.llm.chat(messages)
        payload = extract_json_object(raw)
        return self._parse_plan_payload(payload)

    def replan(self, state: AgentState, failure_summary: str) -> list[PlanStep]:
        if self.mock:
            return []
        if self.llm is None:
            raise RuntimeError("LLM client is required when mock planner is disabled")

        history_payload = [
            {
                "step_id": record.step.step_id,
                "tool": record.step.tool,
                "status": record.status,
                "result": record.result.message if record.result else None,
                "verify_result": record.verify_result.message if record.verify_result else None,
            }
            for record in state.history
        ]
        messages = [
            {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": (
                    f"Original instruction:\n{state.user_instruction}\n\n"
                    f"Context:\n{known_context(state.known_locations)}\n\n"
                    f"Execution history:\n{json.dumps(history_payload, ensure_ascii=False, indent=2)}\n\n"
                    f"Failure summary:\n{failure_summary}\n\n"
                    "Return a recovery plan with only the remaining steps."
                ),
            },
        ]
        raw = self.llm.chat(messages)
        payload = extract_json_object(raw)
        return self._parse_plan_payload(payload)

    def _parse_plan_payload(self, payload: dict[str, Any]) -> list[PlanStep]:
        if payload.get("needs_clarification"):
            question = payload.get("clarification_question") or "Need clarification."
            raise RuntimeError(f"Planner needs clarification: {question}")

        steps = payload.get("steps")
        if not isinstance(steps, list):
            raise ValueError("Planner response must contain a steps list")

        parsed: list[PlanStep] = []
        for index, item in enumerate(steps, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"Plan step {index} is not an object")
            parsed.append(
                PlanStep(
                    step_id=str(item.get("step_id") or index),
                    tool=str(item["tool"]),
                    args=dict(item.get("args") or {}),
                    verify=item.get("verify"),
                    description=str(item.get("description") or ""),
                )
            )
        return parsed

    def _mock_plan(self, instruction: str) -> list[PlanStep]:
        text = instruction.lower()
        if "kitchen" in text or "厨房" in instruction:
            return [
                PlanStep("1", "navigate_to", {"location": "kitchen"}, {"tool": "verify_navigation", "args": {"location": "kitchen"}}, "go to kitchen"),
                PlanStep("2", "detect_object", {"object_name": "cup"}, {"tool": "verify_object_visible", "args": {"object_name": "cup"}}, "find cup"),
                PlanStep("3", "vla_manipulate", {"instruction": "pick up the cup"}, {"tool": "verify_grasp", "args": {"object_name": "cup"}}, "pick up cup"),
                PlanStep("4", "navigate_to", {"location": "start"}, {"tool": "verify_navigation", "args": {"location": "start"}}, "return to start"),
                PlanStep("5", "vla_manipulate", {"instruction": "place the cup on the table"}, {"tool": "verify_condition", "args": {"condition": "cup placed on table"}}, "place cup"),
            ]
        return [PlanStep("1", "vla_manipulate", {"instruction": instruction}, None, "direct VLA command")]
