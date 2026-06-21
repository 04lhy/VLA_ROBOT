#!/usr/bin/env python3
"""Plan executor with tool results and verification records."""

from __future__ import annotations

from agent_types import AgentState, PlanStep, StepRecord, ToolResult
from tools import ToolRegistry


class Executor:
    def __init__(self, tools: ToolRegistry):
        self.tools = tools

    def execute(self, state: AgentState, plan: list[PlanStep]) -> tuple[bool, str]:
        state.plan = plan
        for step in plan:
            print(f"\n[Execute] step={step.step_id} tool={step.tool} args={step.args}")
            result = self.tools.call(step.tool, step.args)
            print(f"[Tool] success={result.success} message={result.message}")

            record = StepRecord(step=step, status="success" if result.success else "failed", result=result)
            if not result.success:
                state.history.append(record)
                return False, result.message

            if step.verify:
                verify_tool = str(step.verify.get("tool"))
                verify_args = dict(step.verify.get("args") or {})
                print(f"[Verify] tool={verify_tool} args={verify_args}")
                verify_result = self.tools.call(verify_tool, verify_args)
                print(f"[Verify] success={verify_result.success} message={verify_result.message}")
                record.verify_result = verify_result
                if not verify_result.success:
                    record.status = "failed"
                    state.history.append(record)
                    return False, verify_result.message

            state.history.append(record)

        return True, "plan completed"
