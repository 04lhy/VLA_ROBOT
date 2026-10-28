#!/usr/bin/env python3
"""Plan executor with tool results, verification records and color branching.

唯一的动态分支：detect_color 返回 "red" 时，在原计划继续之前插入
「去该房间桌前 -> 抓取杯子 -> 放回杯子」子计划（决策约定见 tools.detect_color）。
green / none 不分支，按原计划走。
"""

from __future__ import annotations

from collections import deque

from agent_types import AgentState, PlanStep, StepRecord, ToolResult
from tools import ToolRegistry


def _red_grasp_subplan(step: PlanStep, room: str) -> list[PlanStep]:
    table = f"{room}_table"
    return [
        PlanStep(f"{step.step_id}.1", "navigate_to", {"location": table},
                 {"tool": "verify_navigation", "args": {"location": table}},
                 f"{room} 识别到红色：去桌前"),
        PlanStep(f"{step.step_id}.2", "vla_manipulate", {"instruction": "抓取杯子"},
                 {"tool": "verify_grasp", "args": {"object_name": "cup"}},
                 f"{room}：抓取杯子"),
        PlanStep(f"{step.step_id}.3", "vla_manipulate", {"instruction": "放回杯子"},
                 {"tool": "verify_condition", "args": {"condition": "cup placed back"}},
                 f"{room}：放回杯子"),
    ]


class Executor:
    def __init__(self, tools: ToolRegistry):
        self.tools = tools

    def execute(self, state: AgentState, plan: list[PlanStep]) -> tuple[bool, str]:
        state.plan = plan
        steps = deque(plan)
        while steps:
            step = steps.popleft()
            print(f"\n[Execute] step={step.step_id} tool={step.tool} args={step.args}")
            result = self.tools.call(step.tool, step.args)
            print(f"[Tool] success={result.success} message={result.message}")

            if step.tool == "detect_color" and result.success \
                    and result.observations.get("color") == "red":
                room = str(result.observations.get("location") or step.args.get("location") or "")
                if room and f"{room}_table" in self.tools.location_coords:
                    sub = _red_grasp_subplan(step, room)
                    steps.extendleft(reversed(sub))
                    print(f"[Branch] detect_color=red @ {room} -> 插入子计划: "
                          + " -> ".join(s.tool for s in sub))
                else:
                    print(f"[Branch] detect_color=red 但没有 {room!r} 的桌前坐标，跳过抓取")

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
