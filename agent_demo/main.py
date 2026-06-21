#!/usr/bin/env python3
"""Plan-execute agent demo for VLA/VLN style robot tasks."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict

from agent_types import AgentState
from executor import Executor
from llm_client import DEFAULT_BASE_URL, DEFAULT_MODEL, LLMClient
from planner import Planner
from tools import ToolRegistry

DEFAULT_LOCATIONS = ["start", "kitchen", "living_room", "table", "user"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan-execute robot agent demo.")
    parser.add_argument("instruction", nargs="?", default="去厨房拿一杯水回来", help="User task instruction.")
    parser.add_argument("--mock-planner", action="store_true", help="Use deterministic mock planner instead of LLM API.")
    parser.add_argument("--real-vla-publish", action="store_true", help="Actually publish vla_manipulate commands to /vla_instruction.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="LLM model name.")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL, help="OpenAI-compatible base URL.")
    parser.add_argument("--api-key", default=os.environ.get("DEEPSEEK_API_KEY"), help="LLM API key. Prefer DEEPSEEK_API_KEY.")
    parser.add_argument("--locations", default=",".join(DEFAULT_LOCATIONS), help="Comma-separated known locations for grounding.")
    parser.add_argument("--ros-setup", default=None, help="Path to base_robot_ws/install/setup.bash for real VLA publish.")
    parser.add_argument("--max-replans", type=int, default=1, help="Maximum replans after failure.")
    parser.add_argument("--print-plan", action="store_true", help="Print parsed plan JSON before execution.")
    return parser.parse_args()


def print_plan(plan) -> None:
    print("\n[Plan]")
    print(json.dumps([asdict(step) for step in plan], ensure_ascii=False, indent=2))


def main() -> None:
    args = parse_args()
    known_locations = [item.strip() for item in args.locations.split(",") if item.strip()]
    state = AgentState(
        user_instruction=args.instruction,
        known_locations=known_locations,
        max_replans=args.max_replans,
    )

    llm = None if args.mock_planner else LLMClient(api_key=args.api_key, base_url=args.base_url, model=args.model)
    planner = Planner(llm, mock=args.mock_planner)
    tools = ToolRegistry(mock=not args.real_vla_publish, ros_setup=args.ros_setup)
    executor = Executor(tools)

    print(f"[User] {args.instruction}")
    print(f"[Context] known_locations={known_locations}")
    print(f"[Tools] {tools.names}")

    plan = planner.plan(state)
    if args.print_plan:
        print_plan(plan)

    success, message = executor.execute(state, plan)
    while not success and state.replans_used < state.max_replans:
        state.replans_used += 1
        print(f"\n[Replan] attempt={state.replans_used} reason={message}")
        plan = planner.replan(state, message)
        if not plan:
            break
        if args.print_plan:
            print_plan(plan)
        success, message = executor.execute(state, plan)

    print(f"\n[Done] success={success} message={message}")


if __name__ == "__main__":
    main()
