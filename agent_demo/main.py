#!/usr/bin/env python3
"""Plan-execute agent demo for VLA/VLN style robot tasks."""

from __future__ import annotations

import argparse
import json
import os
from contextlib import ExitStack
from dataclasses import asdict

from audio_input import (
    AudioInputError, add_audio_arguments, print_input_devices, read_audio_instruction, validate_audio_arguments,
)
from agent_types import AgentState
from executor import Executor
from llm_client import DEFAULT_BASE_URL, DEFAULT_MODEL, LLMClient
from omni_planner import DEFAULT_OMNI_MODEL, OmniClient, OmniPlanner
from omni_realtime import DEFAULT_REALTIME_MODEL, OmniRealtimeClient
from planner import PATROL_ROUTE, Planner
from tools import ToolRegistry

DEFAULT_LOCATIONS = ["start", *PATROL_ROUTE]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan-execute robot agent demo.")
    parser.add_argument("instruction", nargs="?", help="User task instruction; mutually exclusive with audio input.")
    add_audio_arguments(parser)
    parser.add_argument("--audio-backend", choices=("asr", "omni", "omni-realtime"), default="asr", help="asr: Fun-ASR + text planner; omni: native HTTP file input; omni-realtime: native streaming input.")
    parser.add_argument("--omni-model", default=os.environ.get("AGENT_OMNI_MODEL") or DEFAULT_OMNI_MODEL)
    parser.add_argument("--omni-realtime-model", default=os.environ.get("AGENT_OMNI_REALTIME_MODEL") or DEFAULT_REALTIME_MODEL)
    parser.add_argument("--omni-ws-url", help="Realtime WebSocket URL; defaults to DASHSCOPE_REALTIME_URL or workspace/region gateway.")
    parser.add_argument("--omni-base-url", help="Omni endpoint; defaults to DASHSCOPE_BASE_URL or the workspace/region gateway.")
    parser.add_argument("--omni-timeout", type=float, default=60.0, help="Omni API timeout in seconds (default: 60).")
    parser.add_argument("--transcribe-only", action="store_true", help="Print audio transcription and exit without invoking the agent.")
    parser.add_argument("--plan-only", action="store_true", help="Generate and print a plan, then exit without executing tools.")
    parser.add_argument("--mock-planner", action="store_true", help="Use deterministic mock planner instead of LLM API.")
    parser.add_argument("--real-vla-publish", action="store_true", help="Actually publish vla_manipulate commands to /vla_instruction.")
    parser.add_argument("--real-navigation", action="store_true", help="Actually navigate via Nav2 (instead of mock navigate_to).")
    parser.add_argument("--real-detection", action="store_true", help="detect_color grabs a live camera frame and runs YOLO ONNX (sim must be running).")
    parser.add_argument("--color-model", default=None, help="Path to best.onnx (default: color_yolo/model/best.onnx).")
    parser.add_argument("--color-python", default=None, help="Python interpreter with ultralytics+onnxruntime (default: conda color env).")
    parser.add_argument("--color-conf", type=float, default=0.3, help="Detection confidence threshold for detect_color.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="LLM model name.")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL, help="OpenAI-compatible base URL.")
    parser.add_argument("--api-key", default=os.environ.get("DEEPSEEK_API_KEY"), help="LLM API key. Prefer DEEPSEEK_API_KEY.")
    parser.add_argument("--locations", default=",".join(DEFAULT_LOCATIONS), help="Comma-separated known locations for grounding.")
    parser.add_argument("--ros-setup", default=None, help="Path to base_robot_ws/install/setup.bash for real VLA publish.")
    parser.add_argument("--max-replans", type=int, default=1, help="Maximum replans after failure.")
    parser.add_argument("--print-plan", action="store_true", help="Print parsed plan JSON before execution.")
    args = parser.parse_args(argv)
    validate_audio_arguments(parser, args)
    has_audio = args.audio_file is not None or args.microphone
    if args.instruction is not None and (has_audio or args.list_input_devices):
        parser.error("text instruction and audio/device inputs are mutually exclusive")
    if args.transcribe_only and not has_audio:
        parser.error("--transcribe-only requires --audio-file or --microphone")
    if args.transcribe_only and args.plan_only:
        parser.error("--transcribe-only and --plan-only are mutually exclusive")
    if args.audio_backend == "omni":
        if args.audio_file is None:
            parser.error("Omni currently requires --audio-file; microphone/Realtime support is a separate backend")
    if args.audio_backend == "omni-realtime":
        if not has_audio:
            parser.error("Omni Realtime requires --microphone or --audio-file (PCM WAV replay)")
        if args.record_seconds is not None:
            parser.error("Omni Realtime ends on Enter; do not use --record-seconds")
    if args.audio_backend in {"omni", "omni-realtime"}:
        if args.mock_planner:
            parser.error("Omni generates the plan directly; omit --mock-planner (tools are mock by default)")
    return args


def print_plan(plan) -> None:
    print("\n[Plan]")
    print(json.dumps([asdict(step) for step in plan], ensure_ascii=False, indent=2))


def main() -> None:
    args = parse_args()
    try:
        with ExitStack() as resources:
            if args.list_input_devices:
                print_input_devices()
                return
            omni = None
            if args.audio_backend in {"omni", "omni-realtime"}:
                if args.audio_backend == "omni":
                    client = OmniClient(model=args.omni_model, base_url=args.omni_base_url, timeout=args.omni_timeout)
                else:
                    client = OmniRealtimeClient(
                        model=args.omni_realtime_model, ws_url=args.omni_ws_url, timeout=args.omni_timeout,
                        http_model=args.omni_model, http_base_url=args.omni_base_url,
                    )
                omni = resources.enter_context(client)
                if args.transcribe_only:
                    print(omni.transcribe_microphone(device_index=args.input_device_index) if args.microphone else omni.transcribe_file(args.audio_file))
                    return
                instruction = ""  # Filled from Omni's validated audio response below.
            elif args.audio_file is not None or args.microphone:
                instruction = read_audio_instruction(args)
                if args.transcribe_only:
                    print(instruction)
                    return
                print(f"[ASR] text: {instruction}")
            else:
                instruction = args.instruction if args.instruction is not None else "去厨房拿一杯水回来"
            if omni is None and not instruction.strip():
                raise AudioInputError("Instruction is empty.")

            known_locations = [item.strip() for item in args.locations.split(",") if item.strip()]
            state = AgentState(
                user_instruction=instruction,
                known_locations=known_locations,
                max_replans=args.max_replans,
            )
            plan = None
            if omni is not None:
                planner = OmniPlanner(omni)
                plan = planner.plan_microphone(state, device_index=args.input_device_index) if args.microphone else planner.plan_audio(state, args.audio_file)
                instruction = state.user_instruction
                label = "Omni Realtime" if args.audio_backend == "omni-realtime" else "Omni"
                metric = "response_seconds" if args.audio_backend == "omni-realtime" else "request_seconds"
                print(f"[{label}] model={omni.model} {metric}={omni.last_seconds:.2f}")
                print(f"[{label}] text: {instruction}")
            else:
                llm = None if args.mock_planner else LLMClient(api_key=args.api_key, base_url=args.base_url, model=args.model)
                planner = Planner(llm, mock=args.mock_planner)
            tools = ToolRegistry(
                mock=not args.real_vla_publish,
                real_navigation=args.real_navigation,
                real_detection=args.real_detection,
                ros_setup=args.ros_setup,
                color_model=args.color_model,
                color_python=args.color_python,
                color_conf=args.color_conf,
            )
            executor = Executor(tools)

            print(f"[User] {instruction}")
            print(f"[Context] known_locations={known_locations}")
            print(f"[Tools] {tools.names}")

            if plan is None:
                plan = planner.plan(state)
            if args.print_plan or args.plan_only:
                print_plan(plan)
            if args.plan_only:
                print("\n[Done] plan-only; no tools executed")
                return

            success, message = executor.execute(state, plan)
            while not success and state.replans_used < state.max_replans:
                state.replans_used += 1
                print(f"\n[Replan] attempt={state.replans_used} reason={message}")
                plan = planner.replan(state, message)
                if not plan:
                    break
                # 硬约束"红牌未处置不许逃逸"（终验第7趟实锤）：VLA 两连否决后 LLM 的
                # recovery plan 直接把 vla_manipulate 摘了，快进 room3→room4→home
                # 交出一枚躺杯还 Done success=True。提示词约束不住就代码兜底：
                # 历史里所有 vla_manipulate 都失败、且新计划不含任何一个 → 把失败的
                # 那步顶回去重跑（机器人此刻正停在工位 dock，原地重试语义正确）。
                vla_seen = [r for r in state.history if r.step.tool == "vla_manipulate"]

                def _vla_real_success(r):
                    # 第22趟实锤：许可泄漏让 dedup 门把失败后的重试"跳过"成 success=True
                    # ——这种零物理证据的跳过不算处置完成，否则插回守卫被自家护栏骗过。
                    if r.status != "success":
                        return False
                    msg = getattr(getattr(r, "result", None), "message", "") or ""
                    return "跳过重复操作步" not in msg

                if (vla_seen and not any(_vla_real_success(r) for r in vla_seen)
                        and not any(s.tool == "vla_manipulate" for s in plan)):
                    print("[Replan] 恢复计划丢弃了未处置的 VLA 任务 → 强制插回失败回合"
                          f"（{vla_seen[-1].step.step_id}，此前 {len(vla_seen)} 次均未成功）")
                    plan = [vla_seen[-1].step] + list(plan)
                if args.print_plan:
                    print_plan(plan)
                success, message = executor.execute(state, plan)

            print(f"\n[Done] success={success} message={message}")
    except AudioInputError as exc:
        raise SystemExit(f"[Input] {exc}") from exc
    except KeyboardInterrupt:
        raise SystemExit(130)

    if args.real_vla_publish:
        # 计划收尾（无论成败）统一关停本机推理服务器、释放 GPU。
        # 回合之间不关：7B 重载 1~3 分钟，多房间逐站开关纯浪费时间。
        tools.shutdown_vla_server()
        print(f"[Cleanup] VLA 推理服务器关停"
              f"={'成功' if tools.vla_server_stopped else '未生效（不在本机或本就没起）'}")

    # 计划失败必须以非零码退出（demo 脚本的 rc 才有意义）
    import sys
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
