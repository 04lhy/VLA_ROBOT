#!/usr/bin/env python3
"""Tool registry for the agent demo.

The current tools are intentionally lightweight. They define stable interfaces
that can later be backed by Nav2, VLN, OpenVLA, perception, and verifier nodes.
"""

from __future__ import annotations

import json
import shlex
import subprocess
from pathlib import Path
from typing import Any, Callable

from agent_types import ToolResult

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROS_SETUP = PROJECT_ROOT / "base_robot_ws" / "install" / "setup.bash"
DEFAULT_VLA_TOPIC = "/vla_instruction"

ToolFn = Callable[..., ToolResult]


class ToolRegistry:
    def __init__(self, *, mock: bool = True, ros_setup: str | None = None, vla_topic: str = DEFAULT_VLA_TOPIC):
        self.mock = mock
        self.ros_setup = Path(ros_setup).expanduser() if ros_setup else DEFAULT_ROS_SETUP
        self.vla_topic = vla_topic
        self._tools: dict[str, ToolFn] = {
            "navigate_to": self.navigate_to,
            "detect_object": self.detect_object,
            "vla_manipulate": self.vla_manipulate,
            "verify_navigation": self.verify_navigation,
            "verify_grasp": self.verify_grasp,
            "verify_object_visible": self.verify_object_visible,
            "verify_condition": self.verify_condition,
        }

    @property
    def names(self) -> list[str]:
        return sorted(self._tools)

    def call(self, tool: str, args: dict[str, Any]) -> ToolResult:
        if tool not in self._tools:
            return ToolResult(tool=tool, success=False, message=f"Unknown tool: {tool}")
        try:
            return self._tools[tool](**args)
        except TypeError as exc:
            return ToolResult(tool=tool, success=False, message=f"Invalid args for {tool}: {exc}")
        except Exception as exc:
            return ToolResult(tool=tool, success=False, message=f"Tool {tool} raised: {exc}")

    def navigate_to(self, location: str) -> ToolResult:
        # Replace with Nav2 action client or VLN service later.
        return ToolResult(
            tool="navigate_to",
            success=True,
            message=f"Mock navigation reached {location}",
            observations={"location": location, "mode": "mock"},
        )

    def detect_object(self, object_name: str) -> ToolResult:
        # Replace with detector/VLM grounding later.
        return ToolResult(
            tool="detect_object",
            success=True,
            message=f"Mock detector found {object_name}",
            observations={"object_name": object_name, "confidence": 0.9, "mode": "mock"},
        )

    def vla_manipulate(self, instruction: str) -> ToolResult:
        if self.mock:
            return ToolResult(
                tool="vla_manipulate",
                success=True,
                message=f"Mock VLA instruction accepted: {instruction}",
                observations={"instruction": instruction, "mode": "mock"},
            )

        self._publish_ros_string(self.vla_topic, instruction)
        return ToolResult(
            tool="vla_manipulate",
            success=True,
            message=f"Published VLA instruction to {self.vla_topic}: {instruction}",
            observations={"topic": self.vla_topic, "instruction": instruction},
        )

    def verify_navigation(self, location: str) -> ToolResult:
        # Replace with Nav2 result + TF/AMCL pose threshold check later.
        return ToolResult(
            tool="verify_navigation",
            success=True,
            message=f"Mock navigation verification passed for {location}",
            observations={"location": location, "distance_to_goal": 0.0, "mode": "mock"},
        )

    def verify_grasp(self, object_name: str | None = None) -> ToolResult:
        # Replace with gazebo_grasp_plugin events or real gripper/contact sensing.
        return ToolResult(
            tool="verify_grasp",
            success=True,
            message=f"Mock grasp verification passed{f' for {object_name}' if object_name else ''}",
            observations={"object_name": object_name, "attached": True, "mode": "mock"},
        )

    def verify_object_visible(self, object_name: str) -> ToolResult:
        return ToolResult(
            tool="verify_object_visible",
            success=True,
            message=f"Mock visibility verification passed for {object_name}",
            observations={"object_name": object_name, "visible": True, "mode": "mock"},
        )

    def verify_condition(self, condition: str) -> ToolResult:
        return ToolResult(
            tool="verify_condition",
            success=True,
            message=f"Mock condition verification passed: {condition}",
            observations={"condition": condition, "mode": "mock"},
        )

    def _publish_ros_string(self, topic: str, text: str) -> None:
        if not self.ros_setup.exists():
            raise FileNotFoundError(f"ROS setup file not found: {self.ros_setup}")

        payload = {"data": text}
        command = " ".join(
            [
                "source",
                shlex.quote(str(self.ros_setup)),
                "&&",
                "ros2",
                "topic",
                "pub",
                "--once",
                shlex.quote(topic),
                "std_msgs/msg/String",
                shlex.quote(json.dumps(payload, ensure_ascii=False)),
            ]
        )
        completed = subprocess.run(["bash", "-lc", command], text=True, capture_output=True, timeout=20)
        if completed.returncode != 0:
            raise RuntimeError(f"ros2 topic pub failed\nstdout={completed.stdout}\nstderr={completed.stderr}")
