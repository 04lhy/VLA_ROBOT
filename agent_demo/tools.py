#!/usr/bin/env python3
"""Tool registry for the agent demo.

The current tools are intentionally lightweight. They define stable interfaces
that can later be backed by Nav2, VLN, OpenVLA, perception, and verifier nodes.
"""

from __future__ import annotations

import json
import math
import shlex
import subprocess
from pathlib import Path
from typing import Any, Callable

from agent_types import ToolResult

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROS_SETUP = PROJECT_ROOT / "base_robot_ws" / "install" / "setup.bash"
DEFAULT_VLA_TOPIC = "/vla_instruction"

# 语义地点 → map 坐标 (x, y, yaw) 的 grounding 表。
# map 帧与 Gazebo 世界帧是同一坐标系（见 maps/demo_scene.yaml 的 origin 只是地图像素左下角位置）。
# 与 nav_waypoints.py 的 WAYPOINTS 保持一致。
LOCATION_COORDS = {
    "start": (0.0, 0.0, 0.0),  # 机器人出生点 = 工厂中央走廊
    # 4 个房间的「指示牌前」坐标（世界坐标系，与 nav_waypoints.py 一致）
    # room1 西南(红)  room2 东南(蓝)  room3 东北(黄)  room4 西北(绿)
    "room1": (-20.447, -14.889, 3.1416),
    "room2": (21.333, -14.889, 0.0),
    "room3": (21.333, 14.436, 0.0),
    "room4": (-20.447, 14.122, 3.1416),
    "home": (0.0, 0.0, 0.0),
    # 每间房的「桌前」坐标（检测到红色目标时，前往该房间桌子前抓取杯子）。
    "room1_table": (-18.062, -10.526, 0.0),
    "room2_table": (15.082, -11.489, 3.1416),
    "room3_table": (15.687, 12.585, 3.1416),
    "room4_table": (-18.238, 12.883, 0.0),
}

ToolFn = Callable[..., ToolResult]


class ToolRegistry:
    def __init__(self, *, mock: bool = True, real_navigation: bool = False,
                 ros_setup: str | None = None, vla_topic: str = DEFAULT_VLA_TOPIC,
                 location_coords: dict[str, tuple[float, float, float]] | None = None):
        self.mock = mock
        self.real_navigation = real_navigation
        self.ros_setup = Path(ros_setup).expanduser() if ros_setup else DEFAULT_ROS_SETUP
        self.vla_topic = vla_topic
        self.location_coords = location_coords if location_coords is not None else LOCATION_COORDS
        self._tools: dict[str, ToolFn] = {
            "navigate_to": self.navigate_to,
            "detect_object": self.detect_object,
            "detect_color": self.detect_color,
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
        coord = self.location_coords.get(location)
        if coord is None:
            return ToolResult(
                tool="navigate_to",
                success=False,
                message=f"Unknown location: {location}",
                observations={"location": location},
            )

        if not self.real_navigation:
            return ToolResult(
                tool="navigate_to",
                success=True,
                message=f"Mock navigation reached {location}",
                observations={"location": location, "coord": coord, "mode": "mock"},
            )

        x, y, yaw = coord
        try:
            self._navigate_to_pose(x, y, yaw)
            return ToolResult(
                tool="navigate_to",
                success=True,
                message=f"导航到 {location} ({x:.2f}, {y:.2f}) 完成",
                observations={"location": location, "coord": coord, "mode": "real"},
            )
        except Exception as exc:
            return ToolResult(
                tool="navigate_to",
                success=False,
                message=f"导航失败: {exc}",
                observations={"location": location, "coord": coord},
            )

    def detect_object(self, object_name: str) -> ToolResult:
        # Replace with detector/VLM grounding later.
        return ToolResult(
            tool="detect_object",
            success=True,
            message=f"Mock detector found {object_name}",
            observations={"object_name": object_name, "confidence": 0.9, "mode": "mock"},
        )

    def detect_color(self, location: str = "") -> ToolResult:
        """识别当前相机画面中目标的颜色（绿色 / 红色）。

        接口预留：后续接入摄像头 + YOLO。
        真实实现大致流程：
          1. 订阅相机图像话题（如 /camera_sensor/image_raw，见 test.world 里的
             camera_sensor 深度相机 / camera 彩色相机）
          2. 运行 YOLO 检测目标
          3. 根据目标颜色返回 "green" / "red" / "none"

        决策约定（供 planner 分支使用，见 planner.py 注释）：
          - "green" → 巡检下一个房间
          - "red"   → 前往该房间桌子（LOCATION_COORDS 里的 `<room>_table`），
                      调用 vla_manipulate 抓取杯子再放回，然后巡检下一个房间

        当前为 mock 占位，默认返回 "green"。
        """
        # TODO: 接入摄像头 + YOLO，返回真实识别到的颜色。
        return ToolResult(
            tool="detect_color",
            success=True,
            message=f"Mock color detection at {location}: green",
            observations={"location": location, "color": "green", "mode": "mock"},
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

    def _navigate_to_pose(self, x: float, y: float, yaw: float, timeout: float = 120.0) -> None:
        """通过 ros2 action 阻塞调用 Nav2 NavigateToPose 并等待到达。"""
        if not self.ros_setup.exists():
            raise FileNotFoundError(f"ROS setup file not found: {self.ros_setup}")

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": float(x), "y": float(y), "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
            },
        }
        command = " ".join(
            [
                "source",
                shlex.quote(str(self.ros_setup)),
                "&&",
                "ros2",
                "action",
                "send_goal",
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                shlex.quote(json.dumps(goal, ensure_ascii=False)),
            ]
        )
        completed = subprocess.run(["bash", "-lc", command], text=True, capture_output=True, timeout=timeout)
        if completed.returncode != 0:
            raise RuntimeError(f"ros2 action send_goal failed\nstdout={completed.stdout}\nstderr={completed.stderr}")
