#!/usr/bin/env python3
"""Tool registry for the agent demo.

The current tools are intentionally lightweight. They define stable interfaces
that can later be backed by Nav2, VLN, OpenVLA, perception, and verifier nodes.
"""

from __future__ import annotations

import json
import math
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Callable

from agent_types import ToolResult

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROS_SETUP = PROJECT_ROOT / "base_robot_ws" / "install" / "setup.bash"
DEFAULT_VLA_TOPIC = "/vla_instruction"

# detect_color 真实推理链路。两个步骤必须用不同的解释器跑，不能塞进同一进程：
#   1) 取帧：系统 python3 + source 过的 ROS（rclpy 只在系统侧）
#   2) 推理：conda color 环境的 python（ultralytics/onnxruntime 装在那里）
COLOR_TOOLS_DIR = PROJECT_ROOT / "color_yolo" / "tools"
FRAME_CAPTURE_SCRIPT = COLOR_TOOLS_DIR / "capture_frames.py"
DETECT_ONNX_SCRIPT = COLOR_TOOLS_DIR / "detect_onnx.py"
DEFAULT_COLOR_MODEL = PROJECT_ROOT / "color_yolo" / "model" / "best.onnx"
DEFAULT_COLOR_PYTHON = "/home/lhy/miniforge3/envs/color/bin/python"

# 语义地点 → map 坐标 (x, y, yaw) 的 grounding 表。
# map 帧与 Gazebo 世界帧是同一坐标系（见 maps/demo_scene.yaml 的 origin 只是地图像素左下角位置）。
# 与 nav_waypoints.py 的 WAYPOINTS 保持一致。
LOCATION_COORDS = {
    "start": (0.0, 0.0, 0.0),  # 机器人出生点 = 工厂中央走廊
    # 4 个房间的「指示牌前」停靠位（世界坐标系，牌子正前方 2.40m，
    # 与 color_yolo/config/markers.yaml 的 dock_stations、nav_waypoints.py 三处同步）。
    # 2026-10-28：1.2m -> 2.4m，1.2m 时牌子在前置相机垂直视场外（0% 可见），
    # 且 YOLO 训练帧采自 standoff 2.15~2.65m，停靠位必须落在训练分布内。
    # room1 西南(红)  room2 东南(绿)  room3 东北(红)  room4 西北(绿)
    "room1": (-19.247, -14.889, 3.1416),
    "room2": (20.133, -14.889, 0.0),
    "room3": (20.133, 14.436, 0.0),
    "room4": (-19.247, 14.122, 3.1416),
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
                 real_detection: bool = False,
                 ros_setup: str | None = None, vla_topic: str = DEFAULT_VLA_TOPIC,
                 location_coords: dict[str, tuple[float, float, float]] | None = None,
                 color_model: str | None = None, color_python: str | None = None,
                 color_conf: float = 0.3, capture_timeout: float = 30.0):
        self.mock = mock
        self.real_navigation = real_navigation
        # detect_color 是否走真实取帧+YOLO 推理（需要仿真在跑、相机话题有数据）
        self.real_detection = real_detection
        self.ros_setup = Path(ros_setup).expanduser() if ros_setup else DEFAULT_ROS_SETUP
        self.vla_topic = vla_topic
        self.location_coords = location_coords if location_coords is not None else LOCATION_COORDS
        self.color_model = Path(color_model).expanduser() if color_model else DEFAULT_COLOR_MODEL
        self.color_python = color_python or DEFAULT_COLOR_PYTHON
        self.color_conf = color_conf
        self.capture_timeout = capture_timeout
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
        """识别当前相机画面中指示牌的颜色（red / green / none）。

        真实模式（--real-detection）两步子进程，绕开系统 python(rclpy) 与
        conda color 环境(ultralytics/onnxruntime) 无法同进程共存的限制：
          1. capture_frames.py：订阅 /front_camera_sensor/image_raw 抓 1 帧存 PNG
          2. detect_onnx.py：YOLO11n best.onnx @ imgsz320、conf 阈值，纯 CPU 推理

        决策约定（executor.py 按此插入子计划）：
          - "green" → 巡检下一个房间
          - "red"   → 前往该房间桌子（LOCATION_COORDS 里的 `<room>_table`），
                      vla_manipulate 抓取杯子再放回，然后继续巡检
          - "none"  → 画面里没有牌子（停靠位偏了 / 相机没对上），
                      调用方按失败重试或重新对准

        mock 模式（默认）恒返回 "green"，保持离线可跑。
        """
        if not self.real_detection:
            return ToolResult(
                tool="detect_color",
                success=True,
                message=f"Mock color detection at {location}: green",
                observations={"location": location, "color": "green", "mode": "mock"},
            )

        workdir = Path(tempfile.mkdtemp(prefix="agent_detect_"))
        try:
            png = self._grab_camera_frame(workdir)
            payload = self._infer_color(png)
        finally:
            shutil.rmtree(workdir, ignore_errors=True)

        color = payload.get("color", "none")
        detections = payload.get("detections", [])
        top = f"，最高置信度 {detections[0]['conf']}" if detections else ""
        return ToolResult(
            tool="detect_color",
            success=True,
            message=f"现场识别{f' {location}' if location else ''}牌子颜色: {color}{top}",
            observations={
                "location": location,
                "color": color,
                "detections": detections,
                "infer_ms": payload.get("infer_ms"),
                "mode": "real",
            },
        )

    def _grab_camera_frame(self, workdir: Path) -> Path:
        """用系统 python + ROS 环境从前置相机抓 1 帧 PNG（explicit 位姿源，不依赖相机 TF）。"""
        for path, hint in ((self.ros_setup, "先 colcon build 并 source install/setup.bash"),
                           (FRAME_CAPTURE_SCRIPT, "color_yolo 工具目录不完整"),
                           ):
            if not path.exists():
                raise FileNotFoundError(f"{path} 不存在（{hint}）")

        command = " ".join([
            "source", shlex.quote(str(self.ros_setup)), "&&",
            "python3", shlex.quote(str(FRAME_CAPTURE_SCRIPT)),
            "--out", shlex.quote(str(workdir)),
            "--tag", "live", "--count", "1", "--pose-source", "explicit",
        ])
        completed = subprocess.run(["bash", "-lc", command], text=True,
                                   capture_output=True, timeout=self.capture_timeout)
        frames = sorted(workdir.glob("live_*.png"))
        if not frames:
            raise RuntimeError(
                f"{self.capture_timeout}s 内没抓到相机帧（仿真在跑？话题有数据？）"
                f"\nstdout={completed.stdout[-500:]}\nstderr={completed.stderr[-500:]}"
            )
        return frames[0]

    def _infer_color(self, png: Path) -> dict[str, Any]:
        """用 color 环境跑 ONNX 推理，解析 detect_onnx.py 输出的最后一行 JSON。"""
        if not DETECT_ONNX_SCRIPT.exists():
            raise FileNotFoundError(f"{DETECT_ONNX_SCRIPT} 不存在")
        if not self.color_model.exists():
            raise FileNotFoundError(
                f"颜色模型不存在: {self.color_model}\n"
                "导出命令：python -c \"from ultralytics import YOLO; "
                "YOLO('color_yolo/runs/redgreen_100e/weights/best.pt').export(format='onnx', imgsz=320)\" "
                "后拷到该路径"
            )
        completed = subprocess.run(
            [self.color_python, str(DETECT_ONNX_SCRIPT),
             "--image", str(png), "--model", str(self.color_model), "--conf", str(self.color_conf)],
            text=True, capture_output=True, timeout=60,
        )
        if completed.returncode != 0:
            raise RuntimeError(f"颜色推理失败: {completed.stderr[-500:]}")
        return json.loads(completed.stdout.strip().splitlines()[-1])

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

    def _navigate_to_pose(self, x: float, y: float, yaw: float, timeout: float = 300.0) -> None:
        """通过 ros2 action 阻塞调用 Nav2 NavigateToPose 并等待到达。

        timeout 与 nav_waypoints.py 的 NAV_TIMEOUT 保持一致（新工厂跨房间路径更长，
        原 120s 偏紧，见 README V1.5.0）。
        """
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
