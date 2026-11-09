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
import time
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
    # room2_table 特意 = VLA 东南工位第十q轮验证位（导航粗到位后
    # vla_manipulate 的 dock snap 精对齐，见 DOCK_POSES）。
    "room1_table": (-18.062, -10.526, 0.0),
    "room2_table": (14.152, -11.868, 1.5708),
    "room3_table": (15.687, 12.585, 3.1416),
    "room4_table": (-18.238, 12.883, 0.0),
}

# VLA 抓取工位精对齐（第十q轮验证位）。模型只在固定机位采过演示数据，
# Nav2/AMCL 停靠精度（±5~25cm 级）不足以复现该位姿下的杯-爪几何，
# 故抓取回合开始前用 gz teleport 把底盘+杯子精对齐；杯子回名义位
# （世界默认四杯叠在原地，演示摆盘本来就要传送）。其余房间暂无 VLA 工位。
DOCK_POSES = {
    "room2": {
        "base": (14.152, -11.868, 1.5708),
        "cup_model": "plastic_cup_0_clone",
        "cup": (14.152, -11.514, 0.456645),
    },
}

# 训练语言指令原文（ep meta instruction）。中文文案模型没见过，必须用它。
VLA_GRASP_INSTRUCTION = "pick up the cup and pour"

ToolFn = Callable[..., ToolResult]


class ToolRegistry:
    def __init__(self, *, mock: bool = True, real_navigation: bool = False,
                 real_detection: bool = False,
                 ros_setup: str | None = None, vla_topic: str = DEFAULT_VLA_TOPIC,
                 location_coords: dict[str, tuple[float, float, float]] | None = None,
                 color_model: str | None = None, color_python: str | None = None,
                 color_conf: float = 0.3, capture_timeout: float = 30.0,
                 vla_wait_timeout: float = 420.0,
                 vla_shutdown_after_success: bool = False):
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
        # VLA 回合阻塞上限：预热 ~15s + 推理 57 步 ~76s，留一倍余量
        self.vla_wait_timeout = vla_wait_timeout
        # 抓取去重（工位许可制）：每次导航到新位置发放一次许可，vla_manipulate
        # 消耗许可；同一停留点的重复操作步（如 LLM 多写的抓取）直接无操作跳过，
        # 避免原地重复 预热+整段回合。多房间巡检天然靠"到桌前导航"逐站续许可。
        # 推理服务器在整轮巡检期间常驻（7B 重载 1~3 分钟，逐房间开关纯浪费时间；
        # 空闲时零计算，只占显存），整个计划结束由调用方/脚本统一关停。
        # 单任务场景想 COMPLETE 即关服务器可传 vla_shutdown_after_success=True。
        self._vla_permit = True
        # VLA 回合结束后待复核的工位：底盘离开该工位之后才能判定杯子是否真被放下
        self._cup_leave_check: str | None = None
        # 上一段回合的物理证据（抬升/末态），供 verify_grasp 用，不再让 mock 说真话
        self._last_grasp_evidence: dict[str, Any] | None = None
        # 判定"真夹起来了"的最小杯底抬升（米）。演示 episode 的物理抬升上限只有
        # ~5cm（GT ee z 0.221→0.273，抓点约束），5cm 阈值正好卡在边缘：桥接稍慢
        # 一点就被判成"空抓"（run15 实录 2.3cm 假阴性）。3.5cm 留出执行容差，
        # 同时仍高于指尖蹭杯造成的 ~2cm 位移。
        self.grasp_lift_min = 0.035
        self.vla_shutdown_after_success = vla_shutdown_after_success
        self.vla_server_stopped: bool | None = None
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
        # 移动到新位置 = 重新允许一次 VLA 回合（工位许可，供 vla_manipulate 消耗）
        self._vla_permit = True

        if not self.real_navigation:
            return ToolResult(
                tool="navigate_to",
                success=True,
                message=f"Mock navigation reached {location}",
                observations={"location": location, "coord": coord, "mode": "mock"},
            )

        x, y, yaw = coord
        try:
            self._relocalize_from_gz()
            self._navigate_to_pose(x, y, yaw)
            self._verify_arrival_gt(location, x, y, yaw)
            message = f"导航到 {location} ({x:.2f}, {y:.2f}) 完成"
            observations: dict[str, Any] = {"location": location, "coord": coord,
                                            "mode": "real"}
            if self._cup_leave_check is not None:
                # 底盘已离开工位：这一刻才能区分"杯子真被放下"还是"夹在爪里被带走"
                room = self._cup_leave_check
                self._cup_leave_check = None
                left = self._cup_left_at_station(room)
                observations["cup_left_at_station"] = {"room": room, "left": left}
                if left is True:
                    message += f"；{room} 工位杯复核：已松手、留在桌面上"
                elif left is False:
                    # 离站复核是补药任务唯一"杯还在工位"的物证（加测第2趟：
                    # 回合内 8.6cm 抬升却在放桌时滑倒，靠这一步戳穿）。False
                    # 必须失败整个步骤，逼重规划回工位重做，不能让倒杯混过验收。
                    message += f"；{room} 工位杯复核：**没留在工位**——杯子被带离/消失/倒伏"
                    return ToolResult(tool="navigate_to", success=False,
                                      message=message, observations=observations)
                else:
                    message += f"；{room} 工位杯复核：读不到杯子位姿"
            return ToolResult(tool="navigate_to", success=True,
                              message=message, observations=observations)
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

    def vla_manipulate(self, instruction: str, dock: str | None = None,
                       wait_timeout: float | None = None) -> ToolResult:
        """起一个 VLA 回合并**阻塞到整段执行完**（桥接 /vla_episode_status）。

        dock：DOCK_POSES 的房间名（如 "room2"）。给定则先用 gz teleport 精对齐
        底盘+杯子到验证工位——模型只在固定机位训练过，Nav2 停靠精度不够。
        桥接端收到指令会先自动预热到 GT f1 臂位（auto_warm），这里不用管。
        回合=整条 抓杯→倾倒→放回→松开 演示，一个 vla_manipulate 即完成
        「抓取+补药+放回」，无需再发第二条指令。
        """
        if self.mock:
            return ToolResult(
                tool="vla_manipulate",
                success=True,
                message=f"Mock VLA instruction accepted: {instruction}",
                observations={"instruction": instruction, "mode": "mock"},
            )

        if not self._vla_permit:
            # 上一段回合结束后没有新的导航（如 LLM 在同一停留点多写的抓取步）：
            # 无操作跳过，计划继续；下一次 navigate_to 会重新放行。
            return ToolResult(
                tool="vla_manipulate", success=True,
                message="同一停留点已完成过 VLA 回合，跳过重复操作步，继续后续计划",
                observations={"instruction": instruction, "mode": "duplicate_skipped",
                              "skipped": True},
            )

        timeout = wait_timeout if wait_timeout is not None else self.vla_wait_timeout
        if dock is not None and dock not in DOCK_POSES:
            return ToolResult(
                tool="vla_manipulate", success=False,
                message=f"未知 VLA 工位: {dock}（可选 {sorted(DOCK_POSES)}）",
            )
        if dock is None:
            # LLM 重规划会丢 dock 参数（终验第11趟 ep2 实锤：改写指令"抓杯倾倒
            # 补药"不带工位 → 不 teleport 对齐、不采样抬升、不查末态姿态、不挂
            # 离站复核，桥接明明 WARN"杯悬空 0.515 搁在指尖上"仍 success=True）。
            # 兜底：按底盘 gz 真值就近推断 DOCK_POSES（1.2m 内），推不出就**拒
            # 起无复核回合**——宁要一次失败重规划，不要一枚没验证过的"成功"。
            pose = None
            try:
                pose = self._gz_world_pose()
            except Exception:
                pose = None
            best, bestd = None, 1.2
            if pose is not None:
                for name, spec in DOCK_POSES.items():
                    d = math.hypot(pose[0] - spec["base"][0], pose[1] - spec["base"][1])
                    if d < bestd:
                        best, bestd = name, d
            if best is not None:
                dock = best
                print(f"[Tool] vla_manipulate 缺 dock，按底盘 GT 推断工位={best}"
                      f"（距离 {bestd:.2f}m），强制走完整物理复核")
            else:
                return ToolResult(
                    tool="vla_manipulate", success=False,
                    message="VLA 步骤缺 dock 参数且机器人不在任何已标定格位（1.2m 内），"
                            "无法对齐/复核——拒绝起无复核回合（导航回工位或补 dock 后重试）",
                )

        try:
            if dock is not None:
                self._snap_dock(dock)
                # snap 已把底盘 teleport 到工位——AMCL 从这一刻起就是旧的。
                # 重定位必须发在**确认段之前**：未送达/读不通的早退路径
                # （续测第3趟 status=6 连败实锤）原本跳过了回合后的
                # initialpose，失同步一路带进下一次导航被 controller
                # ABORTED(status=6) 反复吃预算。
                bx, by, byaw = DOCK_POSES[dock]["base"]
                try:
                    self._publish_initialpose(bx, by, byaw)
                except Exception:
                    pass
            landed, secs_to_land = self._publish_and_confirm(instruction)
        except Exception as exc:
            return ToolResult(
                tool="vla_manipulate", success=False,
                message=f"VLA 回合启动失败: {exc}",
                observations={"instruction": instruction, "dock": dock},
            )
        self._vla_permit = False  # 许可已消耗；再次导航会重新放行
        if landed is None:
            # 第22趟 ep2 实锤：NO_CONFIRM 早退不回灌许可 → 下一回合撞
            # "同一停留点已完成过"门被假跳过判 success=True，躺杯放行巡检。
            # 回合根本没落地＝许可未被消耗，必须回灌（桥接真在跑的话，
            # INSTRUCTION-IGNORE/ECHO-DROP 双护栏自会吸收重发）。
            self._vla_permit = True
            return ToolResult(
                tool="vla_manipulate", success=False,
                message="指令未送达桥接或状态读不通（窗口内无 WARMING/RUNNING 回执；"
                        "读不通时已抑制重发防幽灵回合，桥接可能其实在跑），"
                        "检查 vla_gazebo_bridge 是否在跑",
                observations={"instruction": instruction, "dock": dock,
                              "final_status": "NO_CONFIRM"},
            )

        status, secs, lift = self._wait_vla_episode(max(timeout - secs_to_land, 30.0),
                                                    seen_run=True, cup_room=dock)
        secs += secs_to_land  # 总时长含"发布→桥接接住"的确认段
        if dock is not None:
            # dock 传送 + 回合内底座反冲会让 AMCL/odom 失同步，local costmap 与
            # 全局图对不上号，下一次导航被 controller 判 "None of the points of
            # the global plan were in the local costmap" 而 ABORTED(status=6)
            # ——2026-11-04 彩排#5 实录。回合收口立即用 dock 底盘 GT 发
            # /initialpose 强制 AMCL 重新收粒子；失败不致命（导航自带清图重试）。
            bx, by, byaw = DOCK_POSES[dock]["base"]
            try:
                self._publish_initialpose(bx, by, byaw)
            except Exception:
                pass
        cup_upright = self._cup_upright(dock) if dock is not None else None

        ok = status == "COMPLETE"
        # "夹没夹起来"用回合内的最大抬升判，不能信 COMPLETE：GT 帧放完只代表模型
        # 把整段动作背完了，run13 实测横向偏 4cm 捏空手，杯子纹丝不动也照样 COMPLETE。
        grasped: bool | None = None
        if dock is not None:
            if lift is None:
                ok = False
                grasped = None
            else:
                grasped = lift >= self.grasp_lift_min
                ok = ok and grasped
            # 加测第2趟 ep2 实锤：抬升 8.6cm 真抓+COMPLETE，但放桌瞬间滑倒
            # （终读 R=1.49）——旧版 ok 不看末态姿态，success=True 带着倒杯
            # 继续巡检还触发了直退。立着留在桌面是任务定义的一部分。
            if cup_upright is False:
                ok = False
        if not ok:
            self._vla_permit = True  # 回合未成，留许可允许后续步骤重试
        self._last_grasp_evidence = {
            "room": dock, "final_status": status,
            "lift_m": None if lift is None else round(lift, 3),
            "grasped": grasped, "upright": cup_upright,
            "elapsed_s": round(secs, 1),
        }

        extra = ""
        if lift is not None:
            if grasped:
                extra += f"，已夹起（回合内最大抬升 {lift * 100:.1f}cm）"
            elif lift < 0:
                extra += f"，杯子被压低了 {-lift * 100:.1f}cm 且从未抬升＝没夹住"
            else:
                extra += (f"，杯子最大只动了 {lift * 100:.1f}cm"
                          f"（< {self.grasp_lift_min * 100:.0f}cm 阈值）＝空抓：手指没套住杯子")
        elif dock is not None:
            extra += "，读不到杯子位姿，无法确认是否夹起"
        else:
            extra += "（该工位无 DOCK_POSES 标定，未做物理复核）"
        if cup_upright is not None:
            extra += ("，末态平在桌面高度带" if cup_upright
                      else "，但末态倒伏/离桌")
        if ok and self.vla_shutdown_after_success:
            self.shutdown_vla_server()
            extra += ("，推理服务器已关停" if self.vla_server_stopped
                      else "，推理服务器关停未生效（可能不在本机）")
        # "是否真的松手"这一拍读不出来（杯子夹在爪里举在臂上同样落在高度带里），
        # 交给下一段导航离开工位后的 _cup_left_at_station 复核。
        if dock is not None:
            # 只在回合**成功**时挂离站复核：失败回合杯本就歪着（正是要重试的
            # 原因），若也挂上，重规划回工位的路上会被这条 False 复核反手锁死。
            self._cup_leave_check = dock if ok else None
            # [2026-11-07 RUN3 实锤] 杯子落桌复核明明过了（z=0.457），离站转身后
            # 却被拍飞：dock 几何里杯心距底盘中心仅 ~0.35m，小于底盘原地旋转的
            # 对角扫掠半径（~0.4m），Nav2 起步转向会把刚放好的杯子拍下桌。
            # 回合成功后先 gz teleport 沿 -yaw 直退 0.7m（**不旋转**），退出扫掠
            # 圈再交还导航；下一段 navigate_to 开头的 _relocalize_from_gz 会把
            # AMCL 拉回真值，传送不破坏导航。失败路径不退（replan 会重新 snap）。
            if ok:
                self._retreat_from_dock(dock)
        fail = "" if ok else f"（末状态={status}，{secs:.0f}s）"
        return ToolResult(
            tool="vla_manipulate", success=ok,
            message=f"VLA 回合{'完成' if ok else '未达成'}（{secs:.0f}s，含预热）{extra}{fail}",
            observations={"topic": self.vla_topic, "instruction": instruction,
                          "dock": dock, "final_status": status,
                          "elapsed_s": round(secs, 1),
                          "cup_lift_m": None if lift is None else round(lift, 3),
                          "grasped": grasped,
                          "cup_upright_after": cup_upright},
        )

    def shutdown_vla_server(self) -> None:
        """关掉本机 v4 推理服务器（deploy_4bit.py），释放 GPU/内存。

        调用时机=整个巡检计划收尾（回合之间不关，7B 重载要 1~3 分钟）。
        尽力而为：服务器不在本机时 pkill 杀不到，只置 vla_server_stopped=False。
        """
        try:
            completed = subprocess.run(
                ["bash", "-lc", "pkill -f 'deploy_4bit[.]py'"],
                text=True, capture_output=True, timeout=15)
            self.vla_server_stopped = completed.returncode == 0
        except Exception:
            self.vla_server_stopped = False

    def _gz_world_pose(self, model: str = "base_robot") -> tuple[float, float, float] | None:
        """读 gz 世界位姿 (x, y, yaw)：`gz model -p` 输出 x y z R P Y（弧度）。

        demo_scene 的导航地图由 SDF 几何直接栅格化，map 系=世界系，
        所以这个真值可以直接当 AMCL 的 initialpose 用。读不到返回 None。
        """
        try:
            completed = subprocess.run(
                ["bash", "-lc", f"timeout 8 gz model -m {shlex.quote(model)} -p"],
                text=True, capture_output=True, timeout=20)
            parts = completed.stdout.split()
            return float(parts[0]), float(parts[1]), float(parts[5])
        except Exception:
            return None

    def _relocalize_from_gz(self) -> None:
        """导航前用 gz 真值硬重置 AMCL（仅仿真）。

        dock teleport 会把底盘瞬移、回合内臂体反冲又甩一次，AMCL 单靠激光在
        四间几乎同构的厂房里会锁到隔壁房间——2026-11-04 实录：真值 (1.72,-0.77)
        而 /amcl_pose 报 (2.99,7.86)，差 8.6m，于是 map 系全局规划的起点根本
        不在 odom 系 local costmap 窗口内，DWB 刷
        "None of the points of the global plan were in the local costmap" →
        Controller patience exceeded → ABORTED(status=6)，回 home 段连拒 3 次。
        尽力而为：拿不到真值或发布失败就照常发 goal，不引入新失败面。
        """
        gt = self._gz_world_pose()
        if gt is None:
            return
        try:
            self._publish_initialpose(*gt)
        except Exception:
            return
        time.sleep(4.0)  # 留给 AMCL 几次 /scan 重收粒子
        self._wait_tf_fresh()

    def _wait_tf_fresh(self, max_wait: float = 25.0) -> None:
        """等 map→base_footprint TF 的**时标**追上仿真时钟（尽力而为，不抛错）。

        2026-11-07 run3 实录：room3 收尾后 map 链 TF 时标冻结了几分钟，
        global costmap 把每帧激光都 "Message Filter dropping"，Nav2 盲跑——
        room4 段原地不动 96s 却被 controller 报 "Reached the goal!"（假到达），
        回 home 头两发 300s 干等。goal 发出前确认 TF 新鲜，能把这种
        "半死不活的导航栈"挡在门外，代价只是多等几秒。
        """
        if not self.ros_setup.exists():
            return
        py = r'''
import sys, time, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
max_wait = float(sys.argv[1])
# 只盯 base 侧链（odom->base_footprint/base_link），这是 run3 里冻结的那一段。
# sim 时间用 node.get_clock()（rclpy 内部按正确 QoS 订 /clock——gz 的 /clock
# 是 best_effort，手工订阅 RELIABLE 会"discovered incompatible QoS"一帧收不到，
# 2026-11-07 冒烟实测）。child_frame_id 在 TransformStamped 本体上
# ——既不在 .header 也不在 .transform（两连错冒烟实测）；.header.stamp 才是时标。
CHILDREN = ("odom", "base_footprint", "base_link")
state = {"tf": 0.0}
def on_tf(msg):
    now = node.get_clock().now().nanoseconds * 1e-9
    for t in msg.transforms:
        if t.child_frame_id not in CHILDREN:
            continue
        ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if now > 0 and ts > now + 2.0:   # 未来时标不算"新鲜"
            continue
        state["tf"] = max(state["tf"], ts)
try:
    rclpy.init()
    node = Node("agent_tf_fresh",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    deadline = time.time() + max_wait
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        now = node.get_clock().now().nanoseconds * 1e-9
        if now > 0 and state["tf"] > 0:
            age = now - state["tf"]
            # 只防"滞后冻结"（run3 病根），略超前（gz odom 时标本就压在
            # /clock 前 ~1s，冒烟实测 age=-1.0）也算新鲜
            if age < 1.5:
                print("TF_FRESH age=%.2fs" % age)
                sys.exit(0)
    now = node.get_clock().now().nanoseconds * 1e-9
    print("TF_STALE age=%.1fs clock=%.1f tf=%.1f" % (
        now - state["tf"], now, state["tf"]))
    sys.exit(0)
finally:
    try:
        rclpy.shutdown()
    except Exception:
        pass
'''
        command = " && ".join([
            f"source {shlex.quote(str(self.ros_setup))}",
            f"python3 - {max_wait:.1f} <<'PYEOF'\n{py}PYEOF",
        ])
        try:
            completed = subprocess.run(["bash", "-lc", command], text=True,
                                       capture_output=True, timeout=max_wait + 25)
            out = completed.stdout.strip()
            err = completed.stderr.strip()
            tail = (out.splitlines()[-1] if out else
                    err.splitlines()[-1] if err else "no output")
            print(f"[TF] rc={completed.returncode} {tail}", flush=True)
        except Exception:
            pass

    def _verify_arrival_gt(self, location: str, x: float, y: float, yaw: float) -> None:
        """Nav2 报成功后用 gz 真值复核"真到了"；偏差过大抛错交给上层重规划。

        run3 实锤：map TF 冻结时 controller 能对 40m 外的目标报
        "Reached the goal!"——AMCL 报成功、mock verify 也放行，agent 却压根
        没去 room4。真值复核是唯一能戳穿这种假到达的硬手段（仿真 map=世界系，
        gz model -p 读到的就是真位姿）。容差取 1.25m：正常到达实测偏差
        ≤0.25m，假到达差 40m，两个量级中间怎么都不会误杀。
        """
        gt = self._gz_world_pose()
        if gt is None:
            return  # 读不到真值不下结论（transport 挂了也不能把好导航判死）
        gx, gy, gyaw = gt
        dist = math.hypot(gx - x, gy - y)
        dyaw = abs(math.atan2(math.sin(gyaw - yaw), math.cos(gyaw - yaw)))
        if dist > 1.25 or dyaw > 0.6:
            # 定罪之前先把 GT 真值回灌 initialpose 拽正 AMCL：不拽的话，重
            # 规划重试仍在 Nav2 自己的幻觉坐标里做决策（加测第11趟 home 实锤：
            # (-0.80,1.02) 同点位连拒两次，每发都"Reached goal"在它信的地方）。
            try:
                self._publish_initialpose(gx, gy, gyaw)
            except Exception:
                pass
            raise RuntimeError(
                f"NAV_GT_MISMATCH {location}: Nav2 报成功但 gz 真值在 "
                f"({gx:.2f}, {gy:.2f})，离目标 ({x:.2f}, {y:.2f}) 差 {dist:.1f}m"
                f"{'、朝向差 %.2frad' % dyaw if dyaw > 0.6 else ''}（判假到达，重规划）")

    def _gz_cup_pose(self, room: str) -> tuple[float, ...] | None:
        """gz 回读工位杯世界位姿 (x, y, z, R, P, Y)；读不到返回 None（不下结论）。"""
        cup_model = DOCK_POSES[room]["cup_model"]
        try:
            completed = subprocess.run(
                ["bash", "-lc",
                 f"timeout 5 gz model -m {shlex.quote(cup_model)} -p"],
                text=True, capture_output=True, timeout=15)
            parts = completed.stdout.split()
            # gz CLI 高负载会静默返回**残缺输出**（桥接侧同款坑早有 len>=6 防线，
            # 这里曾直接切短 tuple：三连第5趟第1回合，物理上立杯 z=0.457 收工后
            # _cup_upright 取 pose[3] 炸 "tuple index out of range"，工具带伤整判）。
            # 读不满 6 个数＝不可信，一律 None 走"不下结论"语义。
            if len(parts) < 6:
                return None
            return tuple(float(v) for v in parts[:6])
        except Exception:
            return None

    def _cup_upright(self, room: str) -> bool | None:
        """工位杯**回合末**姿态：落在桌面高度带且平放。

        立桌 z≈0.457；倒伏/落地 z≈0.02 且 |R|≈1.5。

        ⚠️ 这只证明"此刻没倒"，**两件事都证明不了**：杯子是否真被夹起过（空抓时
        它也一直待在桌上）、以及是否真被松手（夹在爪里举在臂上同样落在这个高度带）。
        前者由回合内的最大抬升 `cup_lift` 判定，后者由 `_cup_left_at_station`
        在底盘离站后判定。（2026-11-04 run11/run13 两次假成功分别栽在这两点上。）
        """
        pose = self._gz_cup_pose(room)
        if pose is None:
            return None
        _x, _y, z, roll, pitch = pose[0], pose[1], pose[2], pose[3], pose[4]
        # 上界从固定 0.60 收到名义 ±5cm：第11趟 ep2 无松手、杯"搁在指尖上"
        # z=0.515 也被旧带放行（0.40~0.60）。放杯末态理应 ≈ 名义 ±2cm，0.515
        # 这种悬空姿态必须算"没落桌"，交给抬升采样与离站复核继续追罪。
        cz = (DOCK_POSES[room]["cup"][2] if room in DOCK_POSES
              else 0.457)
        return abs(z - cz) <= 0.05 and abs(roll) < 0.6 and abs(pitch) < 0.6

    def _cup_left_at_station(self, room: str) -> bool | None:
        """底盘离开工位后回读杯子：还在工位桌面上=真的放下并松手了。

        夹着杯子跑=杯子的世界坐标跟着底盘走，必然偏离工位名义坐标；磕到地上=
        z 掉出桌面高度带。两种都判 False。读不到返回 None（不下结论）。
        """
        cx, cy, _cz = DOCK_POSES[room]["cup"]
        pose = self._gz_cup_pose(room)
        if pose is None:
            return None
        # 姿态判据（三连第3趟实锤）：倒伏的杯中心 z≈0.47 照样躲过位置带检查，
        # "留在桌面上"复核被糊弄。立桌 |R|,|P|≈0；躺倒 |R|或|P|≈1.5。0.35 rad
        # 是"晃而未倒"与"真倒了"的安全分界。
        upright = (len(pose) < 5
                   or (abs(pose[3]) < 0.35 and abs(pose[4]) < 0.35))
        return (abs(pose[0] - cx) < 0.30 and abs(pose[1] - cy) < 0.30
                and pose[2] > 0.30 and upright)

    def _retreat_from_dock(self, room: str, dist: float = 0.7) -> None:
        """放杯成功后沿 -yaw 直退 dist 米（gz teleport，不旋转），让底盘退出
        原地转向的扫掠圈，避免起步动作把刚放好的杯子拍下桌。尽力而为：
        读回不符只打日志，绝不影响回合结果（后续 snap/relocalize 都是幂等的）。"""
        dock = DOCK_POSES.get(room)
        if not dock or not self.real_navigation:
            return
        bx, by, byaw = dock["base"]
        tx, ty = bx - dist * math.cos(byaw), by - dist * math.sin(byaw)
        cmd = (f"timeout 6 gz model -m base_robot "
               f"-x {tx:.3f} -y {ty:.3f} -z 0 -R 0 -P 0 -Y {byaw}")
        try:
            subprocess.run(["bash", "-lc", cmd], capture_output=True,
                           text=True, timeout=15)
            time.sleep(2.0)  # 物理沉降，防 teleport 弹跳
            pose = self._gz_world_pose("base_robot")
            if pose is None:
                print(f"[Retreat] {room}: 读不到底盘位姿，跳过直退确认")
            else:
                d = math.hypot(pose[0] - tx, pose[1] - ty)
                tag = "OK" if d <= 0.25 else f"未到位（偏差 {d:.2f}m）"
                print(f"[Retreat] {room}: 直退 {dist}m -> ({tx:.2f},{ty:.2f}) {tag}")
        except Exception as exc:
            print(f"[Retreat] {room}: 直退失败（不致命）: {exc}")

    def _snap_dock(self, room: str, settle: float = 5.0) -> None:
        """底盘+杯子 gz teleport 精对齐到 DOCK_POSES[room]，并**回读验证**。

        gz CLI 在 Nav2/多节点负载下会返回 0 但静默不生效（run12 实录：导航后
        传送底盘，8s 后回读仍在 30cm 外的旧位——整套流程捏空的直接原因）。
        只信回读坐标，不中就重试，最多 3 轮。
        """
        dock = DOCK_POSES[room]
        bx, by, byaw = dock["base"]
        cx, cy, cz = dock["cup"]
        cm = dock["cup_model"]
        command = " && ".join([
            f"timeout 6 gz model -m base_robot -x {bx} -y {by} -z 0 -R 0 -P 0 -Y {byaw}",
            f"timeout 6 gz model -m {cm} -x {cx} -y {cy} -z {cz} -R 0 -P 0 -Y 0",
            f"sleep {settle}",
        ])
        last = ""
        for attempt in range(3):
            completed = subprocess.run(["bash", "-lc", command], text=True,
                                       capture_output=True, timeout=40)
            if completed.returncode != 0:
                last = f"rc={completed.returncode} stderr={completed.stderr[-200:]}"
                continue
            base_pose = self._gz_world_pose("base_robot")
            cup_pose = self._gz_cup_pose(room)
            if base_pose is not None and cup_pose is not None:
                bdx, bdy = base_pose[0] - bx, base_pose[1] - by
                bdyaw = math.atan2(math.sin(base_pose[2] - byaw),
                                   math.cos(base_pose[2] - byaw))
                cdist = math.hypot(cup_pose[0] - cx, cup_pose[1] - cy)
                last = (f"base=({base_pose[0]:.3f},{base_pose[1]:.3f},"
                        f"{base_pose[2]:.3f}) cup=({cup_pose[0]:.3f},{cup_pose[1]:.3f})")
                if (math.hypot(bdx, bdy) <= 0.03 and abs(bdyaw) <= 0.08
                        and cdist <= 0.03):
                    return
                print(f"[Dock] 摆位未确认（第 {attempt + 1} 轮）：{last}，重试")
            else:
                last = "回读为空"
                print(f"[Dock] 回读为空（第 {attempt + 1} 轮），重试")
            time.sleep(1.0)
        raise RuntimeError(f"gz teleport 3 轮仍未对齐工位 {room}：{last}")

    def _read_vla_status(self) -> str | None:
        """读一帧 /vla_episode_status。返回 IDLE/WARMING/RUNNING/COMPLETE 或 None。

        注意：字段过滤必须 --field data 且输出是**裸值行**（无 data: 前缀），
        两种格式都要认——只认前缀曾让桥接两次 COMPLETE 被 agent 读成 NONE。
        必须 --no-daemon：ros2 CLI 默认借 ros2 daemon（xmlrpc）查 topic QoS，
        daemon 被杀/半恢复时 echo 抛 !rclpy.ok() 全读 None——2026-11-04 彩排#2
        实录：桥接明明已 Episode start，agent 连读 3 轮判 NO_CONFIRM 假阴性，
        放后续计划在回合进行中导航，连锁翻车。"""
        # 第一优先：桥接的文件心跳（_publish_status 每拍原子重写）。高负载
        # 机器上 ros2 CLI 冷启动整读超时已是常态（加测2趟/续测3、4趟实录），
        # echo 是旁路备份。文件时间戳 >6s＝桥接没在跳拍，视为不可用继续走
        # echo（陈旧文件不能当回执——上一趟的 COMPLETE 会复活成假回执）。
        try:
            with open('/tmp/vla_episode_status.txt') as f:
                word, _, stamp = f.read().strip().partition(' ')
            if (word in ("IDLE", "WARMING", "RUNNING", "COMPLETE")
                    and time.time() - float(stamp) < 6.0):
                return word
        except Exception:
            pass
        # timeout 12 在 gz 满负荷时会把"DDS 发现慢"误杀成读不通（加测第2趟
        # 实录：桥接明明在跑、90s 窗口一次没读到）。给发现留 30s 余量。
        shell = " && ".join([
            f"source {shlex.quote(str(self.ros_setup))}",
            "timeout 30 ros2 topic echo /vla_episode_status --field data --once "
            "--no-daemon 2>/dev/null",
        ])
        try:
            completed = subprocess.run(["bash", "-lc", shell], text=True,
                                       capture_output=True, timeout=45)
        except Exception:
            return None
        last = None
        for line in completed.stdout.splitlines():
            line = line.strip()
            value = None
            if line.startswith("data:"):
                value = line.split(":", 1)[1].strip().strip("'\"")
            elif line in ("IDLE", "WARMING", "RUNNING", "COMPLETE"):
                value = line
            if value in ("IDLE", "WARMING", "RUNNING", "COMPLETE"):
                last = value
        return last

    def _publish_and_confirm(self, instruction: str, window: float = 90.0,
                             attempts: int = 3) -> tuple[str | None, float]:
        """发布起回合指令并**确认桥接真的接住**（看到 WARMING/RUNNING）。

        ros2 topic pub --once 发一条即退：冷启动机器上 FastDDS 发现还没完成
        publisher 就销毁了，样本无声丢失（2026-11-04 实录：桥接整场收不到、
        状态恒 IDLE、agent 白等 424s），所以要有重发。但**只有读回确凿 IDLE
        （桥接活着且就是没收住）才允许重发**——读 None ≠ 未送达：gz 拥挤时
        echo --no-daemon 整读超时，指令其实已接住在跑（2026-11-07 加测第1趟
        实锤：连发 3 条副本在 DDS 队列积压，COMPLETE 同瞬间被处理成"re-trigger
        full reset"，一条指令演成 4 个回合，幽灵非接管回合把立杯扫倒 R=1.49）。
        读不通就安静等窗口，绝不补发；COMPLETE 同样不动（旧重发布值/上一回合
        刚完）。返回 (回执或 None, 耗时秒)。"""
        start = time.monotonic()
        self._publish_ros_string(self.vla_topic, instruction)  # 首条无条件发
        issued = 1
        while time.monotonic() - start < window:
            st = self._read_vla_status()
            if st in ("WARMING", "RUNNING"):
                return st, time.monotonic() - start
            if st == "IDLE":
                if issued >= attempts:
                    return None, time.monotonic() - start
                self._publish_ros_string(self.vla_topic, instruction)
                issued += 1
            time.sleep(2.0)
        return None, time.monotonic() - start

    def _wait_vla_episode(self, timeout: float, seen_run: bool = False,
                          cup_room: str | None = None) -> tuple[str, float, float | None]:
        """轮询 /vla_episode_status（桥接 1Hz 周期发布）直到 COMPLETE 或超时。

        防陈旧 COMPLETE：必须看到本回合的 WARMING/RUNNING 之后的 COMPLETE 才算数
        （seen_run=True 表示调用方已确认过本回合起手，可直接接受 COMPLETE）。

        同时按拍采样工位杯 z，返回**回合内最大抬升**（米）。抬升是"真夹起来了"
        唯一的硬证据：COMPLETE 只代表 GT 帧放完——run13 实测模型横向偏 4cm 捏了个
        空，插件 0 次附着事件，倾倒+放回整套动作照样跑完、杯子纹丝不动。
        """
        start = time.monotonic()
        last = "NONE"
        lift: float | None = None
        nominal_z = DOCK_POSES[cup_room]["cup"][2] if cup_room in DOCK_POSES else None
        while time.monotonic() - start < timeout:
            st = self._read_vla_status()
            if st:
                last = st
            if st in ("WARMING", "RUNNING"):
                seen_run = True
            if nominal_z is not None:
                pose = self._gz_cup_pose(cup_room)
                if pose is not None:
                    dz = pose[2] - nominal_z
                    lift = dz if lift is None else max(lift, dz)
            if last == "COMPLETE" and seen_run:
                return last, time.monotonic() - start, lift
            time.sleep(2.0)
        return last, time.monotonic() - start, lift

    def verify_navigation(self, location: str) -> ToolResult:
        # Replace with Nav2 result + TF/AMCL pose threshold check later.
        return ToolResult(
            tool="verify_navigation",
            success=True,
            message=f"Mock navigation verification passed for {location}",
            observations={"location": location, "distance_to_goal": 0.0, "mode": "mock"},
        )

    def verify_grasp(self, object_name: str | None = None) -> ToolResult:
        """抓取验证：真实模式改用上一段回合的**物理证据**（杯子抬升），不是恒真。

        旧 mock 版把 `attached: True` 无条件写进验证记录，空抓也算"抓取验证通过"，
        正是 run13 假成功链条上的一环（2026-11-04 实录）。
        """
        ev = self._last_grasp_evidence
        if self.mock or ev is None:
            return ToolResult(
                tool="verify_grasp",
                success=True,
                message=f"Mock grasp verification passed{f' for {object_name}' if object_name else ''}",
                observations={"object_name": object_name, "attached": True, "mode": "mock"},
            )
        grasped, lift, status = ev.get("grasped"), ev.get("lift_m"), ev.get("final_status")
        obs = {"object_name": object_name, "mode": "real", "evidence": ev,
               "attached": bool(grasped)}
        if grasped is True:
            return ToolResult(tool="verify_grasp", success=True,
                              message=f"抓取验证通过：杯子被抬起 {lift * 100:.1f}cm"
                                      f"（阈值 {self.grasp_lift_min * 100:.0f}cm）",
                              observations=obs)
        if grasped is None:
            return ToolResult(tool="verify_grasp", success=True,
                              message="抓取验证未做物理复核（该工位无 DOCK_POSES 标定）",
                              observations=obs)
        return ToolResult(tool="verify_grasp", success=False,
                          message=f"抓取验证失败：回合状态 {status}，杯子最大只动 "
                                  f"{'读不到' if lift is None else f'{lift * 100:.1f}cm'}＝空抓",
                          observations=obs)

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

    def _publish_initialpose(self, x: float, y: float, yaw: float) -> None:
        """向 /initialpose 发 map 系 GT 位姿，强制 AMCL 重定位。

        必须 rclpy 子进程而非 ros2 topic pub CLI：CLI 发完即退，AMCL 订阅者
        可能还没被 FastDDS 发现，一次性样本整条丢掉（本机 2026-11-04 实录）。
        这里先等 subscription_count>0 再连发 3 帧。
        """
        if not self.ros_setup.exists():
            raise FileNotFoundError(f"ROS setup file not found: {self.ros_setup}")
        py = r'''
import sys, time, math, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = (float(a) for a in sys.argv[1:4])
rclpy.init()
# 必须 use_sim_time：AMCL 跑在 /clock 时基上，wall-clock 时间戳（~1.7e9）
# 会被当成"未来"直接丢掉，initialpose 看似发了（订阅者已发现）实际全数无效
# ——2026-11-04 实录：GT 重定位连发数轮，nav.log 一条 initialPoseReceived 都没有。
node = Node("agent_initialpose",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
for _ in range(100):                      # 最多等 10s 让 AMCL 订阅被发现
    if pub.get_subscription_count() > 0:
        break
    time.sleep(0.1)
else:
    print("INITIALPOSE_NO_SUBSCRIBER"); sys.exit(1)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
for _ in range(20):                       # 连发 ~4s，盖住 AMCL 的收粒窗口
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.2)
print("INITIALPOSE_SENT stamp=%d" % msg.header.stamp.sec)
'''
        command = " && ".join([
            f"source {shlex.quote(str(self.ros_setup))}",
            "python3 - "
            f"{float(x)} {float(y)} {float(yaw)} <<'PYEOF'\n{py}PYEOF",
        ])
        completed = subprocess.run(["bash", "-lc", command], text=True,
                                   capture_output=True, timeout=40)
        if completed.returncode != 0:
            raise RuntimeError(
                f"initialpose 发布失败 rc={completed.returncode} "
                f"out={completed.stdout[-200:]} err={completed.stderr[-200:]}")

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
        """阻塞调用 Nav2 NavigateToPose 并等待到达。

        必须用 rclpy ActionClient 子进程（nav_waypoints.py 同款、V1.5.0 实测路径）：
        `ros2 action send_goal` CLI 起 send_goal 后收不到 goal response
        （FastDDS 下 CLI 老毛病，bt_navigator 日志 "Failed to send goal
        response ... client will not receive response"），300s 干等超时、
        机器人一步不动——2026-11-02 首轮联调实录。
        timeout 与 nav_waypoints.py 一致（工厂跨房间路径长，120s 偏紧）。
        """
        if not self.ros_setup.exists():
            raise FileNotFoundError(f"ROS setup file not found: {self.ros_setup}")

        py = r'''
import sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
x, y, yaw, tout = (float(a) for a in sys.argv[1:5])
import math
rclpy.init(); node = Node("agent_nav")
ac = ActionClient(node, NavigateToPose, "/navigate_to_pose")
if not ac.wait_for_server(15.0):
    print("NO_ACTION_SERVER"); sys.exit(2)
# 2026-11-04 08:40 跑实录：VLA 回合后回家 goal 判 ABORTED(status=6)，
# nav.log "GridBased: failed to create plan ... to (0.00, 0.00)"——操作期间
# 相机点云把桌面/周边刷进 live costmap，开局摆在原点的杯堆也把 home 目标格
# 刷成障碍。每次导航前清一次 costmap（只清观测层，静态地图保留）；清不掉
# 也照常发 goal，不引入新失败面。
from nav2_msgs.srv import ClearEntireCostmap  # Humble 名（rolling 才叫 ClearCostmapEntirely）
for svc in ("/global_costmap/clear_entirely_global_costmap",
            "/local_costmap/clear_entirely_local_costmap"):
    try:
        c = node.create_client(ClearEntireCostmap, svc)
        if c.wait_for_service(timeout_sec=3.0):
            f = c.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        else:
            print("COSTMAP_CLEAR_SKIP %s" % svc)
    except Exception as exc:
        print("COSTMAP_CLEAR_SKIP %s: %s" % (svc, exc))
ps = PoseStamped()
ps.header.frame_id = "map"
ps.pose.position.x, ps.pose.position.y = x, y
qz = math.sin(yaw / 2.0); qw = math.cos(yaw / 2.0)
ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
goal = NavigateToPose.Goal()
goal.pose = ps
# 导航栈刚起时 bt_navigator 偶发 goal response 丢失（2026-11-04 实录：前两发
# 全 "Failed to send goal response"、手测第三发即成功）。丢响应≠被拒：重发。
g = None
for attempt in range(4):
    gh = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh, timeout_sec=30.0)
    g = gh.result()
    if g is not None and g.accepted:
        break
    print("GOAL_SEND_RETRY %d: %s" % (attempt + 1,
        "response-lost" if g is None else "rejected"))
    time.sleep(3.0)
if g is None or not g.accepted:
    print("GOAL_REJECTED_OR_LOST"); sys.exit(3)
fh = g.get_result_async()
rclpy.spin_until_future_complete(node, fh, timeout_sec=tout)
fr = fh.result()
if fr is None:
    print("NAV_WAIT_TIMEOUT"); sys.exit(4)
print("NAV_DONE status=%d" % fr.status)
# 4=SUCCEEDED（Humble 的结果类没有 SUCCESS 常量，用字面值，见 action 接口定义）
sys.exit(0 if fr.status == 4 else 5)
'''
        command = " && ".join([
            f"source {shlex.quote(str(self.ros_setup))}",
            "python3 - "
            f"{float(x)} {float(y)} {float(yaw)} {float(timeout)} <<'PYEOF'\n{py}PYEOF",
        ])
        completed = subprocess.run(["bash", "-lc", command], text=True,
                                   capture_output=True, timeout=timeout + 120)
        if completed.returncode != 0:
            raise RuntimeError(
                f"Nav2 失败 rc={completed.returncode} "
                f"out={completed.stdout.strip()[-300:]} err={completed.stderr.strip()[-300:]}")
