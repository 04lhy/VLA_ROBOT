#!/usr/bin/env python3
"""巡检停靠位周围的位姿扫描采集：让 Nav2 走一组位姿，每个位姿调 capture_frames.py 存帧。

自动标注只依赖"相机世界位姿 + 牌子几何真值"，所以数据集不需要人工标注，
只需要位姿覆盖到位误差的包络 —— 本脚本就是负责覆盖这个包络。

先 dry-run 看计划（不需要 ROS、不需要仿真）：
  python3 color_yolo/tools/sweep_stations.py --dry-run
真跑（需要仿真 + Nav2 已启动）：
  source /opt/ros/humble/setup.bash && source base_robot_ws/install/setup.bash
  python3 color_yolo/tools/sweep_stations.py
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import (  # noqa: E402
    Marker,
    gazebo_pose_to_matrix,
    intrinsics_from,
    matrix_to_quaternion,
    pose_to_matrix,
    visible_fraction,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "markers.yaml"
DEFAULT_CAPTURE = Path(__file__).resolve().parent / "capture_frames.py"


def load_config() -> dict:
    with CONFIG.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def plan_for_room(cfg: dict, room: str, min_visible: float, steps: dict,
                  standoff: float = 0.0) -> list[dict]:
    """以真实停靠位为锚，沿"停靠位->牌子"连线后退/横移，生成一组可见的位姿。

    standoff>0 时忽略 steps["back"]，直接以"机器人到牌子的水平距离"为目标取三档，
    这样"后退到 2.4m 采集"就是一个参数而不是手算偏移量。
    """
    camera = cfg["camera"]
    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])
    k = intrinsics_from(camera["horizontal_fov"], camera["width"], camera["height"])
    marker = next(m for m in cfg["markers"] if m["room"] == room)
    marker_obj = Marker(name=marker["name"], room=room, class_name=marker["class_name"],
                        link_pose=marker["pose"], size=marker["size"])
    center = marker_obj.pose_world(model)[:3, 3]
    samples = marker_obj.face_samples_world(model)

    station = cfg["dock_stations"][room]
    anchor = np.array([station[0], station[1]], dtype=float)
    toward = center[:2] - anchor
    distance = float(np.linalg.norm(toward))
    if distance < 1e-6:
        raise RuntimeError(f"{room} 的停靠位与牌子重合，无法建立接近方向")
    axis = toward / distance
    side = np.array([-axis[1], axis[0]], dtype=float)

    poses: list[dict] = []
    backs = ([round(standoff - distance + delta, 2) for delta in (-0.25, 0.0, 0.25)]
             if standoff > 0 else list(steps["back"]))
    for back in backs:
        for lateral in steps["lateral"]:
            for yaw_jitter in steps["yaw_jitter"]:
                position = center[:2] - (distance + back) * axis + lateral * side
                heading = math.atan2(center[1] - position[1], center[0] - position[0]) + yaw_jitter
                station_pose = (float(position[0]), float(position[1]), float(heading))
                cam = camera_pose_at(station_pose, camera)
                fraction = visible_fraction(cam, k, samples, camera["width"], camera["height"])
                poses.append({
                    "room": room,
                    "class_name": marker["class_name"],
                    "pose": station_pose,
                    "back": back,
                    "lateral": lateral,
                    "yaw_jitter": yaw_jitter,
                    "visible_fraction": round(fraction, 3),
                    "keep": fraction >= min_visible,
                })
    return poses


def default_steps() -> dict:
    # 一个房间 7x3x2=42 个位姿、每房约 170 帧，4 房约 670 帧：
    # 对纯色平面板的 4 类检测够训练了，再密主要是 Nav2 走路的时间成本。
    return {"back": [0.2, 0.5, 0.8, 1.1, 1.4, 1.7, 2.0],
            "lateral": [-0.4, 0.0, 0.4],
            "yaw_jitter": [-0.1, 0.1]}


def subsample(poses: list[dict], limit: int) -> list[dict]:
    if limit <= 0 or len(poses) <= limit:
        return poses
    picks = np.linspace(0, len(poses) - 1, limit).round().astype(int)
    return [poses[i] for i in sorted(set(picks.tolist()))]


def camera_pose_from_footprint(footprint, camera: dict) -> np.ndarray:
    """base_footprint 位姿 (x,y,z,roll,pitch,yaw) -> 相机光学位姿。

    链条必须显式包含 base_footprint -> base_link 的 0.091m：
    gz model -p 报的是 footprint，而传感器挂载偏移定义在 base_link 系下，
    漏掉这一段会让相机矮 9cm，在 2m 距离上就是约 20px 的系统偏差。
    """
    x, y, z, roll, pitch, yaw = footprint
    lift = float(camera.get("footprint_to_base_link_z", 0.0))
    footprint_pose = pose_to_matrix((x, y, z), (roll, pitch, yaw))
    base = footprint_pose @ pose_to_matrix((0.0, 0.0, lift), (0.0, 0.0, 0.0))
    mount = pose_to_matrix(camera["mounting_xyz"], camera["mounting_rpy"])
    optical = pose_to_matrix((0.0, 0.0, 0.0), (-np.pi / 2, 0.0, -np.pi / 2))
    return base @ mount @ optical


def camera_pose_at(station, camera: dict) -> np.ndarray:
    """规划用：停靠位 (x, y, yaw)，footprint 贴地。"""
    x, y, yaw = station
    return camera_pose_from_footprint((x, y, 0.0, 0.0, 0.0, yaw), camera)


def gz_pose(model: str) -> tuple[float, ...] | None:
    """读机器人在 Gazebo 里的**实际** base_footprint 位姿 (x y z roll pitch yaw)。

    teleport 指令落地后车体仍会有零点几度的 pitch/yaw 残差，用实测值当真值更准。
    """
    completed = subprocess.run(["gz", "model", "-m", model, "-p"],
                               capture_output=True, text=True, timeout=15.0, check=False)
    fields = completed.stdout.replace(",", " ").split()
    if completed.returncode != 0 or len(fields) < 6:
        return None
    return tuple(float(v) for v in fields[:6])


def camera_pose_json(plan: dict, cfg: dict, measured=None) -> str:
    """相机光学位姿 JSON，作为标注真值传给采图节点。

    位姿来自指令或 Gazebo 实测，比读 AMCL/里程计的 TF 更准
    （把定位误差写进"真值"会教出个错的检测器）。
    """
    camera = cfg["camera"]
    if measured is not None:
        camera_pose = camera_pose_from_footprint(measured, camera)
    else:
        camera_pose = camera_pose_at(plan["pose"], camera)
    return json.dumps({
        "position": [round(float(v), 5) for v in camera_pose[:3, 3]],
        "quaternion": [round(float(v), 6) for v in matrix_to_quaternion(camera_pose)],
        "source": "gz_measured" if measured is not None else "commanded_teleport",
    }, ensure_ascii=False)


def gz_teleport(model: str, x: float, y: float, z: float, yaw: float) -> None:
    subprocess.run(["gz", "model", "-m", model,
                    "-x", f"{x}", "-y", f"{y}", "-z", f"{z}", "-Y", f"{yaw}"],
                   capture_output=True, text=True, timeout=15.0, check=False)


def run_capture(cfg: dict, plan: dict, args: argparse.Namespace, index: int) -> int:
    meta = {
        "source": "sweep",
        "mode": args.mode,
        "room": plan["room"],
        "requested_pose": list(plan["pose"]),
        "back": plan["back"],
        "lateral": plan["lateral"],
        "yaw_jitter": plan["yaw_jitter"],
        "predicted_visible_fraction": plan["visible_fraction"],
    }
    command = [
        sys.executable, str(args.capture),
        "--out", str(args.out),
        "--tag", f"{plan['room']}_{index:03d}",
        "--count", str(args.frames),
        "--root-frame", args.root_frame,
        "--optical-frame", cfg["camera"]["optical_frame"],
        "--image-topic", cfg["camera"]["image_raw_topic"],
        "--info-topic", cfg["camera"]["camera_info_topic"],
        "--meta-json", json.dumps(meta, ensure_ascii=False),
    ]
    if args.mode == "teleport":
        command += ["--pose-source", "explicit", "--camera-pose",
                    camera_pose_json(plan, cfg, getattr(args, "measured", None))]
    completed = subprocess.run(command, text=True)
    if completed.returncode != 0:
        print(f"  采图失败 rc={completed.returncode}", file=sys.stderr)
    return completed.returncode


def run_teleport_mode(cfg: dict, plans: dict[str, list[dict]], args: argparse.Namespace) -> int:
    index, failures = 0, 0
    for room, items in plans.items():
        for plan in items:
            x, y, yaw = plan["pose"]
            print(f"== {room} #{index:03d} teleport -> ({x:.2f},{y:.2f},{yaw:.2f}) "
                  f"预测可见 {plan['visible_fraction'] * 100:.0f}%", flush=True)
            gz_teleport(args.robot_model, x, y, args.robot_z, yaw)
            time.sleep(args.settle)
            args.measured = None
            if args.pose_mode == "measured":
                args.measured = gz_pose(args.robot_model)
                if args.measured is None:
                    print("  读不到实测位姿，退回用指令位姿", file=sys.stderr)
            if run_capture(cfg, plan, args, index) != 0:
                failures += 1
            index += 1
    print(f"teleport 扫描结束：位姿 {index} 个，失败 {failures} 个")
    return 0 if index and failures == 0 else 1


def navigate_and_capture(cfg: dict, plans: dict[str, list[dict]], args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from geometry_msgs.msg import Quaternion, PoseStamped
    from nav2_msgs.action import NavigateToPose

    class Sweeper(Node):
        def __init__(self):
            super().__init__("color_yolo_sweep")
            self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        def goto(self, x: float, y: float, yaw: float) -> bool:
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = args.root_frame
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f"目标 ({x:.2f}, {y:.2f}) 被拒绝")
                return False
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result, timeout_sec=args.nav_timeout)
            if result.result() is None:
                self.get_logger().warn(f"目标 ({x:.2f}, {y:.2f}) 导航超时")
                return False
            return True

        def settle(self, seconds: float) -> None:
            end = self.get_clock().now() + Duration(seconds=seconds)
            while rclpy.ok() and self.get_clock().now() < end:
                rclpy.spin_once(self, timeout_sec=0.1)

    rclpy.init()
    node = Sweeper()
    index, failures = 0, 0
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            print("Nav2 navigate_to_pose 不可用，确认 navigation.launch.py 已启动", file=sys.stderr)
            return 2
        for room, plans_for_room in plans.items():
            for plan in plans_for_room:
                x, y, yaw = plan["pose"]
                print(f"== {room} #{index:03d} pose=({x:.2f},{y:.2f},{yaw:.2f}) "
                      f"预测可见 {plan['visible_fraction'] * 100:.0f}%")
                if not node.goto(x, y, yaw):
                    failures += 1
                    continue
                node.settle(args.settle)
                if run_capture(cfg, plan, args, index) != 0:
                    failures += 1
                index += 1
        print(f"扫描结束：计划位姿 {index} 个，失败 {failures} 个")
        return 0 if failures == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="指示牌前的位姿扫描采集（阶段 1）")
    parser.add_argument("--rooms", nargs="*", default=None, help="只扫指定房间，默认全部 4 个")
    parser.add_argument("--mode", choices=("teleport", "nav2"), default="teleport",
                        help="teleport：gz model 直接摆位，快且位姿是已知量（默认，只需 gzserver）；"
                             "nav2：真的开导航走过去，位姿含定位误差，适合后期补真实分布")
    parser.add_argument("--robot-model", default="base_robot", help="Gazebo 里的机器人模型名")
    parser.add_argument("--robot-z", type=float, default=0.0, help="teleport 时 base_footprint 的高度")
    parser.add_argument("--pose-mode", choices=("measured", "commanded"), default="measured",
                        help="measured：teleport 后用 gz model -p 读实际位姿当真值（推荐）；"
                             "commanded：直接用指令位姿，会带车体姿态残差")
    parser.add_argument("--standoff", type=float, default=0.0,
                        help=">0 时直接指定机器人到牌子的水平距离（米），在该值 ±0.25m 取三档")
    parser.add_argument("--min-visible", type=float, default=0.05,
                        help="保留可见比例不低于该值的位姿（牌子完全出画时采集等于采空白）")
    parser.add_argument("--max-per-room", type=int, default=42,
                        help="每个房间最多走多少个位姿，均匀抽样；<=0 表示全用")
    parser.add_argument("--frames", type=int, default=4, help="每个位姿存几帧")
    parser.add_argument("--settle", type=float, default=2.0, help="到位后等待秒数（等 TF 收敛、相机出流）")
    parser.add_argument("--nav-timeout", type=float, default=300.0)
    parser.add_argument("--out", default=str(ROOT / "raw" / "frames"))
    parser.add_argument("--root-frame", default="map")
    parser.add_argument("--capture", default=str(DEFAULT_CAPTURE))
    parser.add_argument("--dry-run", action="store_true", help="只打印位姿计划，不动机器人")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config()
    rooms = args.rooms or [m["room"] for m in cfg["markers"]]
    steps = default_steps()

    plans: dict[str, list[dict]] = {}
    for room in rooms:
        candidates = plan_for_room(cfg, room, args.min_visible, steps, args.standoff)
        kept = [p for p in candidates if p["keep"]]
        dropped = len(candidates) - len(kept)
        if not kept:
            print(f"!! {room} 没有任何可见位姿，检查 markers.yaml 与相机参数是否同步", file=sys.stderr)
            return 2
        plans[room] = subsample(kept, args.max_per_room)
        fracs = [p["visible_fraction"] for p in plans[room]]
        print(f"{room}: 采用 {len(plans[room])} 个位姿（完全出画已丢 {dropped} 个，"
              f"候选共 {len(candidates)}），可见比例 {min(fracs) * 100:.0f}%~{max(fracs) * 100:.0f}%")

    if args.dry_run:
        for room, items in plans.items():
            for plan in items:
                x, y, yaw = plan["pose"]
                print(f"  {room} back={plan['back']:+.2f} lat={plan['lateral']:+.2f} "
                      f"yaw_j={plan['yaw_jitter']:+.2f} -> ({x:8.3f},{y:8.3f},{yaw:6.3f}) "
                      f"可见 {plan['visible_fraction'] * 100:5.1f}%")
        print("\ndry-run：未发送任何导航目标。去掉 --dry-run 即开始真采。")
        return 0

    args.capture = Path(args.capture)
    args.out = Path(args.out)
    if args.mode == "teleport":
        return run_teleport_mode(cfg, plans, args)
    return navigate_and_capture(cfg, plans, args)


if __name__ == "__main__":
    raise SystemExit(main())
