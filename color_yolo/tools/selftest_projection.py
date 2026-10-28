#!/usr/bin/env python3
"""阶段 1 的离线自检：不接仿真、不接 ROS，用 numpy 验证投影链路和几何结论。

覆盖三件事：
  1. 坐标系约定 —— 用 camera.urdf.xacro 里 optical joint 的 rpy(-pi/2,0,-pi/2) 推出来的
     前/右/下轴，必须与 ENU 世界 +X 前向一致；再用一个手算可验证的合成相机校投影公式。
  2. 真值链路 —— model_pose ∘ link_pose 得到的牌子世界坐标，与停靠位的相对关系是否自洽
     （预期：四个停靠位都在牌子正前方约 1.20m、横向偏移近零）。
  3. 可见度 —— 在真实停靠位上，牌子在画面里的位置/尺寸/可见比例，量化画面上下边界。

用法：python3 color_yolo/tools/selftest_projection.py
只依赖系统 python3 的 numpy + pyyaml（ROS 无关）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import (  # noqa: E402
    Marker,
    bbox_from_pixels,
    gazebo_pose_to_matrix,
    half_vfov_deg,
    intrinsics_from,
    pose_to_matrix,
    project_to_pixels,
    transform_points,
    vertical_angle_deg,
    visible_fraction,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "markers.yaml"


def load_config() -> dict:
    with CONFIG.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def camera_pose_at_station(station, camera: dict) -> np.ndarray:
    """停靠位 (x, y, yaw) -> front_camera_optical_link 的世界位姿。

    链条：base_link -> (mounting xyz/rpy) -> front_camera_link -> 光学位移 rpy(-pi/2,0,-pi/2)
    """
    x, y, yaw = station
    lift = float(camera.get("footprint_to_base_link_z", 0.0))
    base = pose_to_matrix((x, y, lift), (0.0, 0.0, yaw))
    mount = pose_to_matrix(camera["mounting_xyz"], camera["mounting_rpy"])
    optical = pose_to_matrix((0.0, 0.0, 0.0), (-np.pi / 2, 0.0, -np.pi / 2))
    return base @ mount @ optical


def check_frame_convention(camera: dict) -> None:
    optical = pose_to_matrix((0.0, 0.0, 0.0), (-np.pi / 2, 0.0, -np.pi / 2))
    right, down, forward = optical[:3, 0], optical[:3, 1], optical[:3, 2]
    assert np.allclose(forward, (1.0, 0.0, 0.0), atol=1e-9), forward
    assert np.allclose(right, (0.0, -1.0, 0.0), atol=1e-9), right
    assert np.allclose(down, (0.0, 0.0, -1.0), atol=1e-9), down
    print("[ok] optical joint rpy(-pi/2,0,-pi/2) 在 ENU 下给出 前=+X 右=-Y 下=-Z")

    k = intrinsics_from(camera["horizontal_fov"], camera["width"], camera["height"])
    cam = pose_to_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) @ optical
    # 三个手算可验证的点：光轴正中 / 右前 45° / 正前上方 45°
    # depth 是相机系 z（前向=世界 +X），不是欧氏距离，所以三点都是 1.0
    uv, depth = project_to_pixels(cam, k, [(1.0, 0.0, 0.0), (1.0, -1.0, 0.0), (1.0, 0.0, 1.0)])
    fx, cx, cy = k[0, 0], k[0, 2], k[1, 2]
    assert np.allclose(depth, [1.0, 1.0, 1.0], atol=1e-9), depth
    assert np.allclose(uv[0], [cx, cy], atol=1e-9), uv[0]
    assert np.allclose(uv[1], [cx + fx, cy], atol=1e-9), uv[1]
    assert np.allclose(uv[2], [cx, cy - fx], atol=1e-9), uv[2]
    print("[ok] 投影公式：光轴点居中，世界 -Y 落在右侧 (+u)，世界上方落在上半 (v 减小)")


def check_station_geometry(cfg: dict) -> None:
    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])
    width, height = cfg["camera"]["width"], cfg["camera"]["height"]
    print("\n停靠位 -> 牌子 的相对几何（世界系，含 model 偏移）：")
    for marker_cfg in cfg["markers"]:
        marker = Marker(name=marker_cfg["name"], room=marker_cfg["room"],
                        class_name=marker_cfg["class_name"], link_pose=marker_cfg["pose"],
                        size=marker_cfg["size"])
        center = marker.pose_world(model)[:3, 3]
        station = cfg["dock_stations"][marker.room]
        delta = center - np.array([station[0], station[1], 0.0])
        forward = np.array([np.cos(station[2]), np.sin(station[2]), 0.0])
        lateral = np.array([-np.sin(station[2]), np.cos(station[2]), 0.0])
        print(f"  {marker.room:<6}{marker.class_name:<7} 牌子世界=({center[0]:8.3f},{center[1]:7.3f},{center[2]:.2f}) "
              f"牌子在停靠位前方 {forward @ delta:6.3f}m  横向偏移 {lateral @ delta:+7.4f}m")


def report_visibility(cfg: dict) -> None:
    camera = cfg["camera"]
    width, height = camera["width"], camera["height"]
    k = intrinsics_from(camera["horizontal_fov"], width, height)
    vfov = half_vfov_deg(k, height)
    print(f"\n内参（由 horizontal_fov 推）：fx=fy={k[0, 0]:.1f}px cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}")
    print(f"垂直半视场 half_vfov = {vfov:.2f}°（画面上边缘=+{vfov:.2f}°，下边缘=-{vfov:.2f}°）")

    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])
    station = cfg["dock_stations"]["room1"]
    cam_z = camera_pose_at_station(station, camera)[2, 3]
    print(f"\n真实停靠位上的可见度（相机离地 {cam_z:.3f}m）：")
    for marker_cfg in cfg["markers"]:
        marker = Marker(name=marker_cfg["name"], room=marker_cfg["room"],
                        class_name=marker_cfg["class_name"], link_pose=marker_cfg["pose"],
                        size=marker_cfg["size"])
        cam = camera_pose_at_station(cfg["dock_stations"][marker.room], camera)
        corners = marker.corners_world(model)
        uv, depth = project_to_pixels(cam, k, corners)
        box = bbox_from_pixels(uv, depth, width, height)
        angle_center = vertical_angle_deg(cam, marker.pose_world(model)[:3, 3])
        frac = visible_fraction(cam, k, marker.face_samples_world(model), width, height)
        dist = float(np.linalg.norm(cam[:3, 3] - marker.pose_world(model)[:3, 3]))
        size_txt = "无（完全在画外）" if box is None else (
            f"{box['bbox'][2]-box['bbox'][0]:.0f}x{box['bbox'][3]-box['bbox'][1]:.0f}px"
            f"{' 被边缘切断' if box['clipped'] else ''}")
        print(f"  {marker.room:<6}{marker.class_name:<7} 距离 {dist:5.2f}m  中心仰角 {angle_center:+6.1f}°  "
              f"可见比例 {frac*100:5.1f}%  框={size_txt}")


def check_sweep_window(cfg: dict) -> None:
    """扫出"牌子至少露一点"和"整块进画"的距离窗口，供采集网格使用。"""
    camera = cfg["camera"]
    width, height = camera["width"], camera["height"]
    k = intrinsics_from(camera["horizontal_fov"], width, height)
    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])
    marker = Marker(name=cfg["markers"][0]["name"], room=cfg["markers"][0]["room"],
                    class_name=cfg["markers"][0]["class_name"],
                    link_pose=cfg["markers"][0]["pose"], size=cfg["markers"][0]["size"])
    center = marker.pose_world(model)[:3, 3]
    print("\n沿牌子法线方向后退，可见度随距离的变化：")
    print("  相机->牌子 距离   中心仰角   可见比例   框高(px)")
    for dist in (0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.4, 2.8, 3.2):
        # 西墙牌子朝 +X 展示：机器人停在它东侧、朝 -X 看（yaw=pi）；东墙反之。
        if center[0] < 0:
            station = (center[0] + dist, center[1], np.pi)
        else:
            station = (center[0] - dist, center[1], 0.0)
        cam = camera_pose_at_station(station, camera)
        uv, depth = project_to_pixels(cam, k, marker.corners_world(model))
        box = bbox_from_pixels(uv, depth, width, height)
        frac = visible_fraction(cam, k, marker.face_samples_world(model), width, height)
        hpx = 0 if box is None else int(box["bbox"][3] - box["bbox"][1])
        gap = dist - 0.22  # 相机在 base_link 前 0.22m
        print(f"   {gap:6.2f}m          {vertical_angle_deg(cam, center):+7.1f}°   {frac*100:7.1f}%   {hpx:5d}")


def main() -> int:
    cfg = load_config()
    check_frame_convention(cfg["camera"])
    check_station_geometry(cfg)
    report_visibility(cfg)
    check_sweep_window(cfg)
    print("\n自检结束。若上面角度/框与 hand-check 一致，投影链路可用于自动标注。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
