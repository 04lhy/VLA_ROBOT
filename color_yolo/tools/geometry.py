#!/usr/bin/env python3
"""世界坐标 -> 相机像素的投影数学（纯 numpy，不依赖 ROS，可离线单测）。

约定：
  * Gazebo/ROS 世界帧为 ENU：+X 前(北)、+Y 左(西)、+Z 上；rpy 为 extrinsic Rz(yaw)Ry(pitch)Rx(roll)。
  * 相机位姿一律用 **camera_pose_world**（4x4 齐次矩阵，camera 原点在世界里的位置 + 朝向），
    与 `project_to_pixels` 里的求逆一起构成 world -> camera 的变换。
  * 相机光心系 x 右、y 下、z 前：由 front_camera_optical_link 的 TF 给出，
    与 libgazebo_ros_camera 发布的 CameraInfo.K 同一坐标系，因此不需要额外的轴置换。
  * world 文件里 link 的 <pose> 嵌套在 model 的 <pose> 之下，真值位姿 = model_pose ∘ link_pose。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """归一化后转旋转矩阵，兼容 TF 与 Gazebo 给的单位四元数。"""
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_matrix(position: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_rotation(*rpy)
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def gazebo_pose_to_matrix(pose: Sequence[float]) -> np.ndarray:
    """world/xacro 里的 `<pose>x y z roll pitch yaw</pose>`（6 元组）-> 4x4 齐次矩阵。"""
    pose = list(pose)
    if len(pose) != 6:
        raise ValueError(f"Gazebo <pose> 需要 6 个数 (x y z roll pitch yaw)，收到 {pose}")
    return pose_to_matrix(pose[:3], pose[3:6])


def quaternion_pose_to_matrix(position: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    """由 TF/Gazebo 给的 position + quaternion(x,y,z,w) 构造 4x4 位姿矩阵。"""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation(*quaternion)
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def transform_points(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    """按 pose 变换点集：p' = R @ p + t。"""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ pose[:3, :3].T + pose[:3, 3]


def project_to_pixels(camera_pose_world: np.ndarray, k: Sequence[float], points_world: np.ndarray):
    """世界点 -> 像素。返回 uv[N,2] 与相机系深度 z[N]（z<=0 即在相机背后）。"""
    k = np.asarray(k, dtype=float).reshape(3, 3)
    in_cam = transform_points(np.linalg.inv(camera_pose_world), points_world)
    depth = in_cam[:, 2]
    safe = np.where(np.abs(depth) < 1e-9, np.nan, depth)
    uv = (in_cam @ k.T)[:, :2] / safe[:, None]
    return uv, depth


def vertical_angle_deg(camera_pose_world: np.ndarray, point_world: Sequence[float]) -> float:
    """某点相对光轴的仰角（度）。正=画面上半，负=下半；与 half_vfov_deg 同号比较即可判进出画。"""
    in_cam = transform_points(np.linalg.inv(camera_pose_world), point_world)[0]
    forward = in_cam[2]
    if forward <= 1e-9:
        return float("nan")
    return float(np.degrees(np.arctan2(-in_cam[1], forward)))


def half_vfov_deg(k: Sequence[float], height: int) -> float:
    k = np.asarray(k, dtype=float).reshape(3, 3)
    return float(np.degrees(np.arctan(height / 2.0 / k[1, 1])))


@dataclass(frozen=True)
class Marker:
    name: str
    room: str
    class_name: str
    link_pose: Sequence[float]  # world 文件的 <pose>：x y z roll pitch yaw（相对 model）
    size: Sequence[float]       # box <size>：x（宽）、y（板厚）、z（高）

    @property
    def local_pose(self) -> np.ndarray:
        return gazebo_pose_to_matrix(self.link_pose)

    def pose_world(self, model_pose: np.ndarray) -> np.ndarray:
        return model_pose @ self.local_pose

    def corners_world(self, model_pose: np.ndarray) -> np.ndarray:
        extent = np.asarray(self.size, dtype=float) / 2.0
        sign = np.array(
            [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)],
            dtype=float,
        )
        return transform_points(self.pose_world(model_pose), sign * extent)

    def face_samples_world(self, model_pose: np.ndarray, steps: int = 25) -> np.ndarray:
        """在牌子两个大面上撒网格点，用于统计真实可见比例（比 AABB 裁剪准）。"""
        extent = np.asarray(self.size, dtype=float) / 2.0
        axis_u, axis_v = np.meshgrid(
            np.linspace(-extent[0], extent[0], steps),
            np.linspace(-extent[2], extent[2], steps),
        )
        sheets = []
        for side in (-1.0, 1.0):
            sheet = np.column_stack(
                [axis_u.ravel(), np.full(axis_u.size, side * extent[1]), axis_v.ravel()]
            )
            sheets.append(sheet)
        return transform_points(self.pose_world(model_pose), np.vstack(sheets))


def bbox_from_pixels(uv: np.ndarray, depth: np.ndarray, width: int, height: int):
    """8 角点包围盒裁剪到画内；None 表示完全不可见。

    YOLO 用裁剪后的框训练（合成场景里牌子被画面边缘切断是常态），
    因此另配 visible_fraction 一起筛样本，避免标签指向一两个像素。
    """
    in_front = depth > 1e-6
    if not np.any(in_front):
        return None
    visible = uv[in_front]
    x_min, x_max = float(visible[:, 0].min()), float(visible[:, 0].max())
    y_min, y_max = float(visible[:, 1].min()), float(visible[:, 1].max())
    cx_min, cy_min = max(x_min, 0.0), max(y_min, 0.0)
    cx_max, cy_max = min(x_max, float(width)), min(y_max, float(height))
    if cx_max - cx_min < 1.0 or cy_max - cy_min < 1.0:
        return None
    return {
        "bbox": (cx_min, cy_min, cx_max, cy_max),
        "clipped": bool(cx_min > x_min + 1e-6 or cy_min > y_min + 1e-6
                        or cx_max < x_max - 1e-6 or cy_max < y_max - 1e-6),
        "raw_bbox": (x_min, y_min, x_max, y_max),
    }


def visible_fraction(camera_pose_world: np.ndarray, k, samples_world: np.ndarray,
                     width: int, height: int) -> float:
    uv, depth = project_to_pixels(camera_pose_world, k, samples_world)
    inside = (
        (depth > 1e-6)
        & (uv[:, 0] >= 0.0) & (uv[:, 0] < width)
        & (uv[:, 1] >= 0.0) & (uv[:, 1] < height)
    )
    return float(inside.mean())


def intrinsics_from(hfov: float, width: int, height: int) -> np.ndarray:
    """由 Gazebo 的 horizontal_fov 推 K（方形像素，cx/cy 取几何中心）。

    仅用于离线自检；实际标注必须用 /camera_info 里的真 K。
    """
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])


def matrix_to_quaternion(matrix: np.ndarray) -> list[float]:
    """旋转矩阵 -> 四元数 [x, y, z, w]（与 ROS/Gazebo 的存储顺序一致）。"""
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (0.25 * s,
                      (rotation[2, 1] - rotation[1, 2]) / s,
                      (rotation[0, 2] - rotation[2, 0]) / s,
                      (rotation[1, 0] - rotation[0, 1]) / s)
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w, x, y, z = ((rotation[2, 1] - rotation[1, 2]) / s,
                      0.25 * s,
                      (rotation[0, 1] + rotation[1, 0]) / s,
                      (rotation[0, 2] + rotation[2, 0]) / s)
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w, x, y, z = ((rotation[0, 2] - rotation[2, 0]) / s,
                      (rotation[0, 1] + rotation[1, 0]) / s,
                      0.25 * s,
                      (rotation[1, 2] + rotation[2, 1]) / s)
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w, x, y, z = ((rotation[1, 0] - rotation[0, 1]) / s,
                      (rotation[0, 2] + rotation[2, 0]) / s,
                      (rotation[1, 2] + rotation[2, 1]) / s,
                      0.25 * s)
    return [float(x), float(y), float(z), float(w)]
