#!/usr/bin/env python3
"""前置相机取帧：存 PNG + 该帧的 CameraInfo 与相机世界位姿，供自动标注使用。

只依赖系统 python3 + 已 source 的 ROS2（本机没装 cv_bridge，图像用 numpy 手工解码）。

用法（需要仿真在跑）：
  source /opt/ros/humble/setup.bash && source base_robot_ws/install/setup.bash
  python3 color_yolo/tools/capture_frames.py --tag room1 --count 5
  # sweep_stations.py 会以子进程方式调用它，并用 --meta-json 记录当次停靠位
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

# encoding -> (通道数, 是否需要翻转成 BGR)
CHANNELS = {"rgb8": (3, True), "bgr8": (3, False), "rgba8": (4, True), "bgra8": (4, False), "mono8": (1, None)}


def to_bgr(msg: Image) -> np.ndarray:
    """解 Image 为 HxWx3 BGR。

    cv2.imwrite 按 BGR 解释数组，而 Gazebo 发的是 rgb8，所以必须翻一次，
    否则存出来的 PNG 红蓝互换、和后续推理时看到的颜色不一致。
    """
    encoding = msg.encoding.lower()
    if encoding not in CHANNELS:
        raise RuntimeError(f"不支持的编码 {msg.encoding}（可用：{sorted(CHANNELS)}）")
    channels, swap = CHANNELS[encoding]
    # sensor_msgs/Image 没有 bit_depth 字段，位深只能从 encoding 后缀判断
    dtype = np.uint16 if encoding.endswith("16") else np.uint8
    stride = msg.step if msg.step else msg.width * channels * np.dtype(dtype).itemsize
    flat = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, stride)
    image = flat[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        image = np.repeat(image, 3, axis=2)
    elif channels == 4:
        image = image[:, :, :3]
    if swap:
        image = image[:, :, ::-1].copy()
    return image


class FrameCapture(Node):
    def __init__(self, cfg: argparse.Namespace):
        super().__init__("color_yolo_frame_capture")
        self.cfg = cfg
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.image: Image | None = None
        self.info: CameraInfo | None = None
        self.fresh = False
        self.create_subscription(Image, cfg.image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, cfg.info_topic, self._on_info, 10)

    def _on_image(self, msg: Image) -> None:
        self.image = msg
        self.fresh = True

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def spin_until_ready(self, timeout: float) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is not None and self.info is not None:
                return
        raise TimeoutError(
            f"{timeout}s 内没收到 {self.cfg.image_topic} / {self.cfg.info_topic}；"
            "确认仿真在跑且前置相机已加载（`ros2 topic hz` 可自查）"
        )

    def spin_next_frame(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.fresh and self.image is not None and self.info is not None:
                self.fresh = False
                return True
        return False

    def camera_pose(self) -> dict:
        if self.cfg.pose_source == "explicit":
            given = json.loads(self.cfg.camera_pose)
            given.setdefault("root_frame", self.cfg.root_frame)
            given.setdefault("optical_frame", self.cfg.optical_frame)
            return given
        # 传 Time() 取最新可用变换：相机帧由 Gazebo 插件发，时刻与 TF 不完全对齐，
        # 按精确 stamp 查询会在停车时抖出 extrapolation 错误。
        handle = self.buffer.lookup_transform(self.cfg.root_frame, self.cfg.optical_frame,
                                              Time(), Duration(seconds=self.cfg.tf_timeout))
        t, q = handle.transform.translation, handle.transform.rotation
        return {
            "root_frame": self.cfg.root_frame,
            "optical_frame": self.cfg.optical_frame,
            "position": [t.x, t.y, t.z],
            "quaternion": [q.x, q.y, q.z, q.w],
            "tf_stamp": handle.header.stamp.sec + handle.header.stamp.nanosec * 1e-9,
        }

    def save(self, index: int, meta: dict) -> Path:
        image = to_bgr(self.image)
        info = self.info
        if (info.width, info.height) != (image.shape[1], image.shape[0]):
            raise RuntimeError(f"CameraInfo {info.width}x{info.height} 与图像 {image.shape[1]}x{image.shape[0]} 不符")
        now = self.get_clock().now().to_msg()

        out = Path(self.cfg.out)
        out.mkdir(parents=True, exist_ok=True)
        base = out / f"{self.cfg.tag}_{index:05d}"
        if not cv2.imwrite(f"{base}.png", image):
            raise RuntimeError(f"写图失败：{base}.png")

        record = {
            "image": f"{base.name}.png",
            "stamp": now.sec + now.nanosec * 1e-9,
            "camera": {
                "width": int(info.width),
                "height": int(info.height),
                "K": [float(v) for v in info.k],
                "D": [float(v) for v in info.d],
                "distortion_model": info.distortion_model,
                "source_encoding": self.image.encoding,
            },
            "pose": self.camera_pose(),
            "meta": meta,
        }
        Path(f"{base}.json").write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
        return base


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="前置相机取帧（含 CameraInfo 与 TF 位姿）")
    parser.add_argument("--image-topic", default="/front_camera_sensor/image_raw")
    parser.add_argument("--info-topic", default="/front_camera_sensor/camera_info")
    parser.add_argument("--root-frame", default="map", help="TF 根帧：跑 Nav2/amcl 用 map，只有里程计填 odom")
    parser.add_argument("--optical-frame", default="front_camera_optical_link")
    parser.add_argument("--out", default="color_yolo/raw/frames")
    parser.add_argument("--tag", default="cap")
    parser.add_argument("--count", type=int, default=5, help="存多少帧")
    parser.add_argument("--duration", type=float, default=0.0, help=">0 时按秒限时，与 count 谁先到算谁")
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    parser.add_argument("--camera-timeout", type=float, default=15.0)
    parser.add_argument("--quiet-timeout", type=float, default=2.0,
                        help="连续多少秒采不到新内容就收工（静止场景里重复帧没有采集价值）")
    parser.add_argument("--meta-json", default="{}", help="附加到每帧 json 的元数据（sweep 使用）")
    parser.add_argument("--pose-source", choices=("tf", "explicit"), default="tf",
                        help="tf：从 TF 树查相机世界位姿；explicit：用调用方给定的位姿"
                             "（teleport 采集时位姿是已知量，比读 AMCL/里程计更准）")
    parser.add_argument("--camera-pose", default="{}",
                        help='pose_source=explicit 时的相机光学位世界位姿 JSON，'
                             '形如 {"position":[x,y,z],"quaternion":[x,y,z,w]}')
    return parser.parse_args()


def main() -> int:
    cfg = parse_args()
    meta = json.loads(cfg.meta_json)
    rclpy.init()
    node = FrameCapture(cfg)
    try:
        node.spin_until_ready(cfg.camera_timeout)
        start, saved, last_digest = time.time(), 0, None
        last_new = time.time()
        while saved < cfg.count and (cfg.duration <= 0 or time.time() - start < cfg.duration):
            if not node.spin_next_frame(2.0):
                break
            digest = hashlib.sha1(node.image.data).hexdigest()
            if digest == last_digest:
                # 静止场景里连续帧完全相同，存下来只会造出重复样本，
                # 让 train/val 泄漏、mAP 虚高，所以直接跳过。
                if time.time() - last_new > cfg.quiet_timeout:
                    break
                continue
            last_digest, last_new = digest, time.time()
            base = node.save(saved, meta)
            saved += 1
            print(f"[{saved}/{cfg.count}] {base}.png", flush=True)
        if saved == 0:
            print("没有采到唯一帧（场景静止或相机无新数据）")
        print(f"完成：{saved} 帧 -> {cfg.out}")
        return 0 if saved else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
