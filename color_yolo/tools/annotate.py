#!/usr/bin/env python3
"""从世界真值自动生成 YOLO 标注：读 raw/frames 的帧 + CameraInfo + 相机位姿，出 labels/vis/data.yaml。

标注完全由几何投影产生，不需要人工框选；牌子被画面边缘切断时按裁剪框标注
（合成场景里这正是常态，模型因此能学会"只露出一条"的样子）。

离线自检（不需要仿真）：
  python3 color_yolo/tools/annotate.py --selftest
真实标注（先跑完 sweep/capture）：
  python3 color_yolo/tools/annotate.py
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import (  # noqa: E402
    Marker,
    bbox_from_pixels,
    gazebo_pose_to_matrix,
    intrinsics_from,
    matrix_to_quaternion,
    pose_to_matrix,
    project_to_pixels,
    quaternion_pose_to_matrix,
    vertical_angle_deg,
    visible_fraction,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "markers.yaml"


def load_markers(cfg: dict, classes: list[str]) -> tuple[list[Marker], dict[str, int]]:
    class_ids = {name: i for i, name in enumerate(classes)}
    markers = [
        Marker(name=item["name"], room=item["room"], class_name=item["class_name"],
               link_pose=item["pose"], size=item["size"])
        for item in cfg["markers"]
        if item["class_name"] in class_ids
    ]
    return markers, class_ids


def label_frame(record: dict, cfg: dict, markers: list[Marker], class_ids: dict[str, int],
                min_visible: float, min_height_px: float):
    pose = record["pose"]
    camera_pose = quaternion_pose_to_matrix(pose["position"], pose["quaternion"])
    camera = record["camera"]
    width, height = int(camera["width"]), int(camera["height"])
    k = np.asarray(camera["K"], dtype=float)
    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])

    labels, details = [], []
    for marker in markers:
        corners = marker.corners_world(model)
        uv, depth = project_to_pixels(camera_pose, k, corners)
        box = bbox_from_pixels(uv, depth, width, height)
        fraction = visible_fraction(camera_pose, k, marker.face_samples_world(model), width, height)
        detail = {"class": marker.class_name, "visible_fraction": round(fraction, 3)}
        if box is None:
            detail["drop"] = "out_of_frame"
            details.append(detail)
            continue
        x_min, y_min, x_max, y_max = box["bbox"]
        detail.update({"px_h": round(y_max - y_min, 1), "px_w": round(x_max - x_min, 1),
                       "clipped": box["clipped"]})
        if fraction < min_visible:
            detail["drop"] = f"visible<{min_visible}"
            details.append(detail)
            continue
        if (y_max - y_min) < min_height_px:
            detail["drop"] = f"height<{min_height_px}px"
            details.append(detail)
            continue
        cx, cy = (x_min + x_max) / 2 / width, (y_min + y_max) / 2 / height
        bw, bh = (x_max - x_min) / width, (y_max - y_min) / height
        labels.append(f"{class_ids[marker.class_name]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        detail["box"] = [round(v, 1) for v in box["bbox"]]
        details.append(detail)
    return labels, details


def overlay(image: np.ndarray, details: list[dict], class_names: list[str]) -> np.ndarray:
    canvas = image.copy()
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
    for detail in details:
        if "box" not in detail:
            continue
        x_min, y_min, x_max, y_max = detail["box"]
        index = class_names.index(detail["class"]) % len(colors)
        color = colors[index]
        cv2.rectangle(canvas, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
        text = f"{detail['class']} {detail['visible_fraction'] * 100:.0f}%"
        cv2.putText(canvas, text, (int(x_min), max(12, int(y_min) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return canvas


def build_dataset(args: argparse.Namespace) -> int:
    with CONFIG.open("r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    classes = args.classes or cfg["classes"]
    markers, class_ids = load_markers(cfg, classes)

    frames_dir = Path(args.raw)
    records = sorted(frames_dir.glob("*.json"))
    if not records:
        print(f"{frames_dir} 下没有 *.json 帧记录，先跑 capture_frames.py / sweep_stations.py", file=sys.stderr)
        return 2

    images_dir = Path(args.out) / "images"
    labels_dir = Path(args.out) / "labels"
    vis_dir = Path(args.out) / "vis"
    for path in (images_dir, labels_dir, vis_dir):
        path.mkdir(parents=True, exist_ok=True)

    stats = {"frames": 0, "labeled": 0, "boxes": 0, "clipped": 0,
             "per_class": {name: 0 for name in classes}, "drops": {}}
    written: list[str] = []
    rejected: list[dict] = []

    for record_path in records:
        if args.limit and stats["frames"] >= args.limit:
            break
        record = json.loads(record_path.read_text(encoding="utf-8"))
        source = record_path.with_suffix(".png")
        if not source.exists():
            rejected.append({"frame": record_path.name, "drop": "missing_png"})
            continue
        labels, details = label_frame(record, cfg, markers, class_ids, args.min_visible, args.min_height_px)
        stats["frames"] += 1
        for detail in details:
            if "drop" in detail:
                stats["drops"][detail["drop"].split("<")[0]] = stats["drops"].get(detail["drop"].split("<")[0], 0) + 1
            else:
                stats["boxes"] += 1
                stats["per_class"][detail["class"]] += 1
                stats["clipped"] += int(detail["clipped"])

        if not labels:
            rejected.append({"frame": record_path.name, "drop": "no_label",
                             "meta": record.get("meta"), "details": details})
            continue

        stats["labeled"] += 1
        stem = record_path.stem
        target_image = images_dir / f"{stem}.png"
        if args.copy:
            shutil.copy2(source, target_image)
        elif not target_image.is_symlink():
            if target_image.exists():
                target_image.unlink()
            target_image.symlink_to(source.resolve())
        (labels_dir / f"{stem}.txt").write_text("\n".join(labels) + "\n", encoding="utf-8")
        written.append(str(target_image))

        if not args.no_vis:
            image = cv2.imread(str(source))
            if image is not None:
                cv2.imwrite(str(vis_dir / f"{stem}.png"), overlay(image, details, classes))

    # 按"位姿"而不是按"帧"切分：同一停靠位连拍的几帧几乎相同，
    # 若随机按帧分train/val，等于把同一画面同时喂进训练和验证，mAP 会虚高。
    groups: dict[str, list[str]] = {}
    for path in written:
        groups.setdefault(Path(path).stem.rsplit("_", 1)[0], []).append(path)
    keys = list(groups)
    rng = np.random.default_rng(0)
    rng.shuffle(keys)
    split_at = max(1, int(len(keys) * 0.85)) if keys else 0
    train_list = [p for key in keys[:split_at] for p in groups[key]]
    val_list = [p for key in keys[split_at:] for p in groups[key]]
    split_dir = Path(args.out) / "splits"
    split_dir.mkdir(parents=True, exist_ok=True)
    (split_dir / "train.txt").write_text("\n".join(train_list) + "\n", encoding="utf-8")
    (split_dir / "val.txt").write_text("\n".join(val_list) + "\n", encoding="utf-8")

    data_yaml = {
        "path": str(Path(args.out).resolve()),
        "train": "splits/train.txt",
        "val": "splits/val.txt",
        "names": {i: name for i, name in enumerate(classes)},
    }
    (Path(args.out) / "data.yaml").write_text(
        yaml.safe_dump(data_yaml, sort_keys=False, allow_unicode=True), encoding="utf-8")
    (Path(args.out) / "rejected.jsonl").write_text(
        "\n".join(json.dumps(item, ensure_ascii=False) for item in rejected), encoding="utf-8")

    print(f"帧 {stats['frames']}，出标签 {stats['labeled']}，框 {stats['boxes']}"
          f"（被边缘切断 {stats['clipped']}）")
    print("每类框数：" + ", ".join(f"{name}={count}" for name, count in stats["per_class"].items()))
    if stats["drops"]:
        print("丢弃原因计数：" + ", ".join(f"{key}:{value}" for key, value in sorted(stats["drops"].items())))
    print(f"train/val = {len(train_list)}/{len(val_list)}  ->  {Path(args.out) / 'data.yaml'}")
    if not args.no_vis:
        print(f"画框自检图 -> {vis_dir}")
    return 0 if stats["labeled"] else 1


def selftest() -> int:
    """离线验证标注链路：造一帧位姿，比对投影框与独立手算的牌子中心像素。"""
    with CONFIG.open("r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    camera = cfg["camera"]
    width, height = camera["width"], camera["height"]
    k = intrinsics_from(camera["horizontal_fov"], width, height)
    markers, class_ids = load_markers(cfg, cfg["classes"])

    station = cfg["dock_stations"]["room1"]
    # 后退 1.0m，让牌子确实进画（真实停靠位是完全出画的，见 selftest_projection.py）
    pose = (station[0] + 1.0, station[1], station[2])
    lift = float(camera["footprint_to_base_link_z"])
    base = pose_to_matrix((pose[0], pose[1], lift), (0.0, 0.0, pose[2]))
    mount = pose_to_matrix(camera["mounting_xyz"], camera["mounting_rpy"])
    optical = pose_to_matrix((0.0, 0.0, 0.0), (-np.pi / 2, 0.0, -np.pi / 2))
    camera_pose = base @ mount @ optical

    record = {
        "camera": {"width": width, "height": height, "K": [float(v) for v in k.ravel()],
                   "D": [0.0] * 5, "distortion_model": "plumb_bob"},
        "pose": {"position": [float(v) for v in camera_pose[:3, 3]],
                 "quaternion": matrix_to_quaternion(camera_pose)},
    }
    labels, details = label_frame(record, cfg, markers, class_ids, min_visible=0.02, min_height_px=4.0)
    red = next(d for d in details if d["class"] == "red")
    assert "box" in red, f"预期红色牌子进画，实际 {red}"
    print(f"[ok] 该位姿下标注到 red：框={red['box']} 可见比例={red['visible_fraction']}")

    model = gazebo_pose_to_matrix(cfg["world_model"]["pose"])
    center = next(m for m in markers if m.class_name == "red").pose_world(model)[:3, 3]
    uv, _ = project_to_pixels(camera_pose, k, [center])
    x_min, y_min, x_max, y_max = red["box"]
    assert abs(uv[0][0] - (x_min + x_max) / 2) < 3.0, (uv[0][0], x_min, x_max)
    assert abs(uv[0][1] - (y_min + y_max) / 2) < (y_max - y_min) / 2 + 3.0, (uv[0][1], y_min, y_max)
    print(f"[ok] 牌子中心像素 ({uv[0][0]:.1f},{uv[0][1]:.1f}) 落在标注框内，投影链路与标注一致")

    others = [d for d in details if d["class"] != "red"]
    assert all("drop" in d for d in others), f"对面墙的牌子不该进画：{others}"
    print(f"[ok] 其余 3 块牌子均判为不可见（{others[0].get('drop')}），无误标")

    angle = vertical_angle_deg(camera_pose, center)
    print(f"[ok] 该位姿牌子中心仰角 {angle:.1f}°，1 行标注 = {labels[0]}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="从世界真值自动生成 YOLO 标注（阶段 1）")
    parser.add_argument("--raw", default=str(ROOT / "raw" / "frames"), help="capture_frames.py 的输出目录")
    parser.add_argument("--out", default=str(ROOT / "dataset"), help="YOLO 数据集根目录")
    parser.add_argument("--classes", nargs="*", default=None,
                        help="只保留这些类（例如 --classes red green 做 2 类版），默认用配置里的 4 类")
    parser.add_argument("--min-visible", type=float, default=0.05, help="低于该可见比例丢弃")
    parser.add_argument("--min-height-px", type=float, default=8.0, help="裁剪框最小像素高")
    parser.add_argument("--limit", type=int, default=0, help="只处理前 N 帧，0 为全部")
    parser.add_argument("--copy", action="store_true", help="拷贝图片而不是软链")
    parser.add_argument("--no-vis", action="store_true", help="不生成画框自检图")
    parser.add_argument("--selftest", action="store_true", help="离线自检，不需要任何图片")
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    raise SystemExit(selftest() if arguments.selftest else build_dataset(arguments))
