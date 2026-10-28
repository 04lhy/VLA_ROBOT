#!/usr/bin/env python3
"""客观核验自动标注是否对齐：把投影框和图里该颜色的真实连通域做比对。

自动标注的正确性不靠肉眼。世界真值如果和渲染结果有偏差（markers.yaml 没跟上 world、
相机挂载改过、TF 帧名不对），投影框就会飘。这里用 HSV 找出每帧里该颜色的实际像素团块，
与标注框算 IoU 和中心偏差，直接量化"框歪了多少像素"。

用法：
  python3 color_yolo/tools/check_alignment.py            # 用 dataset/labels
  python3 color_yolo/tools/check_alignment.py --iou-min 0.7
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]

# BGR-HSV 下的颜色窗口（Gazebo 的 ambient/diffuse 是纯色，阈值可以放很宽）
HSV_RANGES = {
    "red": [((0, 80, 50), (12, 255, 255)), ((168, 80, 50), (180, 255, 255))],
    "green": [((40, 60, 50), (90, 255, 255))],
    "blue": [((95, 60, 50), (130, 255, 255))],
    "yellow": [((20, 60, 80), (38, 255, 255))],
}


def blobs_by_color(image: np.ndarray, class_name: str):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HSV_RANGES[class_name]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    best = max(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA])
    x, y, w, h, area = stats[best]
    if area < 50:
        return None
    return {"box": (float(x), float(y), float(x + w), float(y + h)),
            "centroid": (float(centroids[best][0]), float(centroids[best][1])), "area": int(area)}


def iou(a, b) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="比对投影框与真实色块，量化标注偏差")
    parser.add_argument("--raw", default=str(ROOT / "raw" / "frames"))
    parser.add_argument("--labels", default=str(ROOT / "dataset" / "labels"))
    parser.add_argument("--classes-file", default=str(ROOT / "config" / "markers.yaml"))
    parser.add_argument("--iou-min", type=float, default=0.6, help="低于该 IoU 记为可疑")
    args = parser.parse_args()

    with open(args.classes_file, encoding="utf-8") as handle:
        names = yaml.safe_load(handle)["classes"]

    rows, no_blob = [], 0
    for json_path in sorted(Path(args.raw).glob("*.json")):
        label_path = Path(args.labels) / f"{json_path.stem}.txt"
        if not label_path.exists():
            continue
        record = json.loads(json_path.read_text(encoding="utf-8"))
        image = cv2.imread(str(json_path.with_suffix(".png")))
        if image is None:
            continue
        width, height = record["camera"]["width"], record["camera"]["height"]
        for line in label_path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            cls = names[int(parts[0])]
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            box = ((cx - bw / 2) * width, (cy - bh / 2) * height,
                   (cx + bw / 2) * width, (cy + bh / 2) * height)
            blob = blobs_by_color(image, cls)
            if blob is None:
                no_blob += 1
                continue
            center = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
            rows.append({"frame": json_path.stem, "cls": cls,
                         "iou": iou(box, blob["box"]),
                         "err": float(np.hypot(center[0] - blob["centroid"][0],
                                               center[1] - blob["centroid"][1])),
                         "clipped": abs(box[1]) < 2 or abs(box[3] - height) < 2 or box[0] < 2 or box[2] > width - 2,
                         "area": blob["area"]})

    if not rows:
        print("没有可比对的标注（先跑 annotate.py）")
        return 2

    print(f"{'帧':<20}{'类':<8}{'IoU':<8}{'偏差px':<9}{'备注'}")
    for row in sorted(rows, key=lambda r: r["iou"]):
        note = "被画面边缘切断" if row["clipped"] else ""
        print(f"{row['frame']:<20}{row['cls']:<8}{row['iou']:<8.3f}{row['err']:<9.1f}{note}")

    ious = np.array([r["iou"] for r in rows])
    errs = np.array([r["err"] for r in rows])
    full = np.array([not r["clipped"] for r in rows])
    print(f"\n共 {len(rows)} 个框，另有 {no_blob} 个框在图里找不到对应色块")
    print(f"IoU: 中位 {np.median(ious):.3f} 最小 {ious.min():.3f} | 中心偏差: 中位 {np.median(errs):.1f}px 最大 {errs.max():.1f}px")
    if full.any():
        print(f"仅看完整未切断的框（{full.sum()} 个）：IoU 中位 {np.median(ious[full]):.3f}，"
              f"偏差中位 {np.median(errs[full]):.1f}px")
    bad = [r for r in rows if r["iou"] < args.iou_min]
    if bad:
        print(f"!! {len(bad)} 个框 IoU < {args.iou_min}，最可疑：{bad[0]['frame']} ({bad[0]['iou']:.3f})")
        return 1
    print("[ok] 全部框的对齐质量可接受，世界真值与渲染结果一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
