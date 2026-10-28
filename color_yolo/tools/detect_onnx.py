#!/usr/bin/env python3
"""单帧颜色推理：输入一张图，输出 JSON（color + 检测框），纯 CPU、无 ROS 依赖。

必须用装了 ultralytics 的 python 跑（本机为 conda color 环境）：
  /home/lhy/miniforge3/envs/color/bin/python color_yolo/tools/detect_onnx.py \
      --image frame.png --model color_yolo/model/best.onnx --conf 0.3

模型是 YOLO11n 导出的 ONNX（红/绿 2 类，imgsz=320），ultralytics 会自动走
onnxruntime CPUExecutionProvider，不占显存，可与 OpenVLA 共存。

输出协议（stdout 最后一行，单行 JSON）：
  {"color": "red|green|none", "detections": [{"class","conf","xyxy"}, ...]}
color 取置信度最高的检测框的类别；agent_demo 的 detect_color 子进程按此解析。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO ONNX 单帧颜色检测（CPU）")
    parser.add_argument("--image", required=True, help="待检测图片路径")
    parser.add_argument("--model", required=True, help="best.onnx 路径")
    parser.add_argument("--conf", type=float, default=0.3,
                        help="置信度阈值：40 帧全量抽查 conf=0.25 已 0 误检，部署默认再收一档")
    parser.add_argument("--imgsz", type=int, default=320, help="与训练/导出保持一致，别动")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = Path(args.image)
    if not image.is_file():
        print(f"图片不存在: {image}", file=sys.stderr)
        return 2

    # 延迟导入：ultralytics 加载要 ~1s，放前面会拖慢 --help
    from ultralytics import YOLO

    model = YOLO(str(args.model))
    start = time.perf_counter()
    results = model.predict(str(image), imgsz=args.imgsz, conf=args.conf, verbose=False)
    elapsed_ms = (time.perf_counter() - start) * 1000.0

    names = model.names
    detections = []
    for box in results[0].boxes:
        cls_id = int(box.cls)
        detections.append({
            "class": names[cls_id],
            "conf": round(float(box.conf), 3),
            "xyxy": [round(float(v), 1) for v in box.xyxy[0].tolist()],
        })
    detections.sort(key=lambda d: d["conf"], reverse=True)
    color = detections[0]["class"] if detections else "none"

    print(json.dumps({"color": color, "detections": detections,
                      "infer_ms": round(elapsed_ms, 1)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
