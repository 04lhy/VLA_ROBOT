#!/usr/bin/env python3
"""convert_demos.py

把采集脚本（collect_4room_demos.py / collect_grasp_demos.py）产出的 npy episode 目录，
转换成 finetune.py「方案 A」直接可训练的格式（不经过 RLDS/tfds）。

做四件事：
  1. 扫描输入目录的 ep_XXXX/，丢弃 grasp_success=false 的 episode
  2. 图像 320x240 -> 224x224 重采样并落盘（训练时不再每帧 resize）
  3. 连续重编号 ep_0000...（meta.json 里记录来源目录，可追溯）
  4. 统计全数据集 actions 每维 low/high/q01/q99 -> dataset_statistics.json
     schema 与 OpenVLA 推理侧完全对齐：{unnorm_key: {"action": {low/high/q01/q99}}}
     predict_action 的反归一化公式为 0.5*(a+1)*(q99-q01)+q01，训练侧归一化即其逆。

用法（系统 python3 即可，只需 numpy + cv2，无需 GPU/vla_env）：
  cd openvla
  python3 vla-scripts/convert_demos.py --in ../vla_demos/vla_demos

  # 按房间颜色生成多种指令，避免 51 条数据语言全是一句：
  python3 vla-scripts/convert_demos.py --in ../vla_demos/vla_demos --varied-instructions
"""

import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np

# station 名 -> 房间颜色（collect_4room_demos.py 的命名：如 southeast_blue_west）
COLOR_WORDS = ("red", "blue", "green", "yellow")


def station_instruction(station: str, default: str) -> str:
    """按停靠位所在房间颜色改写指令；匹配不到颜色则用默认。"""
    for c in COLOR_WORDS:
        if c in station:
            return f"pick up the {c} cup"
    return default


def resize_images(images: np.ndarray, size: int) -> np.ndarray:
    """(T, H, W, 3) uint8 -> (T, size, size, 3) uint8。"""
    out = np.empty((images.shape[0], size, size, 3), dtype=np.uint8)
    for i in range(images.shape[0]):
        out[i] = cv2.resize(images[i], (size, size), interpolation=cv2.INTER_AREA)
    return out


def main():
    parser = argparse.ArgumentParser(description="vla_demos npy -> 方案A训练格式")
    here = Path(__file__).resolve()
    parser.add_argument("--in", dest="in_dir", type=Path,
                        default=here.parent.parent.parent / "vla_demos" / "vla_demos",
                        help="采集输出目录（内含 ep_XXXX/）")
    parser.add_argument("--out", dest="out_dir", type=Path,
                        default=here.parent.parent / "datasets" / "gazebo_4room",
                        help="转换后输出目录")
    parser.add_argument("--stats-key", default=None,
                        help="dataset_statistics.json 的 unnorm_key（默认取 out 目录名）")
    parser.add_argument("--size", type=int, default=224, help="图像重采样边长（OpenVLA 输入 224）")
    parser.add_argument("--varied-instructions", action="store_true",
                        help="按 station 颜色改写指令（red/blue/green/yellow 四种）")
    parser.add_argument("--set-instruction", default=None,
                        help="强制统一指令（覆盖 --varied-instructions；部署端须逐字一致）")
    parser.add_argument("--keep-failed", action="store_true",
                        help="保留 grasp_success=false 的 episode（默认丢弃）")
    args = parser.parse_args()

    stats_key = args.stats_key or args.out_dir.name
    ep_dirs = sorted(p for p in args.in_dir.glob("ep_*") if (p / "meta.json").exists())
    if not ep_dirs:
        raise SystemExit(f"未在 {args.in_dir} 找到 ep_XXXX 目录")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    all_actions, kept, dropped = [], 0, 0
    for src in ep_dirs:
        with open(src / "meta.json") as f:
            meta = json.load(f)

        if not meta.get("grasp_success", True) and not args.keep_failed:
            dropped += 1
            print(f"跳过（抓取失败）: {src.name}")
            continue

        actions = np.load(src / "actions.npy").astype(np.float32)
        images = np.load(src / "images.npy")
        assert images.shape[0] == actions.shape[0], f"{src.name}: images/actions 步数不一致"

        small = resize_images(images, args.size)
        if args.varied_instructions:
            meta["instruction"] = station_instruction(meta.get("station", ""), meta["instruction"])
        if args.set_instruction:
            meta["instruction"] = args.set_instruction

        ep_name = f"ep_{kept:04d}"
        dst = args.out_dir / ep_name
        dst.mkdir(exist_ok=True)
        np.save(dst / "images.npy", small)
        np.save(dst / "actions.npy", actions)
        meta["source_dir"] = str(src)  # 可追溯到原始采集目录
        with open(dst / "meta.json", "w") as f:
            json.dump(meta, f, indent=2)

        all_actions.append(actions)
        kept += 1
        print(f"转换 {src.name} -> {ep_name}  T={actions.shape[0]}  "
              f"instruction={meta['instruction']!r}")

    acts = np.concatenate(all_actions, axis=0)
    dim_names = ["dx", "dy", "dz", "rx", "ry", "rz", "gripper"]
    stats = {
        "low": np.min(acts, axis=0).tolist(),
        "high": np.max(acts, axis=0).tolist(),
        "q01": np.percentile(acts, 1, axis=0).tolist(),
        "q99": np.percentile(acts, 99, axis=0).tolist(),
    }
    stats_path = args.out_dir / "dataset_statistics.json"
    with open(stats_path, "w") as f:
        json.dump({stats_key: {"action": stats}}, f, indent=2)

    print(f"\n完成: {kept} 条 episode（丢弃 {dropped} 条），共 {acts.shape[0]} 步")
    print(f"数据目录: {args.out_dir}")
    print(f"统计文件: {stats_path}  (unnorm_key = {stats_key!r})")
    print(f"{'dim':>8}  {'q01':>10} {'q99':>10}")
    for n, q01, q99 in zip(dim_names, stats["q01"], stats["q99"]):
        print(f"{n:>8}  {q01:>10.4f} {q99:>10.4f}")
    print("\n训练命令见 openvla/vla-scripts/GAZEBO_FINETUNE.md")


if __name__ == "__main__":
    main()
