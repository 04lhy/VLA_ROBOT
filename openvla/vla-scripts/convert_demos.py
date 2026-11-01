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


def drop_stall_runs(actions: np.ndarray, stall_eps: float, grip_eps: float):
    """删掉全部连续零位移帧段；段末夹爪值并入前一个运动帧（否则丢闭合指令）。

    零位移样本是闭环"卡死吸引子"的来源（见 2026-10-31 第七轮复盘）：模型在错误
    相位提前背诵 f9/f10 的 (0,0,0,0,0,0,0.75) 模式，机械臂停住、画面静止、死锁。
    裁剪后训练集内不再存在 mid-episode 零动作帧。
    返回 (keep_idx, 修改过夹爪列的 actions, 报告)。
    """
    T = actions.shape[0]
    stall = np.abs(actions[:, :6]).max(axis=1) < stall_eps
    a = actions.astype(np.float32).copy()
    keep, merged, dropped = [], 0, 0
    i = 0
    while i < T:
        if not stall[i]:
            keep.append(i)
            i += 1
            continue
        j = i
        while j < T and stall[j]:
            j += 1
        dropped += j - i
        if keep:                                   # 正常情况：并入前一个运动帧
            g_last = a[j - 1, 6]
            if abs(g_last - a[keep[-1], 6]) > grip_eps:
                a[keep[-1], 6] = g_last
                merged += 1
        else:                                      # 兜底：episode 开头就静止，保留段末帧
            keep.append(j - 1)
            dropped -= 1
        i = j
    keep_idx = np.array(keep, dtype=int)
    return keep_idx, a[keep_idx], {"dropped": dropped, "gripper_merged": merged}


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


def stack_prev_frames(small: np.ndarray) -> np.ndarray:
    """(T,S,S,3) -> 每帧变为 [上一帧|当前帧] 横拼后压回 (T,S,S,3)。

    单帧输入在倾倒段物理上不可判别相位（上/下行同一腕角画面相同），
    拼上帧画面差 = 转动方向。帧 0 两半都用当前帧。**裁剪之后**才堆叠，
    "上一帧"= 裁剪序列里的前一个保留帧。推理端 vla_gazebo_bridge.py
    `-p stack_frames:=true` 是同一流程（先各自 resize 224 → 拼 448 → INTER_AREA
    压回 224），两端必须逐像素一致，改一处就要重训+重部署。
    """
    prev = np.empty_like(small)
    prev[0] = small[0]
    prev[1:] = small[:-1]
    pair = np.concatenate([prev, small], axis=2)  # (T, S, 2S, 3)
    return resize_images(pair, small.shape[1])


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
    parser.add_argument("--no-drop-stall", action="store_true",
                        help="不裁剪零位移静止帧（默认裁剪，段末夹爪值并入前一运动帧）")
    parser.add_argument("--stall-eps", type=float, default=0.003,
                        help="单维位移小于该值视为零（与桥接 stall_eps 一致）")
    parser.add_argument("--gripper-merge-eps", type=float, default=0.05,
                        help="静止段段末夹爪与前帧差超该值才做并入")
    parser.add_argument("--stack-frames", action="store_true",
                        help="帧堆叠输入（v4 权重起用）：[上一帧|当前帧] 拼合压回 224；"
                             "部署端桥接须同时 -p stack_frames:=true")
    args = parser.parse_args()

    stats_key = args.stats_key or args.out_dir.name
    ep_dirs = sorted(p for p in args.in_dir.glob("ep_*") if (p / "meta.json").exists())
    if not ep_dirs:
        raise SystemExit(f"未在 {args.in_dir} 找到 ep_XXXX 目录")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    all_actions, kept, dropped = [], 0, 0
    stall_total, merge_total = 0, 0
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

        # 裁剪零位移静止帧（闭环卡死吸引子治理，见 drop_stall_runs docstring）
        T0 = actions.shape[0]
        if not args.no_drop_stall:
            keep, actions, rep = drop_stall_runs(actions, args.stall_eps, args.gripper_merge_eps)
            images = images[keep]
            # meta 逐帧数组同步裁剪：桥接脱困重放读 ee_poses，必须与 actions 对齐
            for key in ("ee_poses", "joint_states", "is_first", "is_last"):
                v = meta.get(key)
                if isinstance(v, list) and len(v) == T0:
                    meta[key] = [v[i] for i in keep.tolist()]
            meta["stall_frames_dropped"] = sorted(set(range(T0)) - set(keep.tolist()))
            stall_total += rep["dropped"]
            merge_total += rep["gripper_merged"]

        small = resize_images(images, args.size)
        if args.stack_frames:
            small = stack_prev_frames(small)
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
        meta["stack_frames"] = bool(args.stack_frames)
        with open(dst / "meta.json", "w") as f:
            json.dump(meta, f, indent=2)

        all_actions.append(actions)
        kept += 1
        print(f"转换 {src.name} -> {ep_name}  T={actions.shape[0]}/{T0} 裁静止 {T0 - actions.shape[0]}  "
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
    if not args.no_drop_stall:
        print(f"静止帧裁剪: 删 {stall_total} 帧，{merge_total} 段夹爪目标值已并入前一运动帧")
    print(f"数据目录: {args.out_dir}")
    print(f"统计文件: {stats_path}  (unnorm_key = {stats_key!r})")
    print(f"{'dim':>8}  {'q01':>10} {'q99':>10}")
    for n, q01, q99 in zip(dim_names, stats["q01"], stats["q99"]):
        print(f"{n:>8}  {q01:>10.4f} {q99:>10.4f}")
    print("\n训练命令见 openvla/vla-scripts/GAZEBO_FINETUNE.md")


if __name__ == "__main__":
    main()
