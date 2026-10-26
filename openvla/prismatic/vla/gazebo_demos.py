"""
gazebo_demos.py

方案 A：直接读取 `vla-scripts/convert_demos.py` 产出的 npy 目录（ep_XXXX/ + dataset_statistics.json），
替代上游缺失的 RLDS/tfds 数据链路，供 finetune.py 使用。

样本构造与上游 `RLDSBatchTransform` 语义一致：
  - prompt:  PurePromptBuilder =>  "In: What action should the robot take to {instruction}?\nOut: {7 个 action token}</s>"
  - labels == input_ids（全序列 next-token 监督，同上游 LoRA finetune）
  - action 先用 dataset_statistics.json 的 q01/q99 归一化到 [-1, 1] 再做 256-bin 离散化；
    归一化公式与 predict_action 的反归一化 `0.5*(a+1)*(q99-q01)+q01` 严格互逆。
"""

import json
import random
from pathlib import Path
from typing import Dict, Tuple, Type

import numpy as np
import torch
from PIL import Image, ImageEnhance
from torch.utils.data import Dataset
from transformers import PreTrainedTokenizerBase

from prismatic.models.backbones.llm.prompting import PurePromptBuilder, VicunaV15ChatPromptBuilder
from prismatic.vla.action_tokenizer import ActionTokenizer

# HuggingFace Default / LLaMa-2 IGNORE_INDEX (for labels)
IGNORE_INDEX = -100


def save_dataset_statistics(dataset_statistics: Dict, run_dir: Path) -> None:
    """Drop-in 替代上游 rlds.utils.data_utils.save_dataset_statistics（schema 一致，json 里已是 list）。"""
    out_f = Path(run_dir) / "dataset_statistics.json"
    with open(out_f, "w") as f:
        json.dump(dataset_statistics, f, ensure_ascii=False, indent=4)


class GazeboDemosDataset(Dataset):
    def __init__(
        self,
        dataset_dir: Path,
        action_tokenizer: ActionTokenizer,
        base_tokenizer: PreTrainedTokenizerBase,
        image_transform,
        prompt_builder_fn: Type = PurePromptBuilder,
        stats_key: str = None,
        image_aug: bool = False,
    ) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.action_tokenizer = action_tokenizer
        self.base_tokenizer = base_tokenizer
        self.image_transform = image_transform
        self.prompt_builder_fn = prompt_builder_fn
        self.image_aug = image_aug

        # 加载动作归一化统计；stats_key 不存在且只有一份统计时自动取之
        stats_path = self.dataset_dir / "dataset_statistics.json"
        assert stats_path.exists(), f"缺少 {stats_path}，请先运行 vla-scripts/convert_demos.py"
        with open(stats_path, "r") as f:
            self.dataset_statistics = json.load(f)
        if stats_key not in self.dataset_statistics:
            assert len(self.dataset_statistics) == 1, \
                f"stats_key {stats_key!r} 不在 {list(self.dataset_statistics)} 中"
            stats_key = next(iter(self.dataset_statistics))
        action_stats = self.dataset_statistics[stats_key]["action"]
        self.q01 = np.array(action_stats["q01"], dtype=np.float32)
        self.q99 = np.array(action_stats["q99"], dtype=np.float32)

        # 建立 (episode_idx, step) 扁平索引
        ep_dirs = sorted(self.dataset_dir.glob("ep_*"))
        assert ep_dirs, f"{self.dataset_dir} 下没有 ep_XXXX 目录"
        self.ep_dirs = ep_dirs
        self.instructions = []
        self.samples: list = []
        for ei, ep in enumerate(ep_dirs):
            with open(ep / "meta.json") as f:
                self.instructions.append(json.load(f)["instruction"])
            actions = np.load(ep / "actions.npy", mmap_mode="r")
            images = np.load(ep / "images.npy", mmap_mode="r")
            assert images.shape[0] == actions.shape[0], f"{ep.name}: images/actions 步数不一致"
            assert images.shape[1:3] == (224, 224), \
                f"{ep.name}: 图像为 {images.shape[1:3]}，应为 224x224（先跑 convert_demos.py）"
            self.samples.extend((ei, t) for t in range(actions.shape[0]))

        # mmap 句柄按需打开（num_workers=0 下缓存安全）
        self._images_cache: Dict[int, np.ndarray] = {}
        self._actions_cache: Dict[int, np.ndarray] = {}

    def __len__(self) -> int:
        return len(self.samples)

    def _norm_action(self, raw: np.ndarray) -> np.ndarray:
        """与 predict_action 反归一化互逆：a' = (2*(a - q01) - (q99 - q01)) / (q99 - q01)。"""
        rng = self.q99 - self.q01
        safe = np.where(rng > 1e-6, rng, 1.0)
        norm = np.where(rng > 1e-6, (2.0 * (raw - self.q01) - rng) / safe, 0.0)
        return np.clip(norm, -1.0, 1.0).astype(np.float32)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        ei, t = self.samples[idx]

        if ei not in self._images_cache:
            self._images_cache[ei] = np.load(self.ep_dirs[ei] / "images.npy", mmap_mode="r")
            self._actions_cache[ei] = np.load(self.ep_dirs[ei] / "actions.npy", mmap_mode="r")
        image = Image.fromarray(np.ascontiguousarray(self._images_cache[ei][t])).convert("RGB")
        if self.image_aug:
            # 只做光度扰动（不做几何变换，避免动作与画面错位）
            image = ImageEnhance.Brightness(image).enhance(random.uniform(0.8, 1.2))
            image = ImageEnhance.Contrast(image).enhance(random.uniform(0.8, 1.2))
        pixel_values = self.image_transform(image)

        action = self._norm_action(np.asarray(self._actions_cache[ei][t]))
        action_string = self.action_tokenizer(action)

        prompt_builder = self.prompt_builder_fn("openvla")
        conversation = [
            {"from": "human", "value": f"What action should the robot take to {self.instructions[ei]}?"},
            {"from": "gpt", "value": action_string},
        ]
        for turn in conversation:
            prompt_builder.add_turn(turn["from"], turn["value"])

        # Tokenize (and Label) =>> labels == input_ids（全序列监督，同上游）
        input_ids = self.base_tokenizer(prompt_builder.get_prompt(), add_special_tokens=True).input_ids
        labels = list(input_ids)

        return dict(
            pixel_values=pixel_values,
            input_ids=torch.tensor(input_ids, dtype=torch.long),
            labels=torch.tensor(labels, dtype=torch.long),
        )
