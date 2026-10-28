#!/usr/bin/env bash
# 阶段 0：建独立环境 color，只装 CPU 版依赖。
#
# 为什么必须独立环境：系统 python3 的 numpy 1.21.5 被 rclpy 依赖，
# 往里 pip install -U 会把 ROS2 打挂。
#
# 为什么必须用 --index-url 指向 cpu 轮子：PyPI 上的默认 torch wheel 自带 CUDA
# 运行库（约 2.5GB + 一堆 nvidia-* 包），装进来就可能悄悄占显存。
# 本环境的目标恰恰是"绝不和 OpenVLA 抢显存"，所以只允许 cpu 构建。
set -euo pipefail

ENV_NAME="${1:-color}"
MIRROR="${PIP_MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
TORCH_CPU_INDEX="${TORCH_CPU_INDEX:-https://download.pytorch.org/whl/cpu}"
CONDA_ROOT="${CONDA_ROOT:-$HOME/miniforge3}"

if [ -x "$CONDA_ROOT/bin/mamba" ]; then
  PM=("$CONDA_ROOT/bin/mamba")
elif [ -x "$CONDA_ROOT/bin/conda" ]; then
  PM=("$CONDA_ROOT/bin/conda")
else
  echo "找不到 mamba/conda（试过 $CONDA_ROOT）。用 CONDA_ROOT=... 指定。" >&2
  exit 1
fi

echo ">> 使用 $PM 创建环境 $ENV_NAME"
ENV_PY="$CONDA_ROOT/envs/$ENV_NAME/bin/python"
# 已存在则跳过创建，让脚本可以安全重跑（pip 每次都会重装到同一环境）
if [ -x "$ENV_PY" ]; then
  echo "   环境 $ENV_NAME 已存在，跳过创建"
else
  "$PM" create -y -n "$ENV_NAME" python=3.10
fi

if [ ! -x "$ENV_PY" ]; then
  echo "环境 python 不存在：$ENV_PY" >&2
  exit 1
fi

echo ">> 装 CPU 版 torch / torchvision（不走 PyPI 默认 CUDA 轮子）"
"$ENV_PY" -m pip install --index-url "$TORCH_CPU_INDEX" torch torchvision

echo ">> 装 YOLO 与导出/推理依赖（onnxruntime 仅 CPU EP）"
"$ENV_PY" -m pip install -i "$MIRROR" ultralytics opencv-python onnx onnxruntime pyyaml

echo ">> 校验"
"$ENV_PY" - <<'PY'
import importlib.metadata as md
import torch
import onnxruntime as ort

print(f"torch            {torch.__version__}  cuda_available={torch.version.cuda is not None}")
print(f"ort              {md.version('onnxruntime')}  providers={ort.get_available_providers()}")
print(f"ultralytics      {md.version('ultralytics')}")
print(f"opencv           {md.version('opencv-python')}")
assert torch.version.cuda is None, "装到了 CUDA 版 torch，与本环境的省显存目标冲突"
assert "CUDAExecutionProvider" not in ort.get_available_providers(), "装到了 GPU onnxruntime"
print("[ok] 环境为纯 CPU：不会占用任何显存")
PY

echo
echo "完成。后续激活：conda activate $ENV_NAME   （或 $(basename $PM) run -n $ENV_NAME ...）"
