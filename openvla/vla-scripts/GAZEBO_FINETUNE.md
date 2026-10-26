# Gazebo 演示数据 LoRA 微调（方案 A：npy 直连，不走 RLDS/tfds）

数据链路：`collect_4room_demos.py` 采集 → `convert_demos.py` 转换/归一化统计 →
`finetune.py`（`GazeboDemosDataset`）LoRA 训练 → `deploy_4bit.py` 加载合并后 ckpt 推理。

## 1. 转换数据（CPU 即可，系统 python3 + numpy/cv2）

```bash
cd openvla
python3 vla-scripts/convert_demos.py --in ../vla_demos/vla_demos \
    --set-instruction "pick up the cup and pour"
# 产物: datasets/gazebo_4room/ep_XXXX/{images.npy (T,224,224,3), actions.npy, meta.json}
#       datasets/gazebo_4room/dataset_statistics.json  (unnorm_key = "gazebo_4room")
```

当前采用**单一固定指令** `pick up the cup and pour`：动作序列含倾倒段，只写
"pick up the cup" 会让语言条件盖不住观测到的动作。**部署端发话必须与训练指令
逐字一致**。以后若要练多场景，可改用 `--varied-instructions`（按房间颜色 4 种指令）。

## 2. 训练（需要 GPU，在 vla_env 里跑）

```bash
mamba activate vla_env
cd openvla
WANDB_MODE=offline torchrun --standalone --nnodes 1 --nproc-per-node 1 \
    vla-scripts/finetune.py \
    --vla_path openvla/openvla-7b \
    --data_root_dir datasets --dataset_name gazebo_4room \
    --run_root_dir runs \
    --batch_size 4 --max_steps 2000 --save_steps 500 --learning_rate 5e-4 \
    --lora_rank 32 --use_quantization --image_aug
```

- 数据只有 4590 步，`--max_steps` 建议 1000–3000，过大易过拟合（盯 W&B 的 action_accuracy，>0.8 后不必再训）。
- `--use_quantization` 给显存 <24GB 用（省显存但略伤效果）；24GB 以上可去掉。
- ckpt 落在 `runs/<exp_id>/`，其中自动写入 `dataset_statistics.json`（推理反归一化用）。

## 3. 部署推理

```bash
cd openvla/vla-scripts
VLA_UNNORM_KEY=gazebo_4room python3 deploy_4bit.py --openvla_path ../runs/<exp_id>
```

- `deploy_4bit.py` 已打补丁：加载本地 ckpt 目录时自动读取同目录 `dataset_statistics.json`。
- ckpt 目录里只有 `gazebo_4room` 一个 key 时，`VLA_UNNORM_KEY` 可留空（自动选）；不设该环境变量默认仍是 `bridge_orig`（部署基座模型用）。

## 4. 桥接节点参数（重要）

触发指令（须与训练指令逐字一致）：

```bash
ros2 topic pub /vla_instruction std_msgs/msg/String "data: 'pick up the cup and pour'" -1
```

## 4. 桥接节点参数（重要）

微调模型输出的动作**反归一化后已是原始单位**（米 / 弧度 / gripper∈[0,1]，5Hz），
`vla_gazebo_bridge.py` 的默认缩放是给基座模型（bridge_orig 量纲）调的，换微调模型后要覆盖：

```bash
ros2 run base_robot vla_gazebo_bridge.py --ros-args \
    -p position_scale:=1.0 -p rotation_scale:=1.0 \
    -p max_linear_step:=0.04 -p max_angular_step:=0.2
```

依据（gazebo_4room 数据统计，q01/q99）：dx±0.031 m、dz±0.026 m、rx±0.174 rad/步。
默认的 `max_angular_step=0.1` 会把倾倒段削顶，必须放宽到 ≥0.2；
若仍用默认 `position_scale=0.5`，机械臂只会走学到步幅的一半、够不到杯子。

## 5. 采集新数据后续接

```bash
# 采集（注意先把 collect_4room_demos.py 里 ep_idx 起始值和 --out 默认值改对）
ros2 run base_robot collect_4room_demos.py --cycles 1 --out ~/vla_demos
# 重新转换（全量覆盖 datasets/gazebo_4room，统计会一起重算）
python3 openvla/vla-scripts/convert_demos.py --in ~/vla_demos
```

## 已知边界

- 归一化用 q01/q99（非 min/max），落在分位带外的极端步会被 clip 到 ±1，与原值有毫米级偏差——与 OpenVLA 上游行为一致。
- `labels == input_ids` 全序列监督（同上游 finetune），prompt 段也进 loss。
- 图像增广仅做亮度/对比度扰动，不做几何翻转（翻转会改变 dy/rx/rz 符号，动作就错了）。
