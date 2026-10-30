# Gazebo 演示数据 LoRA 微调（方案 A：npy 直连，不走 RLDS/tfds）

数据链路：`collect_4room_demos.py` 采集 → `convert_demos.py` 转换/归一化统计 →
`finetune.py`（`GazeboDemosDataset`）LoRA 训练 → `deploy_4bit.py` 加载合并后 ckpt 推理。

> **v2 数据（2026-10-30 重采，当前使用）**：旧版 `gazebo_4room`（51 条）有 ~40% 零动作
> 静止帧，LoRA 会坍缩成"不动也高分"的 do-nothing 策略。v2 已修复采集脚本（静止帧裁剪到
> ~7%、夹爪改连续值 开度/0.04、ep 编号自动续接），新数据在
> `vla_demos_v2/`（16 停靠位 × 58 步 = 928 帧），转换产物 `datasets/gazebo_4room_v2/`。
> 训练/部署一律用 v2 目录名。

## 1. 转换数据（CPU 即可，系统 python3 + numpy/cv2）

```bash
cd openvla
python3 vla-scripts/convert_demos.py --in ../vla_demos_v2 --out datasets/gazebo_4room_v2
# 产物: datasets/gazebo_4room_v2/ep_XXXX/{images.npy (T,224,224,3), actions.npy, meta.json}
#       datasets/gazebo_4room_v2/dataset_statistics.json  (unnorm_key = "gazebo_4room_v2")
# 采集时已用 --instruction 内置指令，无需再 --set-instruction
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
    --data_root_dir datasets --dataset_name gazebo_4room_v2 \
    --run_root_dir runs \
    --batch_size 4 --max_steps 600 --save_steps 200 --learning_rate 5e-4 \
    --lora_rank 32 --use_quantization --image_aug
```

- v2 数据只有 928 步（bs4 × 600 步 ≈ 2.6 epoch），`--max_steps` 建议 300–800。旧版按
  4590 步调的 2000 会把小数据灌到 8+ epoch，重新背下残余静止模式（盯 W&B 的
  action_accuracy，>0.8 后不必再训；另建议肉眼抽查预测动作非零比例）。
- `--use_quantization` 给显存 <24GB 用（省显存但略伤效果）；24GB 以上可去掉。
- ckpt 落在 `runs/<exp_id>/`，其中自动写入 `dataset_statistics.json`（推理反归一化用）。

## 3. 部署推理

```bash
cd openvla/vla-scripts
VLA_UNNORM_KEY=gazebo_4room_v2 python3 deploy_4bit.py --openvla_path ../runs/<exp_id>
```

- `deploy_4bit.py` 已打补丁：加载本地 ckpt 目录时自动读取同目录 `dataset_statistics.json`。
- ckpt 目录里只有 `gazebo_4room_v2` 一个 key 时，`VLA_UNNORM_KEY` 可留空（`VLA_UNNORM_KEY=`
  显式置空自动选唯一 key）；**完全不设该变量默认是 `bridge_orig`，key 不存在时每次 /act
  都返回 error、机械臂一步不动**——排查"不动"先查这里。

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

依据（gazebo_4room_v2 数据统计，q01/q99）：dx±0.031 m、dz±0.026 m、rx±0.174 rad/步，
与旧版量纲一致；夹爪 q01/q99 变为 0.75/1.0（连续值，推理端不会再全闭合夹飞杯子）。
默认的 `max_angular_step=0.1` 会把倾倒段削顶，必须放宽到 ≥0.2；
若仍用默认 `position_scale=0.5`，机械臂只会走学到步幅的一半、够不到杯子。

## 5. 采集新数据后续接

```bash
# 采集（v2 脚本已修复：ep 编号自动续接 --out 目录、静止帧裁剪、夹爪连续值）
ros2 run base_robot collect_4room_demos.py --cycles 1 \
    --out /path/to/vla_demos_v2 --instruction "pick up the cup and pour"
# 转换到新目录（统计一起重算；勿混入旧 gazebo_4room 目录）
python3 openvla/vla-scripts/convert_demos.py --in /path/to/vla_demos_v2 \
    --out datasets/gazebo_4room_v2
```

- 个别停靠位夹取失败（杯子被扫到桌边滚落）属正常，脚本自动跳过不保存，可单独重跑
  `--stations <下标>` 补采。
- **幽灵杯子**：某次抓取把杯子撞坏后，模型可能卡在世界原点 (0,0,0) 且对
  `gz model -x/-y/-z` 完全无响应（验证行会显示 x=-13.4 这类离谱相对位置）。恢复办法：
  从 `build/base_robot/world_gen/demo_scene.world` 抠出该 cup 的 `<model>` 块，外面包一层
  `<sdf version="1.6">`，然后 `gz model -d -m <杯子名>` 删除、
  `gz model -m <杯子名> -f /tmp/cup.sdf` 重新 spawn，即可继续 reset_cup。

## 已知边界

- 归一化用 q01/q99（非 min/max），落在分位带外的极端步会被 clip 到 ±1，与原值有毫米级偏差——与 OpenVLA 上游行为一致。
- `labels == input_ids` 全序列监督（同上游 finetune），prompt 段也进 loss。
- 图像增广仅做亮度/对比度扰动，不做几何翻转（翻转会改变 dy/rx/rz 符号，动作就错了）。
