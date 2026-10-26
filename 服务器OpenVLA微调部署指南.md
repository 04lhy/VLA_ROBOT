# 服务器 OpenVLA 微调部署指南（方案 A：npy 直连，无 RLDS/tensorflow）

> 2026-10-26 编写。数据已在本地转换完成（51 episodes / 4590 步 / 661MB，统一指令
> `pick up the cup and pour`，统计键 `gazebo_4room`）。本手册覆盖：传什么、环境怎么建、
> 怎么训、训完怎么部署。全程不需要在本机跑 ROS。

---

## 1. 服务器要求

| 项 | 最低 / 推荐 | 说明 |
|----|------------|------|
| GPU | 24GB（RTX 3090/4090/A5000）+ `--use_quantization`；40GB+（A100）可不量化 | 7B 模型 bf16 权重 ~15GB，量化训练省显存但略伤效果 |
| 驱动/CUDA | 驱动支持 CUDA 12.x；torch 自带 cu121 wheel，**不要求服务器装 CUDA toolkit** | |
| Python | 3.10 | transformers 4.40.1 验证版本 |
| 磁盘 | ≥ 50GB（数据 0.7G + 基座权重 15G + 合并 ckpt 15G + 余量） | |
| 网络 | 能装 pip（国内镜像即可） | 用本地权重的话**不需要能访问 huggingface.co**；走镜像 hf-mirror.com 也行 |

## 2. 要准备/传输的三样东西

### 2.1 代码（本仓库，含方案 A 全部修改）

⚠️ **当前所有改动都未 commit**（finetune.py、deploy_4bit.py、convert_demos.py、
prismatic/vla/\_\_init\_\_.py、gazebo_demos.py、GAZEBO_FINETUNE.md、vla_grasp_test.py），
走 git 传输前先提交。

```bash
# 方式一：git（推荐，先 commit+push 再在服务器 clone/pull）
git add -A && git commit -m "方案A: npy数据集训练链路" && git push

# 方式二：不 commit，直接打包（datasets 排除后只 ~几十MB 代码）
cd ~/Desktop
tar czf /tmp/vla_code.tgz --exclude='*/datasets' --exclude=build --exclude=install --exclude=log \
    VLA_ROBOT/base_robot_ws/src VLA_ROBOT/openvla VLA_ROBOT/agent_demo \
    VLA_ROBOT/*.md VLA_ROBOT/.gitignore
scp /tmp/vla_code.tgz user@server:~/
```

### 2.2 数据（二选一）

```bash
# 选项 1（推荐）：传已转换好的训练目录（661MB，服务器免装 opencv）
rsync -avP ~/Desktop/VLA_ROBOT/openvla/datasets/gazebo_4room/ user@server:~/VLA_ROBOT/openvla/datasets/gazebo_4room/

# 选项 2：传原始采集数据（~1GB），到服务器再转换（服务器需 numpy + opencv-python）
rsync -avP ~/Desktop/VLA_ROBOT/vla_demos/vla_demos/ user@server:~/vla_demos_raw/
python3 vla-scripts/convert_demos.py --in ~/vla_demos_raw --set-instruction "pick up the cup and pour"
```

`dataset_statistics.json` 在 `gazebo_4room/` 里一起传，这是推理反归一化的命根子。

### 2.3 基座权重 openvla-7b（二选一）

- 本地已有缓存：`~/.cache/huggingface/hub/models--openvla--openvla-7b`（**15GB**），可 scp 到服务器同路径；
- 或服务器直接下载：`HF_ENDPOINT=https://hf-mirror.com huggingface-cli download openvla/openvla-7b`。

## 3. 服务器 conda 环境

```bash
conda create -n vla_ft python=3.10 -y && conda activate vla_ft

# torch 按服务器 CUDA 选轮子（驱动报 12.x 都可用 cu121）
pip install torch==2.2.0 torchvision==0.17.0 --index-url https://download.pytorch.org/whl/cu121

# 方案 A 最小依赖集（版本与 pyproject 对齐）
pip install "transformers==4.40.1" "peft==0.11.1" "tokenizers==0.19.1" "timm==0.9.10" \
    "sentencepiece==0.1.99" "huggingface_hub<0.23" "accelerate>=0.25.0" "draccus==0.8.0" \
    "einops" "rich" "matplotlib" "jsonlines" "protobuf" "wandb" pillow numpy opencv-python \
    -i https://pypi.tuna.tsinghua.edu.cn/simple
```

**不要跑 `pip install -e .`**：pyproject 会拖 tensorflow / tensorflow_datasets / dlimp
（要访问 GitHub，Plan A 根本不用）。若服务器已有跑 deploy 的 openvla 环境，复用也行，
但版本必须满足：transformers==4.40.1、peft==0.11.1、torch≥2.2、tokenizers==0.19.1、timm==0.9.10。

环境自检：

```bash
cd ~/VLA_ROBOT/openvla
python -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
python -c "import prismatic; from prismatic.vla.gazebo_demos import GazeboDemosDataset; print('imports OK')"
```

## 4. 训练

### 4.1 冒烟测试（50 步，~15 分钟）

```bash
cd ~/VLA_ROBOT/openvla
mkdir -p runs
WANDB_MODE=offline torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
    --vla_path openvla/openvla-7b \
    --data_root_dir datasets --dataset_name gazebo_4room \
    --run_root_dir runs \
    --batch_size 4 --max_steps 50 --save_steps 50 --learning_rate 5e-4 \
    --lora_rank 32 --use_quantization --image_aug
```

预期日志：

- `[GazeboDemos] 4590 个样本 (episode x step)` ← 数字必须是 4590
- `vla.print_trainable_parameters()` 打印 LoRA 可训练参数占比（~1% 量级）
- loss 从个位数下降，`action_accuracy` / `l1_loss` 有数值（wandb offline 目录 `wandb/offline-run-*`）

看到 `Saved Model Checkpoint for Step 50 at: runs/...` 即链路全通。

### 4.2 正式训练（tmux 里跑）

```bash
tmux new -s vlaft
WANDB_MODE=offline torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
    --vla_path openvla/openvla-7b \
    --data_root_dir datasets --dataset_name gazebo_4room \
    --run_root_dir runs \
    --batch_size 4 --max_steps 2000 --save_steps 500 --learning_rate 5e-4 \
    --lora_rank 32 --use_quantization --image_aug
# Ctrl-B D 脱离 tmux；回看: tmux attach -t vlaft
watch -n 5 nvidia-smi
```

参数速查：

| 项 | 建议 | 说明 |
|----|------|------|
| max_steps | 1000–3000 | 4590 步数据，bs4×2000 步 ≈ 1.7 epoch；**action_accuracy > 0.8 就停**，再练易过拟合 |
| batch_size | 24GB 量化:4；40GB 不量化:8–16 | OOM 就降 bs / 开 --use_quantization |
| learning_rate | 5e-4（默认） | 不稳就 2e-4 |
| save_steps | 500 | 默认只保留最新 ckpt 并反复合并 LoRA（每次合并慢，属正常） |
| 时长 | 4590 步小数据：小时级 | 冒烟实测 it/s ×2000 粗估 |

### 4.3 产出

`runs/<exp_id>/`：合并后的完整 bf16 模型（`*.safetensors` ~15GB + `config.json`）
和 **`dataset_statistics.json`**（推理反归一化必需，别漏拷）。

## 5. 训练后部署（两条路）

### 方式 A：拉回本机（WSL 仿真闭环测试）

```bash
# 服务器 → 本机（15GB，内网 rsync 断点续传）
rsync -avP user@server:~/VLA_ROBOT/openvla/runs/<exp_id>/ ~/models/gazebo_ft/

# 本机 vla_env
cd ~/Desktop/VLA_ROBOT/openvla/vla-scripts
VLA_UNNORM_KEY= python3 deploy_4bit.py --openvla_path ~/models/gazebo_ft
```

`VLA_UNNORM_KEY=` 留空 = 自动选 `gazebo_4room`（ckpt 目录里只有这一个 key）。

### 方式 B：推理服务留服务器，本机只跑仿真

```bash
# 服务器（监听所有网卡；用本地权重目录免 HF 下载）
VLA_UNNORM_KEY= python3 deploy_4bit.py --openvla_path ~/.cache/huggingface/hub/.../<snapshot目录> --host 0.0.0.0 --port 8000

# 本机 bridge 指过去
ros2 run base_robot vla_gazebo_bridge.py --ros-args -p server_url:=http://<服务器IP>:8000/act
```

注意防火墙放行 8000；跨网每步传 ~150KB JSON（224×224×3），内网没压力，公网慎走。
服务器跑 deploy_4bit 同样吃 GPU 显存（4bit ~7GB），与训练错峰。

### 触发与验收

```bash
ros2 topic pub /vla_instruction std_msgs/msg/String "data: 'pick up the cup and pour'" -1
ros2 run base_robot vla_grasp_test.py   # 默认指令已同步改为这句话
```

- **发话必须逐字等于训练指令**，改措辞 = 重新转换 + 重训。
- bridge 量纲参数按 `openvla/vla-scripts/GAZEBO_FINETUNE.md` 第 4 节覆盖
  （`position_scale:=1.0 rotation_scale:=1.0 max_linear_step:=0.04 max_angular_step:=0.2`），
  不改的话够不到杯子、倾倒段被削顶。
- 验收口径：demo_scene 中机器人在 16 个停靠位任一位置，能完成 抓起→倾倒→放回 全序列；
  先看单站成功率，再谈泛化。

## 6. 常见坑速查

| 症状 | 原因 / 解法 |
|------|-------------|
| HF 下载卡住 | `HF_ENDPOINT=https://hf-mirror.com`，或直接拷 2.3 的本地缓存 |
| `pip install -e .` 失败/巨慢 | 别装（§3）；按 §3 最小集即可 |
| `ImportError: prismatic.vla.datasets` | `prismatic/vla/__init__.py` 的惰性导入补丁没传上去，检查代码同步 |
| 训练 OOM | `--use_quantization`；降 `--batch_size` 至 2；降 `--lora_rank` 至 16 |
| action_accuracy 恒 ~0 | `dataset_statistics.json` 与数据不是同一批（重跑 convert 后统计变了但传了旧 json）；或 labels 全被截断（看 §7 备注：prompt+7 token 很短，正常不会） |
| 推理动作幅度诡异 | `VLA_UNNORM_KEY` 没设置/设错，加载了 bridge_orig 统计；ckpt 目录里必须带 dataset_statistics.json |
| wandb 报要登录 | 命令前必须 `WANDB_MODE=offline` |
| 合并 ckpt 后目录没 statistics | 只有 step>0 触发过保存才有；确认用了 `--save_steps` 且跑过了步数 |

## 7. 备注

- 归一化按 q01/q99 分位（非 min/max），极端步会被 clip 到 ±1，反归一化回来有毫米级偏差——上游 OpenVLA 同款行为，不影响。
- 训练是全序列监督（prompt 也进 loss，同上游 LoRA finetune）。
- 采集侧两个待办不影响本次训练：`collect_4room_demos.py` 的 `ep_idx=24` 硬编码和 `--out` 默认路径（别人机器的路径）在续采前要改。

## 8. 出发清单

- [ ] 本仓库改动已 commit/push（或 tar 打包）
- [ ] `openvla/datasets/gazebo_4room/`（含 dataset_statistics.json）已 rsync，服务器 `ls` 有 51 个 ep_XXXX
- [ ] openvla-7b 权重已就位（缓存拷贝或镜像下载），`du -sh` ≈ 15G
- [ ] conda env `vla_ft` 建好，§3 两条 import 自检通过
- [ ] 冒烟 50 步跑通：样本数 4590、loss 下降、ckpt 落盘、run 目录带 statistics json
- [ ] tmux 正式训练 2000 步，`nvidia-smi` 显存稳
- [ ] 部署路径二选一，bridge 量纲参数已覆盖
- [ ] 单停靠位闭环验收：抓起→倾倒→放回
