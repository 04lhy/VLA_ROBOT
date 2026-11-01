# Gazebo 演示数据 LoRA 微调（方案 A：npy 直连，不走 RLDS/tfds）

数据链路：`collect_4room_demos.py` 采集 → `convert_demos.py` 转换/归一化统计 →
`finetune.py`（`GazeboDemosDataset`）LoRA 训练 → `deploy_4bit.py` 加载合并后 ckpt 推理。

> **v2 数据（2026-10-30 重采，当前使用）**：旧版 `gazebo_4room`（51 条）有 ~40% 零动作
> 静止帧，LoRA 会坍缩成"不动也高分"的 do-nothing 策略。v2 已修复采集脚本（静止帧裁剪到
> ~7%、夹爪改连续值 开度/0.04、ep 编号自动续接），新数据在
> `vla_demos_v2/`（16 停靠位 × 58 步），转换产物 `datasets/gazebo_4room_v2/`。
> **2026-10-31 起 convert_demos.py 默认把残余静止帧全部裁掉**（见 §1），
> 训练集 16 × 54 = 864 帧，零位移样本归 0——残余 ~7% 静止帧（每集 f9/f10、f47/f48）
> 正是闭环"卡死吸引子"的来源，1500 步模型三轮复现中途停走。裁剪前的数据备份在
> `datasets/gazebo_4room_v2_bak_pretrim/`。训练/部署一律用 v2 目录名。

## 1. 转换数据（CPU 即可，系统 python3 + numpy/cv2）

```bash
cd openvla
python3 vla-scripts/convert_demos.py --in ../vla_demos_v2 --out datasets/gazebo_4room_v2
# 默认裁剪零位移静止帧（--no-drop-stall 可关；段末夹爪值自动并入前一运动帧，不丢闭合/张开指令）
# 产物: datasets/gazebo_4room_v2/ep_XXXX/{images.npy (T,224,224,3), actions.npy, meta.json}
#       datasets/gazebo_4room_v2/dataset_statistics.json  (unnorm_key = "gazebo_4room_v2")
# 采集时已用 --instruction 内置指令，无需再 --set-instruction
```

**v4：帧堆叠输入（2026-11-01，治"摇摆段复读"）**

```bash
python3 vla-scripts/convert_demos.py --in ../vla_demos_v2 --out datasets/gazebo_4room_v4 --stack-frames
```

- 病根：腕部相机随 joint6 转，倾倒上行/下行经过**同一腕角=同一画面**，单帧输入在
  物理上无法判别相位（v3 闭环靠 GT 看门狗代驾顶过去）。堆叠后画面差=转动方向。
- 做法：每帧 = [上一帧|当前帧] 横拼 224x448 再 INTER_AREA 压回 224x224（帧 0 两半
  相同），**裁剪静止帧之后**才堆叠。产物形状不变、actions 与 v2 逐元素相同
  （已数值校验），只有 images.npy 不同 → 训练代码/数据集加载零改动。
- 推理端桥接必须同流程：`-p stack_frames:=true`（两端逐像素一致已验证；不一致=白训）。
  桥接内部用相机环形缓冲把配对间隔固定在 0.2s（= 训练间隔），勿改成"上一发送帧"（§4）。
- 换 v4 权重时旧 v3 桥接配置照常，**只加这一个参数**；忘加的表现：模型对着单帧画面
  输出≈训练分布外的乱动作，抓取段就会明显漂。

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

- v2 裁剪后 864 步（bs4 × 600 步 ≈ 2.8 epoch），`--max_steps` 建议 300–800。旧版按
  4590 步调的 2000 会把小数据灌到 8+ epoch，重新背下残余静止模式（盯 W&B 的
  action_accuracy，>0.8 后不必再训；另建议肉眼抽查预测动作非零比例）。
- `--use_quantization` 给显存 <24GB 用（省显存但略伤效果）；24GB 以上可去掉。
- ckpt 落在 `runs/<exp_id>/`，其中自动写入 `dataset_statistics.json`（推理反归一化用）。
- **现役权重**：`--max_steps 1200` 的训练产物（训练机上数据集改名 `gazebo_4room_v3`，内容与
  本地裁剪后 `gazebo_4room_v2` 逐维一致）。第八轮端到端验证结果见 §6 与
  `test_records/2026-11-01_v3_round8/`。
- **下一版（v4，帧堆叠）**：`--data_root_dir datasets --dataset_name gazebo_4room_v4`
  （= 本地 `datasets/gazebo_4room_v4/`，images 为 [上帧|当前帧] 压回 224，其余同 v2/v3，
  见 §1）。训练超参照抄即可；部署端桥接须加 `-p stack_frames:=true`。

## 3. 部署推理

```bash
cd openvla/vla-scripts
# 注意：flag 是 --openvla-path（连字符）；VLA_UNNORM_KEY= 置空自动选 ckpt 内唯一 key
VLA_UNNORM_KEY= ~/miniforge3/envs/openvla_env/bin/python deploy_4bit.py \
    --openvla-path ~/models/gazebo_ft/gazebo_4room_v3
```

- `deploy_4bit.py` 已打补丁：加载本地 ckpt 目录时自动读取同目录 `dataset_statistics.json`。
- ckpt 目录里只有一个 key 时，`VLA_UNNORM_KEY` 可留空（`VLA_UNNORM_KEY=`
  显式置空自动选唯一 key）；**完全不设该变量默认是 `bridge_orig`，key 不存在时每次 /act
  都返回 error、机械臂一步不动**——排查"不动"先查这里。
- 当前在线权重：`~/models/gazebo_ft/gazebo_4room_v3`（旧 v2 保留在同目录，可回滚）。

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

桥接还依赖 `--params-file` 传入 `robot_description`（KDL FK 用）。`/tmp` 会被清理，重建方法：

```bash
source /opt/ros/humble/setup.bash && source $WS/base_robot_ws/install/setup.bash
ros2 topic echo /robot_description --once --field data > /tmp/rd_raw.txt
python3 -c "import yaml; raw=open('/tmp/rd_raw.txt').read(); i=raw.find('\n---\n'); \
s=raw[:i+1] if i!=-1 else raw; assert '<robot' in s; \
yaml.dump({'/**':{'ros__parameters':{'robot_description':s}}}, open('/tmp/bridge_rd_params.yaml','w'))"
```

注意 YAML 必须有 `ros__parameters` 层级，否则 rcl 报 `Cannot have a value before ros__parameters`。

闭环安全/节奏参数（第九轮系列调定，均为主题默认值，桥接命令里可不显式传）：

- `escape_progress_frames=6`：进度看门狗——GT 指针连续 6 步不推进（模型复读同一
  俯仰 token 锁相）就强制续放 1 帧 GT，专治倾倒段"摇秤"；每步只推 1 帧，有硬上限。
- `smooth_agree_steps=2` + `smooth_commit_min_ptr=20` + `smooth_commit_max_ptr=45`：
  自适应平滑窗口——主导位移维连续 2 步同向且 ptr∈[20,45)（已抓牢、未进入放置段）
  才跳过 EMA 全额执行；接近段全额执行会张爪冲撞碰飞杯子（第九轮教训），放置段全额
  执行会高速下摔磕倒杯子（第九b轮教训），两头都必须保持 `action_smooth=0.15` 慢行。
- **终局冻结**：启用 escape_gt 时，`model_ptr` 到 GT 末尾即冻结机械臂并停止推理
  （日志 `EPISODE COMPLETE`）。不加会在末帧后无限重播俯仰 token 空抖手腕（第九b轮尾部）。
- **起测预热位（每轮必做，第十轮教训）**：`vla_preset_pose` 的零位 ≠ 演示起点——
  GT f0→f1 本身含 joint2 0→0.493 翻肘，桥接首帧全额快跳会让手臂带残余下垂，
  闭合指尖压杯沿（零位起手 0/4 抓到）。起桥接前先慢发到位并静置：
  `ros2 topic pub -r 2 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory
  "{joint_names: [joint1..joint6], points: [{positions: [0.0, 0.493, 0.084, 0.001, -0.5, -0.001], time_from_start: {sec: 3}}]}"`
  （+张爪、静置 ≥15s）。world 刚加载先静置 ≥75s 再传送底盘，传送后不再碰底盘。
- **指令 topic 门控（第十轮真根因修复，10l 验证）**：预热位只是必要条件——10k
  带预热仍空抓。真凶是"幽灵步"：旧桥接无门控，就绪即按 `-p instruction` 开跑，
  topic 晚到的 2~11s 里已偷偷跑 ~10 步（夹爪闭合+手臂下压），正式回合第一步
  即闭爪犁杯。现 `vla_gazebo_bridge.py` 默认 `instruction=''`，空指令不动作；
  instruction topic 到达=**全量重置回合状态**后起手（可重复触发新回合）。
  **手动试验起桥接一律勿传 `-p instruction:=…`**，只发 topic 触发
  （launch 自启桥接照常带参数，演示自动流程不变，拦截 pkill 步骤照旧）。
- **夹爪早闭门控 `grip_open_until_ptr`（默认 8，第十m轮教训）**：门控修好后 10m
  仍空抓——动作管线与成功的 10l 逐条相同，但模型从第 1 步就吐 grip 0.75（GT
  f0~f7 本应 1.0 张爪），指尖 0.030 下探蹭到杯壁把杯推离 ~2cm 后抓空。现
  `model_ptr < 8` 时桥接把夹爪**强制张满**，≥8 放行（复刻 GT 在杯位 f8 闭合
  时序）。想改闭合时机调此参数即可，0=关闭门控。**帧门还不完整（10n 又空抓）**：
  放行瞬间指尖可能还在杯沿上方，照样推杯——再叠**闭合深度门控**
  `grip_close_z_margin=0.02`：EE z ≤ GT f8 闭合帧高度+容差（本场景 ≈0.241）
  才放行闭合，放行后锁存不回张；**10p 再暴露"只放行不主动捏"仍空抓**——模型张爪
  指令拖到手沉到 z=0.19 杯底限位才自己闭，闭爪变铲杯推离。故深度达标**当步强制
  压到 0.75（到位即捏，不等模型）**，10q 冷启动全验证通过（§6）。启动日志
  `Grip close depth gate: allow z<=…` 可核对阈值。
- `stack_frames`（默认 false）：**v4 帧堆叠权重专用**，发送 [上帧|当前帧] 压回 224 的
  复合画面，与 convert `--stack-frames` 逐像素同流程；v2/v3 权重务必保持默认 false。
  配对帧由 ~8Hz 相机**环形缓冲**（`deque(maxlen=8)`，每次回调都入队、早于处理闸门）
  里取**时间戳最接近 `t_cur−0.2s`** 的帧——必须与训练配对间隔 0.2s 一致。第十轮教训：
  若 prev 用"上次发送的帧"（间隔≈/act 周期 0.9~1.7s），静止场景两半逐像素相同、
  运动场景 delta 大 5 倍，模型都把爬行段读成接近段→张爪空抓（OOD，非权重问题）。

> **launch 自动桥接抢跑警告**：`base_robot.launch.py` 现在会自动起一个带 pickup 指令的
> `vla_gazebo_bridge`。若推理服务器先于手动桥接在线，自启桥接会立即开始驱动机械臂
> （且无 min_ee_z/脱困参数）。手动试验前的顺序：起仿真 → **杀掉自动桥接**（单独一条
> `pkill -f 'vla_gazebo_bridge[.]py'`，命令内不要同时出现裸文件名，否则自匹配误杀）→
> 传送/归零 → 起正式桥接。

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

## 6. 端到端验证记录

**第八轮（2026-11-01，v3 权重 = 裁剪数据 × 1200 步）：首次完整闭环。**
抓取 → 提杯 5.2cm → 36° 俯仰倾倒（attach 焊接全程持杯）→ 张爪释放，杯子站立回桌（z 误差 0.4mm）。
零位移卡死（ESCAPE-GT / VLA(stall)）**全程 0 次**——静止帧随数据删除后，七轮 v2 权重反复复现的
"背 f9/f10 停顿吸引子"彻底消失，证实裁剪治本。遗留：倾倒段模型爱重复同一俯仰 token、
平滑项吃净角，走终点慢（只影响时长）。完整日志与时间线：
`$WS/test_records/2026-11-01_v3_round8/README.md`。

脱困桥接（escape_gt/phase-gap/min_ee_z）本轮仍全量开启，作为安全网保留：
ESCAPE-PHASE 拦了 16 次跳相均自行交还；min_ee_z=0.19 触发 7 次，夹爪未蹭桌。

**第九c轮（2026-11-01，看门狗+平滑窗口+终局冻结）：节奏问题一并解决。**
桥接侧三项改动（见 §4）把全程从 366s 压到 **105s**：抓取提杯（净空 9.9cm）→
倾倒 → EMA 轻放，杯子站立回桌（z 误差 +1.1mm），GT 放完自动 EPISODE COMPLETE 冻结。
中间两次翻车（第九轮接近段冲撞碰飞杯、第九b轮放置段摔杯+尾部空抖）与三轮对比
详见 `test_records/2026-11-01_v3_round9/README.md`。

**第十c轮（2026-11-01，v4 帧堆叠权重 + 环形缓冲配对）：复读显著缓解，全程 57s。**
v4 首测（轮10）空抓——根因不是权重，是推理配对间隔 0.9~1.7s ≠ 训练 0.2s 的 OOD
（离线用数据集帧喂服务器输出≈GT，见 `test_records/2026-11-01_v4_round10/README.md`）。
桥接加相机环形缓冲按时间戳取 0.2s 前帧后（轮10c）：抓取提杯 +6.05cm → 倾倒 35° →
落杯站立（z 误差 +0.35mm、倾角≈1°，历轮最佳）。俯仰复读从 v3 的**双向**摇摆
（+9/+8/−8/−3 连读）变为单向持续一段（−10），ESCAPE-PROGRESS 看门狗代驾 3→2 次，
全程零 stall、零 ESCAPE-GT。现役权重 v4；回滚 = `~/models/gazebo_ft/gazebo_4room_v3`
+ 去掉 `stack_frames` 参数。

**第十k/l 轮（2026-11-01，幽灵步→topic 门控）：v4 抓取根因定案。**10d/e/g/i 零位起手
全碰杯；10j 预热位起手成功，一度归因"预热位修复（2/2）"——10k **带预热位仍空抓**
证伪。日志铁证：10k 首个可见动作 joint2 已从预热 0.493 掉到 0.993、夹爪 0.030，
即桥接就绪到 topic 到达间的"幽灵步"提前闭合+下压。修复=桥接 instruction 默认空 +
topic 全量重置起手（§4，已重建）。**10l 修复后首测真抓**：提升 +5.6cm → 倾倒 →
张爪释放，EPISODE COMPLETE 零代驾；落杯斜立 18°/滑 7cm（放杯相残留倾角，属模型
行为，遗留打磨点）。v4 抓取统计：正确协议（预热+门控）1/1；旧流程 2/7。详见
`test_records/2026-11-01_v4_round10/README.md`。

**第十m轮（2026-11-01，夹爪早闭门控）**：门控修好后 10m 又空抓——动作管线与 10l
逐条相同（raw/chosen/ptr/角度全对应），纯接触混沌定胜负：模型从第 1 步吐 grip 0.75
（GT f0~f7 应张爪 1.0），指尖 0.030 下探蹭杯壁把杯推离 ~2cm（10l 恰好跨正抓中）。
桥接加 `grip_open_until_ptr=8`（§4）：ptr<8 强制张满、≥8 放行，复刻 GT f8 杯位闭合
时序。

**第十o轮（2026-11-01，闭合深度门控）：v4 最干净一轮。**10n 帧数门控下仍空抓——
放行闭合时指尖还在杯沿上方（用户实况复盘"没夹到杯子中间就闭合、杯子被往前推"）。
补深度门控（`grip_close_z_margin`，§4）：EE z ≤ GT f8 闭合高度+2cm 才允许闭爪。
10o 结果：提升 **+5.5cm** → 倾倒俯仰 49.7° → 张爪释放，落杯**站立**（z +0.5mm、
倾角 1.5°、位移仅 1.5cm），57 步全程 ESCAPE-GT/stall/Traceback=0、gz 存活。
**当前 v4 部署配方三件套**：①`stack_frames:=true`+环形缓冲；②预热位起手+topic
门控（勿传 `-p instruction`）；③夹爪三级门控=`grip_open_until_ptr=8`（帧）+
`grip_close_z_margin=0.02`（深度）+到位即捏。完整复现步骤见
`test_records/2026-11-01_v4_round10/README.md`。

**第十p/q轮（2026-11-01，到位即捏 + Gazebo 完整冷重启验证）**：10p 换全新实例，
两级门控下闭爪仍发生在 z=0.19 杯底限位之后（模型拖后自己才说闭）→ 一闭爪推杯
+2cm 空抓；补"深度达标当步强制压 0.75"。10q 走完整冷重启流程（清场→launch→
拦截自启桥接→静置 75s→归位双测→预热→门控起手）：**提升 +5.39cm → 倾倒 46.2° →
落回站立（z +0.4mm、y 差 2.7mm、倾角 0.8°）**，57 步零 ESCAPE-GT/stall/Traceback，
底盘反冲仅 7mm（历轮最小，杯子扰动最小所致）。抓取统计（正确流程=预热+topic 门控）：
10l~10q 六轮 3/6（10l/10o/10q 成；10m/10n/10p 皆败于其后逐级修复的早闭路径），
最终三级门控代码形态 1/1（10q）——样本少，以"逐级修因、每因一证"为准。
