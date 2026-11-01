# 第八轮验证：gazebo_4room_v3 权重（裁剪静止帧 + 1200 步）

**结论：4 房间演示链路首次完整闭环 —— 抓取 → 提杯 → 倾倒 → 释放回桌，零卡死。**

## 权重与数据谱系

- 权重：训练机产物 `openvla-7b+gazebo_4room_v3+...--v3s1200--image_aug--1200_chkpt`
  → 已归位 `~/models/gazebo_ft/gazebo_4room_v3`（LoRA 已合并，4 分片齐全，自带 dataset_statistics）。
- `gazebo_4room_v3`（训练机数据集名）与本地裁剪后 `datasets/gazebo_4room_v2`（864 帧）**统计逐维一致**，
  即同一份"删净零位移静止帧"的数据；夹爪 q01/q99 = 0.75/1.0。
- 训练指令 `pick up the cup and pour`（部署端须逐字一致）。

## 运行配置

- 服务器：`VLA_UNNORM_KEY= …/openvla_env/bin/python deploy_4bit.py --openvla-path ~/models/gazebo_ft/gazebo_4room_v3`
- 桥接：position_scale 1.0 / rotation_scale 1.0 / max_linear_step 0.04 / max_angular_step 0.2，
  脱困参数全开（escape_gt_file/meta 指向裁剪后 ep_0004，54 帧；escape_stall_frames 4；escape_phase_gap 8；min_ee_z 0.19）
- 站点 southeast_blue_south：机器人 (14.152, -11.868, yaw 90°)，杯子精确归位后开跑。

## 结果时间线（366 秒）

| 时刻 | 事件 |
|---|---|
| t+3s | 首帧即正确复述 GT f0 抬升段 `[0.03,0,0.001,…,1.0]` |
| t+~100s | f8 带 0.75 闭合下探 → 接触 → attach 焊接成功 |
| t+~110s | **杯子离地，z 峰值 0.5089**（桌面 0.4566） |
| t+110s~280s | 倾倒段"摇秤"：±0.174 rad 交替，指针 22→26 爬行；ESCAPE-PHASE 共 16 次兜跳相 |
| t+~396s | 走完 54 帧终点，夹爪张开 1.0 |
| 终态 | **杯子站立回桌 z=0.4571（误差 0.4mm）**，俯仰回正；水平滚 9.5cm、自转 109°（焊接倾倒后正常落点） |

统计对照（vs v2 权重七轮）：

| 指标 | v2·七轮 | v3·八轮 |
|---|---|---|
| ESCAPE-GT（零位移卡死接管） | 每轮必触发 | **0** |
| VLA(stall) | stall 涨至 15 | **0** |
| ESCAPE-PHASE | 频发（指针曾瞬移 57） | 16（窗口匹配兜住，全部自行恢复） |
| 杯子离地 | 从未（推歪 ±13.6°） | 5.2cm + 完整倾倒释放 |

## 遗留问题

1. 倾倒段进度慢：模型爱重复同一俯仰 token，`action_smooth` 又吃掉净转角 → 只影响时长不影响成败；
   候选对策：调大 alpha 或减推理频率，或继续加训练步数。
2. launch 自带桥接抢跑：推理服务器先在线时，launch 自启桥接（无脱困参数/min_ee_z）会立即驱动机械臂。
   试验时先杀自启桥接再起正式桥接，或仿真阶段不挂服务器。
3. `/tmp/bridge_rd_params.yaml`（robot_description 参数文件）随 /tmp 清理丢失：
   重建 = `ros2 topic echo /robot_description --once --field data` 截到首个 `---` 前，
   包成 `/**: ros__parameters: {robot_description: …}`（缺 ros__parameters 层级 rcl 会拒载）。

## 文件

- `bridge8.log` — 桥接全程（接管/交还/钳制/动作逐帧）
- `cup_track8.log` — 杯子位姿 1Hz 轨迹（`时间 x y z R P Y # track8`）
- `deploy_server_v3.log` — 推理服务器（/act 全部 200）
- `gz_sim.log` — 仿真启动日志
