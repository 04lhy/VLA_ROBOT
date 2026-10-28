# VLA_ROBOT — 移动机械臂 ROS2 仿真平台

> 基于 ROS2 Humble + Gazebo Classic 的轮式移动机械臂仿真项目，集成 OpenVLA 视觉-语言-动作模型、自主导航（Nav2）、SLAM 建图（slam_toolbox）及夹爪抓取修复插件。

---

## 📦 快速开始

```bash
# 1. 进入工作空间
cd base_robot_ws

# 2. 安装系统依赖（首次使用）
rosdep install -i --from-path src --rosdistro humble -y

# 3. 编译
colcon build --symlink-install

# 4. 激活环境
source install/setup.bash

# 5. 启动核心仿真
ros2 launch base_robot base_robot.launch.py
```

详细环境配置、启动命令、依赖安装请参考 [Launch.md](Launch.md)。

---

## 🏗️ 项目结构

```
VLA_ROBOT/
├── Launch.md                          # 环境配置与启动命令完整文档
├── README.md                          # 本文件
├── 夹爪抓取问题分析与修复方案.md        # 夹爪抓取修复技术文档
├── 服务器OpenVLA微调部署指南.md         # 方案A 服务器迁移/训练/部署手册
├── vla_demos/                         # 采集原始数据（ep_XXXX npy，git 忽略，rsync 传输）
├── .gitignore
├── agent_demo/                        # 决策 agent demo（plan-execute 框架）
├── asr_test/                          # ASR 语音转文本测试
├── color_yolo/                        # 前置相机红/绿指示牌检测链路（采集/标注/CPU 推理，V1.8.0 新增）
├── base_robot_ws/                     # ROS2 工作空间
│   └── src/
│       ├── base_robot/                # 主 ROS2 包（机械臂 + 底盘 + 仿真）
│       ├── kdl_parser_ros2/           # KDL 正逆运动学解析
│       └── gazebo_grasp_plugin/       # 夹爪抓取修复插件（新增）
└── openvla/                           # OpenVLA 模型部署
```

---

## 🧪 VLA 夹取测试

通过 OpenVLA 驱动机械臂夹取物体，三终端流程：**VLA 服务 → 仿真 → 测试脚本**。

### 终端 1：启动 OpenVLA 服务（mamba/conda 独立环境）

```bash
cd openvla/vla-scripts
mamba activate vla_env          # 独立 Python 环境，避免与 ROS2 冲突
python3 deploy_4bit.py          # 默认 openvla/openvla-7b，监听 http://127.0.0.1:8000/act
# 权重已缓存可离线加载：
# HF_HUB_OFFLINE=1 python3 deploy_4bit.py
```

### 终端 2：启动仿真

```bash
cd base_robot_ws
source install/setup.bash
ros2 launch base_robot base_robot.launch.py
```

### 终端 3：运行 VLA 夹取测试脚本

```bash
cd base_robot_ws
source install/setup.bash
ros2 run base_robot vla_grasp_test.py
```

### vla_grasp_test.py 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--instruction` | `pick up the cup and pour` | 发送给 VLA 的自然语言指令（微调模型须与训练指令逐字一致） |
| `--object` | `plastic_cup_0` | 目标物体名（用 `gz model` 查询位姿判定抬起） |
| `--timeout` | `180` | 等待夹取的超时时间（秒） |

示例：

```bash
# 夹取蓝色塑料杯（默认指令/物体/超时）
ros2 run base_robot vla_grasp_test.py

# 夹取指定杯子，指定指令与超时
ros2 run base_robot vla_grasp_test.py --instruction "pick up the plastic cup" --object plastic_cup_0 --timeout 180
```

> 运行逻辑：脚本记录杯子初始高度 → 向 `/vla_instruction` 发布指令触发 VLA → 用 `gz model` 轮询杯子世界坐标 z 值，抬升超过阈值即判定夹取成功 → 发布 `stay still` 停止机械臂。

VLA 指令也可运行时手动动态切换：

```bash
ros2 topic pub /vla_instruction std_msgs/msg/String "data: 'pick up the plastic cup'" -1
```

---

## 🧪 数据采集抓取（collect_grasp_demos.py）

脚本驱动机械臂完成一次完整的「抓取 → 抬起 → 倾倒 → 放回」，并在每个控制步记录「相机图像 + 末端 7 维增量动作 + 语言指令」，产出 OpenVLA LoRA 微调所需的 `(image, action, instruction)` 三元组。**无需 OpenVLA 服务**。

```bash
cd base_robot_ws
source install/setup.bash
ros2 run base_robot collect_grasp_demos.py              # 默认采集 20 条
ros2 run base_robot collect_grasp_demos.py --episodes 1  # 单条演示
ros2 run base_robot collect_grasp_demos.py --dry-run     # 只打印 FK/规划，不动机械臂
```

动作约定（与 `vla_gazebo_bridge.py` 完全一致）：
`action = [dx, dy, dz, rx, ry, rz, gripper]`，`gripper ∈ [0,1]`（0=闭合，1=张开）。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--episodes` | `20` | 采集条数 |
| `--out` | `~/vla_demos` | 输出目录（每条一个 `ep_XXXX/`：`images.npy` / `actions.npy` / `meta.json`） |
| `--instruction` | `pick up the cup` | 语言指令 |
| `--cup-model` | `plastic_cup_0` | 被抓取杯子模型名（用于定位） |
| `--tilt-angle` | `1.57` | 倾倒时腕部 joint6 旋转角（弧度≈90°，负号反方向） |
| `--grip-open` | `0.03` | 夹紧开度（0.04=8cm 全开，0.03≈6cm 夹住 6.6cm 杯子） |
| `--joint-approach` / `--joint-grasp` / `--joint-lift` | 见脚本 | 三段关节角（approach 仅用于标定打印，不再进入动作序列） |
| `--randomize-cup` | — | 杯子小范围随机平移，增加数据多样性 |
| `--no-record` | — | 不落盘，只演示动作 |

> 动作序列：home（张开）→ 水平前伸套杯 → 闭合夹住 → 抬起 → 倾倒 ~2s → 复原竖直 → 下降放回桌面 → 松开（张到最大 0.04）→ 机械臂退回 home。需配合 `gazebo_grasp_fix` 插件（`release_tolerance` 已调为 0.005）。

---

## 🚀 V1.8.0 | 底盘前置深度相机与红/绿指示牌检测链路（color_yolo）

**发布日期**：2026-10-28
**版本状态**：✅ 正式发布（前置相机 + world 路径烘焙 + color_yolo 数据/训练/部署链路 + agent_demo 颜色分支巡检）

### ✨ 新增功能

- **底盘前置深度相机**（`demo_scene.world` / `test.world`，已提交）：底盘前部新增 `front_camera_sensor`（depth 类型），话题 `/front_camera_sensor/image_raw`，800×600、名义 10Hz 实测 ≈8Hz；真实内参 fx=fy=428.9、cx=400.5、cy=300.5。光学中心离地**实测 0.157m**（= base_footprint→base_link 0.091m + 安装 0.06625m；提交信息里写的 0.176m 有误）
- **color_yolo 红/绿检测数据链路**（顶层新目录）：给决策 agent 提供巡检分支信号——**绿色→继续巡检下一间 / 红色→去桌前抓取**。推理纯走 CPU（显存仅 8G 需与 OpenVLA 共存），0 显存占用
  - `config/markers.yaml`：4 块指示牌 link 位姿/尺寸/颜色真值、4 个巡检停靠位、相机内参来源、base_footprint→base_link 0.091m 偏移
  - `tools/geometry.py` + `tools/selftest_projection.py`：纯 numpy 世界点→相机像素投影数学与离线自检（无 ROS 依赖，可离线复跑）
  - `tools/capture_frames.py`：单帧落盘 PNG + CameraInfo + 相机世界位姿（sha1 去重）
  - `tools/sweep_stations.py`：位姿扫描采集；`--mode teleport|nav2`（默认 teleport）、`--pose-mode measured`（用 `gz model -p` 读回的实测位姿当真值）、`--standoff <m>`（直接指定机器人到牌子水平距离，±0.25m 取三档）、`--dry-run` 离线看计划
  - `tools/annotate.py`：世界真值自动产 YOLO 标签 + `dataset/vis/` 画框自检图 + `data.yaml`；train/val 按「位姿」分组切分，避免同场景泄漏
  - `tools/check_alignment.py`：HSV 检出真实色块 vs 投影框的客观核验（IoU / 中心偏差）
  - `tools/detect_onnx.py`：单帧颜色推理（color 环境跑，无 ROS 依赖），输出 `{"color","detections","infer_ms"}` 单行 JSON，供 agent_demo 子进程调用
  - `model/best.onnx`：部署用 ONNX 模型（`*.onnx`/`runs/` 均在 .gitignore，换机需从 best.pt 重新导出后拷入）
  - **YOLO11n 训练完成**：94 epochs 早停（patience=30），val 全类 mAP50=0.995、mAP50-95=0.995，red/green P≈0.98、R=1.0
  - **接进 agent_demo**：`tools.py` 的 `detect_color` 真实实现（`--real-detection`）——系统 python 抓帧 + color 环境推理两步子进程；`executor.py` 在 red 时自动插入「去 `<room>_table` → 抓取杯子 → 放回杯子」子计划（原 planner TODO 落地）；巡检 mock 计划改为每间房 navigate+detect_color

### 🛠️ 优化 / 改进

- **world 路径机器无关化定案**（`demo_scene.world` / `test.world`，共 19 处上一台机器的 `/home/aunk_lg/...` 硬编码绝对路径）：git 里保留 `../meshes/`、`../config/` 相对写法，由 `CMakeLists.txt` 构建时烘焙成安装目录绝对路径再安装。**Gazebo 里直接用相对路径不可行**——`<parameters>` 不按 world 目录解析（找不到 yaml → gazebo_ros2_control 插件异常、三个控制器全挂、spawner 挂死等待），mesh 相对引用实测触发 gzserver segfault（exit -11）。曾用覆盖 `GAZEBO_MODEL_PATH/RESOURCE_PATH` 的 launch 方案尝试救相对路径，因丢掉了 Gazebo 自带 `/usr/share/gazebo-11` 导致 "Failed to initialize scene" + SIGABRT，已废弃回退，勿再走
- **⚠️ 新增构建规则：编辑 world 文件后必须重新构建** `colcon build --packages-select base_robot --symlink-install`，否则 install 用的还是旧烘焙产物（CMake 已挂 `CMAKE_CONFIGURE_DEPENDS`，改 world 会自动触发重新 configure）
- **指示牌颜色改 2 类红/绿**（配合 2 类检测模型）：西南=红、东南=绿、西北=绿、东北=红（原东南蓝、东北黄取消）；牌子 link 名去颜色后缀，统一 `room_{sw,se,nw,ne}_marker`
- `.gitignore` 新增 `color_yolo/raw/` 忽略（原始采集帧不入库）

### 📌 数据与性能现状（2026-10-28 实测）

| 项 | 值 |
|------|------|
| 已采集 | 4 房 × 5 停靠位 × 2 帧 = **40 帧 / 40 框**（red 20、green 20；其中 16 框被画面边缘切断） |
| 采集距离 | `--standoff 2.4`（预测可见率 93–100%，0 失败） |
| train/val | 34 / 6（`dataset/data.yaml`） |
| 训练 | YOLO11n @imgsz 320，94 epochs 早停，val mAP50 **0.995**（red/green 各 0.995，R=1.0，无串类） |
| ONNX 全量抽查 | conf=0.25 对 40 帧：40/40 匹配、FP=0、FN=0、IoU 中位 0.976（最差 0.958）；部署阈值 conf=0.3 |
| agent 联测 | 真实 detect_color @4 个 2.4m 停靠位：room1 红 0.944 / room2 绿 0.958 / room3 红 0.944 / room4 绿 0.948，**4/4 全对**；单次调用 ≈1.1s（大头是 color 环境进程启动，模型本体 13.3ms/帧） |
| 标注-渲染对齐 | 全部框 IoU 中位 0.947（最小 0.933）、中心偏差中位 3.1px / 最大 4.5px；未切断框（24 个）0.944 / 4.1px；0 个框找不到对应色块 |
| CPU 推理 | YOLO11n ONNX @320 稳态 7.9ms/帧，含预处理端到端 9.7ms/帧，显存 0 |

### ⚠️ 注意事项

- **巡检停靠位已从 1.20m 后退到 2.40m**（2026-10-28 定案落地）：相机仅离地 0.157m，1.2m 处牌子中心仰角 48.1° > 垂直半视场 34.97°，0% 在画面内；距离 ≥1.13m 开始入画、≥1.99m 完整入画，训练帧采自 2.15~2.65m。停靠位三处已同步为 2.4m（`markers.yaml` / `agent_demo/tools.py` LOCATION_COORDS / `nav_waypoints.py` WAYPOINTS），**改任一必须三处一起改**，否则停靠位会漂出训练分布
- `gz model` teleport **不要用 `-p x y z r p Y` 连写**——负坐标会被 getopt 当选项吞掉、报 "Invalid arguments" 且静默不生效；用分轴参数 `-x <x> -y <y> -z <z> -Y <yaw>`（sweep_stations.py 即此写法）
- 采集脚本（`sweep_stations.py` / `capture_frames.py`）必须用**系统 python3 + 已 source ROS 环境**跑，不能在 conda 环境执行；只有训练/推理用 `color` 环境
- 标注真值必须用 teleport 后 `gz model -p` 读回的实测位姿（gz 报的是 base_footprint，换算 base_link 还要补 0.091m）；**禁用 AMCL/里程计 TF**，否则定位误差被写进标签
- 采图所在仿真必须**完整 launch 启动**（含控制器）：只起 gzserver 会让 gazebo_ros2_control 加载失败、机械臂失驱塌下挡住前置相机，整批图报废
- 陈旧产物待处理：已提交的 `urdf/4_robot/4_robot.urdf` 中 mesh 是上一台机器绝对路径（xacro 源用 `$(find base_robot)` 是正确的，此 `.urdf` 为陈旧生成物，暂未处理）
- V1.5.0 条目记载的牌子配色「西南红/东南蓝/东北黄/西北绿」已过时，本版改为红/绿 2 类

---

## 🚀 V1.7.0 | OpenVLA 微调数据链路（方案 A：npy 直连）

**发布日期**：2026-10-26
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **多房间循环采集脚本** (`collect_4room_demos.py`)：4 桌 × 4 边共 16 个抓取停靠位，`gz model` teleport 到位后跑死动作并逐步记录 (图像, 7 维增量动作, 指令)；抬起后检测杯子 z 抬升，**夹取失败的 episode 自动丢弃**
- **数据转换脚本** (`openvla/vla-scripts/convert_demos.py`)：采集目录 → 训练格式——图像重采样 224×224、成功过滤、连续重编号、统计每维 q01/q99 产出 `dataset_statistics.json`（schema 与 OpenVLA `predict_action` 反归一化互逆）；`--set-instruction` 统一训练指令，当前定版 **`pick up the cup and pour`**（覆盖抓取+倾倒全序列）
- **GazeboDemosDataset** (`openvla/prismatic/vla/gazebo_demos.py`)：mmap 直读 npy 的 PyTorch 数据集，样本构造对齐上游 `RLDSBatchTransform`（prompt / labels / 256-bin 动作离散化），**不依赖 RLDS/tensorflow/tfds**
- **文档**：`openvla/vla-scripts/GAZEBO_FINETUNE.md`（转换→训练→部署→bridge 量纲全流程）、`服务器OpenVLA微调部署指南.md`（服务器要求、传输清单、最小 conda 环境、冒烟/正式训练、双路部署、排坑表）

### 🛠️ 优化 / 改进
- `finetune.py`：RLDS 加载块替换为 `GazeboDemosDataset`（shuffle + drop_last + 无限 epoch 循环补上 RLDS 隐式行为），`save_dataset_statistics` 本地实现，ckpt 分支逻辑不变
- `prismatic/vla/__init__.py`：`materialize` 改惰性导入——原 eager import 缺失的 RLDS 模块导致整个 `prismatic.vla` 包无法导入（`finetune.py` 此前直接 ImportError）
- `deploy_4bit.py`：加载本地 ckpt 目录自动读取同目录 `dataset_statistics.json` 填入 `norm_stats`；`unnorm_key` 改由环境变量 `VLA_UNNORM_KEY` 控制（留空自动选唯一 key；不设默认仍 `bridge_orig`，基座模型用法不变）
- `vla_grasp_test.py`：默认指令同步改为 `pick up the cup and pour`（面向微调模型；测基座请显式传老话术）
- `CMakeLists.txt` 注册 `collect_4room_demos.py`；`.gitignore` 忽略 `vla_demos/`

### 📌 数据集现状
| 项 | 值 |
|------|------|
| 规模 | 51 episodes × 90 步 = 4590 样本（16 停靠位全覆盖，每站 2–4 条） |
| 转换产物 | `openvla/datasets/gazebo_4room/`（661MB，git 忽略） |
| 动作统计 (q01/q99) | dx±0.031m · dz±0.026m · rx±0.174rad · ry±0.01rad · gripper{0,1} |
| 归一化验证 | 与 `predict_action` 反归一化全量往返误差 ≈ 0 |

### ⚠️ 注意事项
- 部署微调模型时 `vla_gazebo_bridge.py` 须覆盖量纲参数（`position_scale:=1.0`、`max_angular_step:=0.2` 等，见 GAZEBO_FINETUNE.md 第 4 节），沿用默认会够不到杯子、倾倒被削顶
- 服务器环境**不要 `pip install -e .`**（会拖 tensorflow/tfds/dlimp），按部署指南 §3 最小 pip 集
- `collect_4room_demos.py` 续采前需修：`ep_idx=24` 硬编码（会覆盖已有 ep_0024–0050）与 `--out` 默认值（指向他人机器路径）

---

## 🚀 V1.6.0 | 自动化抓取数据采集与夹爪脱开修复

**发布日期**：2026-10-23
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **自动化抓取数据采集脚本** (`collect_grasp_demos.py`)：脚本驱动机械臂完成「水平前伸抓取 → 抬起 → 倾倒 → 放回桌面 → 退回 home」，同步记录 (图像, 增量动作, 指令) 用于 OpenVLA LoRA 微调，无需 VLA 服务
- **机械臂预置位姿节点** (`vla_preset_pose.py`)：arm_controller 激活后自动归零位，确保腕部相机第一帧就能看到杯子
- **相机视角 RViz 配置** (`config/camera_view.rviz`)

### 🛠️ 优化 / 改进
- `collect_grasp_demos.py` 抓取改为**水平前伸**（不再先抬到杯子上方再压下去），末尾放回杯子后机械臂退回 home
- **夹爪脱开修复** (`arm.xacro` / `demo_scene.world`)：`gazebo_grasp_fix` 的 `release_tolerance` 0.01 → 0.005，修复夹爪张开后杯子仍吸附在夹爪上的问题（手指 0.01m 位移恰好等于阈值，`0.01 > 0.01` 恒不成立）
- `demo_scene.world` 桌子高度下调（台面 0.755 → 0.43），使杯子进入机械臂臂展范围，并新增 4 个杯子模型
- `vla_gazebo_bridge.py` 新增 `server_timeout` 参数（默认 15s）；夹爪映射改为 `[0,1]` 直接映射，修复 `(x+1)/2` 导致夹爪永远半开夹不住的问题
- `vla_grasp_test.py` 默认目标改为新场景 `plastic_cup_0`，超时 120s → 180s
- `base_robot.launch.py` 新增 `vla_preset_pose` 节点，串联到 `load_gripper_controller` 之后、`vla_gazebo_bridge` 之前
- `agent_demo/tools.py` `_navigate_to_pose` 超时 120s → 300s，与 `nav_waypoints.py` 对齐

### 📌 关键变更说明
| 文件 | 变更内容 |
|------|----------|
| `collect_grasp_demos.py` | 新增：抓取+倾倒+放回数据采集脚本 |
| `vla_preset_pose.py` | 新增：机械臂预置位姿节点 |
| `camera_view.rviz` | 新增：相机视角 RViz 配置 |
| `arm.xacro` / `demo_scene.world` | `release_tolerance` 0.01→0.005 修复杯子吸附 |
| `demo_scene.world` | 桌子高度下调 + 新增 4 个杯子 |
| `vla_gazebo_bridge.py` | `server_timeout` + 夹爪映射修复 |
| `vla_grasp_test.py` | 默认目标 plastic_cup_0 + 超时调整 |
| `base_robot.launch.py` | 新增预置位姿节点串联 |

---

## 🚀 V1.5.0 | 更换工厂巡检地图

**发布日期**：2026-10-22
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **新工厂巡检场景**（`world/demo_scene.world`）：4 个房间（西南红/东南蓝/东北黄/西北绿指示牌）+ 4 张桌子 + 4 个杯子，机器人出生在中央走廊 `(0,0,0)`
- **SDF 直接生成导航地图**（`maps/demo_scene.yaml` / `demo_scene.pgm`）：由世界几何（墙/门/桌）栅格化生成，8 个巡检点全部连通可用，无需 SLAM
- **专用 SLAM RViz 配置**（`config/slam.rviz`）：固定坐标系 `map`，含地图/雷达/机器人显示，修复原 `rviz.launch.py` 无配置导致的 RViz 空白

### 🛠️ 优化 / 改进
- `navigation.launch.py` / `set_initial_pose.py` / `my_nav_params.yaml` 的 AMCL 初始位姿对齐新出生点 `(0,0,0)`
- `nav_waypoints.py` / `agent_demo/tools.py` 的巡检坐标表更新为新工厂 8 个巡检点（4 房间指示牌前 + 桌前）
- `nav_waypoints.py` 单点导航超时 `NAV_TIMEOUT` 120s → 300s（新工厂跨房间路径更长，原超时偏紧）

---

## 🚀 V1.4.0 | VLA 夹取测试与相机/夹爪修复

**发布日期**：2026-10-21
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **VLA 夹取测试脚本** (`vla_grasp_test.py`)：通过 `/vla_instruction` 触发 VLA 桥接节点驱动机械臂夹取物体，监听 `grasp_events` 附着事件确认夹取结果，支持 `--instruction` / `--object` / `--timeout` 参数

### 🛠️ 优化 / 改进
- **相机话题修复** (`base_robot.launch.py`)：`vla_gazebo_bridge` 节点新增 `camera_topic` 参数，指向 `test.world` 中腕部摄像头的实际话题 `/camera_sensor/image_raw`（此前误订阅 `/wrist_camera_sensor/image_raw`，导致画面全黑、VLA 无输入、机械臂不动）
- **相机朝向修复** (`test.world`)：腕部相机 sensor pose 由 `yaw=180°`（朝上/朝后拍摄天花板）改为 `pitch=-90°`（沿手指方向朝物体拍摄）
- **夹爪映射修复** (`vla_gazebo_bridge.py`)：VLA 夹爪输出按 `[0,1]`（0=闭合，1=张开）直接映射，修复旧代码 `(x+1)/2` 把 `[0,1]` 误当 `[-1,1]` 处理、导致夹爪永远半开夹不住的问题
- **CMakeLists 注册新脚本**：`vla_grasp_test.py`

### 📌 关键变更说明
| 文件 | 变更内容 |
|------|----------|
| `vla_grasp_test.py` | 新增 VLA 夹取测试脚本 |
| `base_robot.launch.py` | 新增 camera_topic 参数修复相机话题 |
| `test.world` | 修复腕部相机朝向（yaw 180° → pitch -90°） |
| `vla_gazebo_bridge.py` | 修复夹爪 [0,1] 映射 |


---

## 🚀 V1.4.0 | 决策 Agent 与真实导航集成

**发布日期**：2026-10-21
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **决策 agent 框架** (`agent_demo/`)：plan-execute 架构。Planner 通过 DeepSeek/OpenAI 兼容 API 调用 LLM 生成受限 JSON 计划，Executor 执行白名单工具（导航 / 检测 / 操作 / 验证），失败时自动 replan 重规划
- **真实导航集成** (`agent_demo/tools.py`)：`navigate_to` 工具通过 Nav2 `NavigateToPose` action 真实控制小车移动（`--real-navigation` 开启），语义地点名经 `LOCATION_COORDS` 映射到地图坐标
- **固定巡检路线** (`agent_demo/planner.py`)：输入「开始巡检」即依次导航各房间（`PATROL_ROUTE` 定义路线）
- **颜色识别接口（规划中）**：`detect_color` 工具 + 各房间桌子坐标（`<room>_table`），预留摄像头 + YOLO 识别绿色/红色并按颜色分支巡检（绿色→下一间，红色→去桌子抓放杯子→下一间）

### 🛠️ 优化 / 改进
- `agent_demo/README.md` 修正硬编码路径（`/root/openvla_new` → 相对路径）

### 📌 关键变更说明
| 文件 | 变更内容 |
|------|----------|
| `agent_demo/tools.py` | `LOCATION_COORDS` 坐标表 + `navigate_to` 真实导航 + `detect_color` 颜色识别接口 |
| `agent_demo/planner.py` | 固定巡检路线 `PATROL_ROUTE` + 颜色分支巡检注释 |
| `agent_demo/main.py` | 新增 `--real-navigation` 开关 |
| `agent_demo/README.md` | 巡检 / 颜色识别文档 + 路径修正 |

---

## 🚀 V1.3.0 | 多点自主导航与启动健壮性增强

**发布日期**：2026-10-21
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **多点自主导航脚本** (`nav_waypoints.py`)：按顺序依次前往地图上多个目标点（map 坐标系），支持单点超时、失败跳过、到达停留等配置，作为抓取任务与全场景巡检的导航基础
- **AMCL 初始位姿自动发布节点** (`set_initial_pose.py`)：Nav2 Humble 的 AMCL 已不再读取 yaml 里的 `set_initial_pose` / `initial_pose` 参数，新增节点在导航启动后自动向 `/initialpose` 发布位姿，确保定位正确
- **自主任务导航回原点** (`auto_task.py`)：抓取完成后自动调用 Nav2 导航回原点 `(0, 0, 0)`

### 🛠️ 优化 / 改进
- **移除冗余 spawn_entity** (`base_robot.launch.py`)：`test.world` 已内嵌 `base_robot` 模型（含 diff_drive / gazebo_ros2_control / 传感器 / grasp_fix 等插件），移除 `spawn_entity` 后改用 `TimerAction` 定时加载控制器，根治 `Entity already exists` / 服务响应超时导致的控制器加载事件链卡死
- **导航启动自动定位** (`navigation.launch.py`)：挂载初始位姿发布节点，无需再在 RViz 手动 2D Pose Estimate
- **AMCL 初始位姿对齐** (`my_nav_params.yaml`)：`initial_pose` 与机器人出生点 `(2.0279, 1.8948, yaw 1.7867)` 保持一致
- **夹爪张开鲁棒性提升** (`auto_task.py`)：张开指令发送次数 3→5、等待时长增加，降低首条指令丢失概率
- **CMakeLists 注册新脚本**：`nav_waypoints.py` / `set_initial_pose.py`

### 📌 关键变更说明
| 文件 | 变更内容 |
|------|----------|
| `nav_waypoints.py` | 新增多点导航脚本 |
| `set_initial_pose.py` | 新增 AMCL 初始位姿发布节点 |
| `base_robot.launch.py` | 移除 spawn_entity，TimerAction 定时加载控制器 |
| `navigation.launch.py` | 挂载初始位姿发布节点 |
| `auto_task.py` | 新增导航回原点 + 夹爪鲁棒性提升 |
| `my_nav_params.yaml` | AMCL 初始位姿对齐机器人出生点 |

---

## 🚀 V1.2.0 | 夹爪抓取修复与底盘控制增强

**发布日期**：2026-06-03
**版本状态**：✅ 正式发布

### ✨ 新增功能
- **夹爪抓取修复插件** (`gazebo_grasp_plugin`)：移植自 [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)，以 fixed joint 方案根治 Gazebo ODE 仿真中夹爪穿模/弹飞/打滑问题。纯 C++ Model Plugin，源码约 1250 行
- **底盘实时控制**：`simple_commander.py` 新增底盘线速度/角速度 GUI 滑块，通过 `/cmd_vel` 话题直接控制机器人移动
- **附着检测机制**：`auto_task.py` 重构为基于 `gz topic -e` 持久订阅的夹爪抓取附着监听，检测到 `attached: true` 后自动抬升

### 🛠️ 优化 / 改进
- **夹爪物理参数调优**：link7/link8 手指摩擦系数提升至 μ=10.0，阻尼 damping=10，摩擦 friction=5，大幅改善抓取稳定性
- **夹爪关节力限制调整**：joint7/joint8 最大力矩 effort 从 10N 降至 5N，防止大力度造成物体弹飞
- **gazebo_grasp_fix 插件双路径加载**：同时在 `arm.xacro` 和 `test.world` 中配置插件，确保 URDF spawn 和 world 直接加载两种方式均能覆盖
- **测试场景物品物理调优**：coke_can 配置接触刚度 kp=1e+07、摩擦 μ=1.0，beer 使用标准圆柱碰撞体，cafe_table 台面高度 0.755m，配合夹爪修复插件确保抓取演示稳定可靠
- **test.world 清理**：移除仿真运行时保存的 `<state>` 块（运行时状态数据不应纳入版本管理），添加机器人初始位姿
- **Launch.md 全面完善**：新增系统前置要求、包结构说明、增量编译指南、完整依赖清单及一键安装命令、常见问题排查章节
- **.gitignore 完善**：新增 VS Code 数据库缓存文件、Claude Code 工作目录忽略规则

### 📌 关键变更说明
| 文件 | 变更内容 |
|------|----------|
| `arm.xacro` | 夹爪手指高摩擦参数 + 关节阻尼 + gazebo_grasp_fix 插件配置 |
| `test.world` | 清理仿真状态块 + 机器人初始位姿 + gazebo_grasp_fix 插件 + 物品物理参数调优 |
| `auto_task.py` | 重构为附着检测驱动的抓取任务流程 |
| `simple_commander.py` | 新增底盘控制功能（线速度/角速度滑块） |
| `Launch.md` | 全文重写，补充依赖说明、包结构、排查指南 |
| `gazebo_grasp_plugin/` | 新增包：抓取修复插件（纯 C++ Gazebo Model Plugin） |

### 📦 包依赖补充
- `gazebo_grasp_plugin`：Gazebo 夹爪抓取修复（已包含在工作空间中）
- `libgazebo-dev`：Gazebo 开发库（gazebo_grasp_plugin 编译依赖）
- `libprotobuf-dev`、`protobuf-compiler`：protobuf 编译依赖

---

## 🚀 V1.1.0 | VLA 集成版

**发布日期**：2026-05-31
**版本状态**：✅ 正式发布

### ✨ 新增功能
- 集成 **OpenVLA 模型**部署（`deploy_4bit.py`），支持视觉-语言-动作推理
- 新增 **VLA-Gazebo 桥接节点**（`vla_gazebo_bridge.py`），自动随核心仿真启动
- 新增 **自主任务脚本**（`auto_task.py`）：导航到目标点 → 机械臂操作 → 夹爪控制
- 新增 **KDL 运动学解析**（`kdl_parser_ros2` 包），支持正逆运动学计算
- 新增 **KDL 往返测试**（`test_kdl_roundtrip.py`），验证运动学精度
- 新增 **腕部摄像头**模型（`wrist_camera.urdf.xacro`）
- 新增多个 Gazebo 世界文件（`factory.world`、`room_demo.world`）
- 新增 **SLAM 建图启动文件**（`rviz.launch.py`）
- 新增 **地图文件**（`maps/room1.yaml`、`maps/room1.pgm`）

### 🛠️ 优化 / 改进
- `base_robot.launch.py` 启动后自动加载 VLA 桥接节点
- `navigation.launch.py` 改为使用包相对路径，提高可移植性
- `CMakeLists.txt` 补全所有脚本的安装声明
- `Launch.md` 重写为可移植格式，补充所有新命令

---

## 🚀 V1.0.0 | 初始正式版

**发布日期**：2026-03-16
**版本状态**：✅ 正式发布

### ✨ 新增功能
- 集成 **SLAM 建图**功能（slam_toolbox）
- 集成 **Nav2 导航**功能栈，支持自主导航
- 提供一键启动 launch 文件（base_robot / navigation / move）
- 提供机械臂简易控制脚本 `simple_commander.py`，支持机械臂关节控制
- 完成智能车建模，机械臂文件兼容 ROS2 工作空间

### 🛠️ 环境支持
- 适配 **ROS2 Humble** 环境

### 🐛 已知问题
- Gazebo 在高负载下有小概率崩溃
- OpenVLA 4bit 量化推理延迟约 2-3 秒/step

---

## 🔧 环境要求

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Gazebo | Classic 11 |
| Python | 3.10 |
| CUDA | 12.1 (GPU, VLA 推理可选) |
