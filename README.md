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
├── .gitignore
├── agent_demo/                        # 决策 agent demo（plan-execute 框架）
├── asr_test/                          # ASR 语音转文本测试
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
| `--instruction` | `pick up the coke can` | 发送给 VLA 的自然语言指令 |
| `--object` | `coke_can` | 目标物体名（用于匹配 `grasp_events` 附着事件） |
| `--timeout` | `120` | 等待夹取的超时时间（秒） |

示例：

```bash
# 夹取可乐瓶（默认指令/物体/超时）
ros2 run base_robot vla_grasp_test.py

# 夹取啤酒瓶，指定指令与超时
ros2 run base_robot vla_grasp_test.py --instruction "pick up the beer" --object beer --timeout 90
```

> 运行逻辑：脚本先持久订阅 `/gazebo/default/grasp_events` 监听附着事件 → 向 `/vla_instruction` 发布指令触发 VLA → 检测到 `attached: true` 判定夹取成功 → 发布 `stay still` 停止机械臂。

VLA 指令也可运行时手动动态切换：

```bash
ros2 topic pub /vla_instruction std_msgs/msg/String "data: 'pick up the coke can'" -1
```

---

---

## 🚀 V1.5.0 | 更换工厂巡检地图

**发布日期**：2026-10-22
**版本状态**：✅ 当前版本

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
