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
├── base_robot_ws/                     # ROS2 工作空间
│   └── src/
│       ├── base_robot/                # 主 ROS2 包（机械臂 + 底盘 + 仿真）
│       ├── kdl_parser_ros2/           # KDL 正逆运动学解析
│       └── gazebo_grasp_plugin/       # 夹爪抓取修复插件（新增）
└── openvla/                           # OpenVLA 模型部署
```

---

## 🚀 V1.2.0 | 夹爪抓取修复与底盘控制增强

**发布日期**：2026-06-03
**版本状态**：✅ 当前版本

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
