# ROS2 仿真项目环境配置文档

> 假设工作空间根目录为 `$WS`（如 `/home/xxx/VLA_ROBOT`），下文中所有路径以 `$WS` 表示。

---

## 〇、系统前置要求

| 组件 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 22.04 LTS (Jammy) |
| ROS2 | Humble Hawksbill (完整桌面安装) |
| Gazebo | Gazebo Classic 11 (与 ROS2 Humble 配套) |
| Python | 3.10 (系统默认) |
| 编译工具 | colcon、CMake 3.8+、gcc 11+ |

验证环境：
```bash
# 确认 ROS2 环境
echo $ROS_DISTRIB  # 应输出 humble
ros2 --version

# 确认 Gazebo 可用
gz sim --version 2>/dev/null || gazebo --version
```

---

## 一、基础编译与环境激活

### 1.1 首次完整编译

```bash
# 定位到工作空间
cd $WS/base_robot_ws

# 安装 rosdep 依赖（首次使用需先 rosdep init / rosdep update）
rosdep install -i --from-path src --rosdistro humble -y

# 编译工作空间（包含 base_robot、kdl_parser_ros2 和 gazebo_grasp_plugin 三个包）
colcon build --symlink-install

# 激活编译后的环境
source install/setup.bash
```

### 1.2 增量编译

```bash
cd $WS/base_robot_ws

# 只编译某个特定包
colcon build --packages-select base_robot

# 只编译 base_robot 和 gazebo_grasp_plugin
colcon build --packages-select base_robot gazebo_grasp_plugin
```

> **提示**：建议将 `source $WS/base_robot_ws/install/setup.bash` 添加到 `~/.bashrc` 中，避免每次新终端都要手动 source。

---

## 二、机器人启动命令

### 2.1 核心仿真启动（Gazebo + 控制器 + VLA 桥接）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot base_robot.launch.py
```

> **启动顺序链**：robot_state_publisher → Gazebo → joint_state_broadcaster → arm_controller → gripper_controller → vla_gazebo_bridge
>
> 机器人实体由 `test.world` 直接加载（不再通过 `spawn_entity` 生成）。控制器通过 `TimerAction`（延迟 8s，等待 Gazebo 加载完成）触发首个 spawner，后续各环节再通过 `RegisterEventHandler(OnProcessExit(...))` 串行启动。
>
> **关键修复插件**：
> - `libgazebo_grasp_fix.so` — 通过 arm.xacro 及 test.world 中的 `<gazebo>` 标签自动加载。采用 fixed joint 方案根治仿真中夹爪穿模/弹飞/打滑问题。详见 `夹爪抓取问题分析与修复方案.md`。
> - `libgazebo_ros_diff_drive.so` — 差速驱动插件，订阅 `/cmd_vel` 话题控制底盘移动。
> - `libgazebo_ros2_control.so` — ROS2 控制框架桥接，加载 `controller_position.yaml` 中的控制器配置。

### 2.2 SLAM 建图

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot rviz.launch.py
```

> 启动 async_slam_toolbox_node + RViz2，用于建图。需先启动 2.1 核心仿真。
>
> 依赖：`ros-humble-slam-toolbox`

### 2.3 导航功能

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot navigation.launch.py
```

> 启动 Nav2 bringup + RViz2，使用 `maps/room1.yaml` 地图和 `config/my_nav_params.yaml` 参数。
>
> AMCL 初始位姿会由 `set_initial_pose.py` 节点在启动后自动发布（无需在 RViz 手动 2D Pose Estimate）。初始位姿坐标在 `navigation.launch.py` 中配置，需与 `test.world` 中机器人出生点保持一致。
>
> 依赖：`ros-humble-navigation2`、`ros-humble-nav2-bringup`

### 2.4 RViz 模拟控制（无 Gazebo，纯 RViz 调试）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot move.launch.py
```

> 使用 fake_diff_drive 模拟差速驱动，arm_control_gui 提供机械臂 GUI 控制，适合离线调试。
>
> 依赖：`ros-humble-joint-state-publisher-gui`

### 2.5 机械臂 + 底盘控制台（Tkinter GUI）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot simple_commander.py
```

> **新增底盘控制功能**：Tkinter GUI 除 6 轴机械臂 + 双指夹爪独立控制外，新增底盘线速度/角速度滑块，通过 `/cmd_vel` 话题直接控制机器人移动。
>
> 依赖：Python3 `tkinter`（系统自带）
>
> ⚠️ 需在启动 2.1 核心仿真**之前**执行（仅指关节初始化），之后在仿真运行期间可通过 GUI 实时控制。

### 2.6 自主任务（机械臂抓取演示）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot auto_task.py
```

> **新版抓取任务流程**：
> 1. 启动 `gz topic -e` 持久订阅 `/gazebo/default/grasp_events` 话题，监听附着事件（不丢消息）
> 2. 夹爪完全张开 → 机械臂移动到预抓取位姿 1 → 移动到抓取位姿 2
> 3. 夹爪闭合 → 等待附着确认（最多 90s 超时）
> 4. 检测到附着成功 → 机械臂抬升至安全位姿
> 5. 自主导航回原点 `(0, 0, 0)`
>
> 需先启动 2.1 核心仿真与 2.3 导航功能（第 5 步依赖 Nav2 导航栈）。

### 2.7 VLA 轻量化部署

```bash
cd $WS/openvla/vla-scripts
python3 deploy_4bit.py --openvla-path /your/path/openvla
```

> **环境隔离要求**：必须在 mamba/conda 独立环境中运行，避免与 ROS2 的 Python 环境冲突。
>
> **权重下载**：首次运行会自动下载约 3×6GB 权重文件，请确保磁盘空间 ≥ 25GB。
>
> **推理性能**：4bit 量化推理延迟约 2-3 秒/step（GPU CUDA 12.1）。

### 2.8 KDL 运动学测试

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot test_kdl_roundtrip.py
```

> 测试正逆运动学往返精度、模拟 OpenVLA delta action、发布抬升指令到 `/vla_instruction`。
>
> 需先启动 2.1 核心仿真（因为需要 `robot_description` 参数）。

### 2.9 ASR 语音转文本测试
以下内容需要安装dashscope、pyaudio
```bash
pip install pyaudio dashscope
```

#### 录音文件
不启用gazebo仿真时：
启动OpenVLA服务之后（见2.7节），执行以下指令（conda环境）：
```bash
export DASHSCOPE_API_KEY="你的百炼API Key"
cd $WS/asr_test
python3 asr_file_to_openvla_server.py Chinese.mp3
```

实测可以看到以下输出：
```bash
[ASR] audio: Chinese.mp3
[ASR] text: 请夹起前面的瓶子。
[OpenVLA] POST http://127.0.0.1:8000/act
[OpenVLA] dummy image: 256x256, color=(128, 128, 128)
[OpenVLA] status: success
[OpenVLA] action: [0.0006859096428927063, -0.0010716421534033849, 0.001577533126315641, -0.015465142300316037, 0.006587913932169148, -0.006513903635389756, 0.996078431372549]
```
启用gazebo仿真时：
启动OpenVLA服务（见2.7节）、Gazebo/ROS2 仿真之后，执行以下指令（conda环境）：
```bash
export DASHSCOPE_API_KEY="你的百炼API Key"
cd $WS/asr_test
python3 asr_file_to_vla.py Chinese.mp3
```

#### 麦克风流式输入
启动OpenVLA服务之后（见2.7节）、Gazebo/ROS2 仿真之后，执行以下指令（conda环境）：
```bash
export DASHSCOPE_API_KEY="你的百炼API Key"
cd $WS/asr_test
python3 asr_mic_to_vla.py
```

有多个输入设备时。可以执行以下指令：
```bash
python3 asr_mic_to_vla.py --list-devices
```
如果看到麦克风设备，比如 [2] USB Microphone，就运行：
```bash
python3 asr_mic_to_vla.py --input-device-index 2
```

### 2.10 多点自主导航

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot nav_waypoints.py
```

> 依次前往多个地图目标点（`map` 坐标系），用于验证导航栈或执行巡检任务。
>
> 目标点列表在 `nav_waypoints.py` 顶部的 `WAYPOINTS` 列表中修改，每个点格式为 `(x, y, yaw)`；`WAIT_AT_WAYPOINT` 控制到达后停留时间，`NAV_TIMEOUT` 控制单点超时，`STOP_ON_FAILURE` 控制失败时是否停止。
>
> 需先启动 2.1 核心仿真与 2.3 导航功能。

### 2.11 决策 Agent（plan-execute 框架）

```bash
cd $WS/agent_demo
python3 main.py "开始巡检" --mock-planner --print-plan        # mock 计划
python3 main.py "开始巡检" --mock-planner --real-navigation  # 真实导航巡检
```

> 语言输入 → LLM 生成计划 → 执行白名单工具（导航 / 检测 / 操作 / 验证）→ 失败重规划。
> 详细用法、工具说明、巡检 / 颜色识别接口见 `agent_demo/README.md`。

---

## 三、包结构说明

### 3.1 base_robot（主包）

| 目录/文件 | 说明 |
|-----------|------|
| `launch/*.launch.py` | 4 个启动文件（base_robot / rviz / navigation / move） |
| `scripts/` | 9 个 Python 脚本（见下表） |
| `urdf/4_robot/` | 机器人 URDF/Xacro 模型定义 |
| `world/` | Gazebo 世界文件（test.world 等） |
| `config/` | 控制器/Nav2/SLAM/RViz 配置文件 |
| `maps/` | 导航地图（room1.yaml / room1.pgm 等） |
| `meshes/` | 3D 模型网格（STL/DAE） |

**Python 脚本一览**：

| 脚本 | 功能 |
|------|------|
| `simple_commander.py` | Tkinter GUI：6 轴机械臂 + 双指夹爪 + 底盘控制 |
| `auto_task.py` | 自主抓取演示任务（附着检测 + 导航回原点） |
| `nav_waypoints.py` | 多点自主导航（顺序前往多个目标点） |
| `set_initial_pose.py` | AMCL 初始位姿自动发布 |
| `vla_gazebo_bridge.py` | VLA 推理结果 → Gazebo 关节指令桥接 |
| `tts_node.py` | 语音播报节点 |
| `test_kdl_roundtrip.py` | KDL 正逆运动学往返精度测试 |
| `fake_diff_drive.py` | 差速驱动模拟（离线调试） |
| `arm_control_gui.py` | 机械臂 GUI 控制 |

### 3.2 kdl_parser_ros2

KDL 运动学解析器 ROS2 分支。基于 `orocos_kdl` 库，提供：
- URDF → KDL Chain 解析
- 正运动学（FK）：关节角 → 末端位姿
- 逆运动学（IK）：末端位姿 → 关节角
- 雅可比矩阵计算

### 3.3 gazebo_grasp_plugin（新增）

Gazebo Model Plugin，纯 C++ 实现（约 1250 行），不依赖 ROS 通信层。核心逻辑：

1. 监测夹爪手指（link7/link8）对物体施加的对向力
2. 判定为抓取 → 在 palm_link 和物体之间创建 fixed joint
3. 可选禁用夹爪与物体的碰撞
4. 检测到释放条件 → 销毁 fixed joint → 恢复碰撞

**编译依赖**：`libgazebo-dev`（已在 4.1 中包含）

**配置方式**：在 URDF/Xacro 或 SDF 世界文件的 `<gazebo>` 标签中添加 plugin 声明。本项目同时在 `arm.xacro` 和 `test.world` 中加载，确保覆盖 URDF spawn 和 world 直接加载两种方式。

---

## 四、Gazebo 缓存修复

```bash
# 安装 Gazebo（已安装可跳过）
sudo apt install gazebo

# 创建缓存目录并下载模型文件
mkdir -p ~/.gazebo
git clone https://gitee.com/ohhuo/gazebo_models.git ~/.gazebo/models

# 删除多余的 git 配置文件
rm -rf ~/.gazebo/models/.git
```

> 首次启动 Gazebo 时会从远程下载模型文件，国内网络可能很慢或失败。使用 gitee 镜像可解决此问题。

---

## 五、依赖环境安装（完整清单）

### 5.1 系统级依赖（一次性安装）

```bash
# ROS2 Humble 完整安装（如已安装则跳过）
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

# Gazebo Classic + 开发库
sudo apt install gazebo libgazebo-dev

# protobuf（gazebo_grasp_plugin 编译依赖）
sudo apt install libprotobuf-dev protobuf-compiler
```

### 5.2 ROS2 包依赖

```bash
# --- 基础仿真 ---
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher

# --- 控制器 ---
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

# --- SLAM 建图 ---
sudo apt install ros-humble-slam-toolbox

# --- 导航 ---
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y

# --- KDL 运动学 ---
sudo apt install ros-humble-orocos-kdl-vendor
sudo apt install ros-humble-urdf

# --- 通用工具 ---
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-ament-cmake-python
sudo apt install ros-humble-ament-index-cpp
```

### 5.3 一键安装命令

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-orocos-kdl-vendor \
  ros-humble-urdf \
  ros-humble-rviz2 \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-python \
  ros-humble-ament-index-cpp \
  gazebo libgazebo-dev \
  libprotobuf-dev protobuf-compiler
```

### 5.4 Python / pip 依赖（VLA 环境）

```
⚠️ 做好 ROS 本机与 mamba 环境的隔离！
Python 版本：统一 3.10
Torch 版本：2.2.0+cu121（GPU 版）
```

```bash
# VLA Python 依赖（在 mamba/conda 环境中安装）
pip install timm==0.9.10
pip install tokenizers==0.19.1
pip install torch>=2.2.0
pip install torchvision>=0.16.0
pip install transformers==4.40.1
```

> 注意：轻量化执行会从 Hugging Face 云端传输约 3×6GB 权重文件，请提前留好空间（建议 ≥ 25GB 可用空间）。

---

## 六、项目文件结构速览

```
VLA_ROBOT/
├── Launch.md                      # 本文件：环境配置与启动命令文档
├── README.md                      # 版本更新日志
├── 夹爪抓取问题分析与修复方案.md    # 夹爪抓取问题根因分析与 gazebo_grasp_plugin 移植方案
├── .gitignore                     # Git 忽略规则
├── agent_demo/                    # 决策 agent demo（plan-execute 框架）
├── asr_test/                      # ASR 语音转文本测试
├── base_robot_ws/                 # ROS2 工作空间
│   ├── src/
│   │   ├── base_robot/            # 主 ROS2 包
│   │   │   ├── launch/            # 启动文件 (*.launch.py)
│   │   │   ├── scripts/           # Python 脚本
│   │   │   ├── urdf/4_robot/      # 机器人 URDF/Xacro 模型
│   │   │   ├── world/             # Gazebo 世界文件
│   │   │   ├── config/            # RViz / Nav2 / SLAM 配置文件
│   │   │   ├── maps/              # 地图文件 (room1.yaml / room1.pgm)
│   │   │   └── meshes/            # 3D 模型网格 (STL/DAE)
│   │   ├── kdl_parser_ros2/       # KDL 运动学解析器（ROS2 分支）
│   │   └── gazebo_grasp_plugin/   # 夹爪抓取修复插件（fixed joint 方案）
│   ├── build/                     # 编译产物（已 gitignore）
│   ├── install/                   # 安装产物（已 gitignore）
│   └── log/                       # 日志（已 gitignore）
└── openvla/                       # OpenVLA 模型与部署脚本
```

---

## 七、常见问题排查

### 7.1 Gazebo 启动后闪退

```bash
# 检查 Gazebo 模型路径
echo $GAZEBO_MODEL_PATH
ls ~/.gazebo/models/

# 如模型缺失，按第四章步骤下载
```

### 7.2 控制器加载失败

```bash
# 确认 controller_manager 已安装
ros2 pkg list | grep controller_manager

# 检查 controller_position.yaml 路径是否正确
# 包内路径: base_robot/config/controller_position.yaml
```

### 7.3 夹爪抓取失败（物体弹飞/打滑）

确认 `gazebo_grasp_plugin` 已编译并加载：
```bash
# 检查插件库是否存在
ls $WS/base_robot_ws/install/gazebo_grasp_plugin/lib/libgazebo_grasp_fix.so

# 启动 Gazebo 后查看插件加载日志
gz log 2>&1 | grep -i grasp
```

### 7.4 VLA 桥接节点启动失败

```bash
# 确认 vla_gazebo_bridge.py 有执行权限
ls -la $WS/base_robot_ws/src/base_robot/scripts/vla_gazebo_bridge.py

# 如缺少权限
chmod +x $WS/base_robot_ws/src/base_robot/scripts/vla_gazebo_bridge.py
```

### 7.5 colcon build 失败

```bash
# 清理旧的编译产物后重试
cd $WS/base_robot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```
