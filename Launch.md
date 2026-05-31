# ROS2 仿真项目环境配置文档

> 假设工作空间根目录为 `$WS`（如 `/home/xxx/VLA_ROBOT`），下文中所有路径以 `$WS` 表示。

## 一、基础编译与环境激活

```bash
# 定位到工作空间
cd $WS/base_robot_ws

# 编译工作空间（包含 base_robot 和 kdl_parser_ros2 两个包）
colcon build

# 激活编译后的环境
source install/setup.bash
```

---

## 二、机器人启动命令

### 2.1 核心仿真启动（Gazebo + 控制器 + VLA桥接）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot base_robot.launch.py
```

> 自动依次启动：robot_state_publisher → Gazebo → 生成实体 → joint_state_broadcaster → arm_controller → gripper_controller → vla_gazebo_bridge

### 2.2 SLAM 建图

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot rviz.launch.py
```

> 启动 async_slam_toolbox_node + RViz2，用于建图。需先启动 2.1 核心仿真。

### 2.3 导航功能

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot navigation.launch.py
```

> 启动 Nav2 bringup + RViz2，使用 `maps/room1.yaml` 地图和 `config/my_nav_params.yaml` 参数。

### 2.4 RViz 模拟控制（无 Gazebo，纯 RViz 调试）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 launch base_robot move.launch.py
```

> 使用 fake_diff_drive 模拟差速驱动，arm_control_gui 提供机械臂 GUI 控制，适合离线调试。

### 2.5 机械臂关节控制

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot simple_commander.py
```

> ⚠️ 需在启动 2.1 核心仿真**之前**执行，用于初始化机械臂关节状态。

### 2.6 自主任务（导航 + 机械臂 + 夹爪）

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot auto_task.py
```

> 需先启动 2.1 核心仿真 + 2.3 导航功能。任务流程：获取当前位姿 → 导航到目标点 → 机械臂移动到指定姿态 → 夹爪全开。

### 2.7 VLA 轻量化部署

```bash
cd $WS/openvla/vla-scripts
python3 deploy_4bit.py
```

> 需在 mamba/conda 环境中运行，会自动下载约 3×6GB 权重文件。

### 2.8 KDL 运动学测试

```bash
cd $WS/base_robot_ws
source install/setup.bash
ros2 run base_robot test_kdl_roundtrip.py
```

> 测试正逆运动学往返精度、模拟 OpenVLA delta action、发布抬升指令到 `/vla_instruction`。需先启动 2.1 核心仿真（需要 `robot_description` 参数）。

---

## 三、Gazebo 缓存修复

```bash
# 安装 Gazebo
sudo apt install gazebo

# 创建缓存目录并下载模型文件
mkdir -p ~/.gazebo
git clone https://gitee.com/ohhuo/gazebo_models.git ~/.gazebo/models

# 删除多余的 git 配置文件
rm -rf ~/.gazebo/models/.git
```

---

## 四、依赖环境安装

### 4.1 基础功能依赖
```bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-gazebo-ros2-control
```

### 4.2 建图功能依赖
```bash
sudo apt install ros-humble-slam-toolbox
```

### 4.3 导航功能依赖
```bash
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
```

### 4.4 控制器依赖
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### 4.5 KDL 运动学依赖
```bash
sudo apt install ros-humble-orocos-kdl-vendor
sudo apt install ros-humble-urdf
```

### 4.6 mamba 环境配置
```
⚠️ 做好 ROS 本机与 mamba 环境的隔离！
Python 版本：统一 3.10
Torch 版本：2.2.0+cu121（GPU 版）
```

### 4.7 VLA 环境配置
```bash
# 丢给 AI 安装即可，注意轻量化执行会从云端传输约 3×6GB 权重文件，请提前留好空间
timm==0.9.10
tokenizers==0.19.1
torch>=2.2.0
torchvision>=0.16.0
transformers==4.40.1
```

---

## 五、项目文件结构速览

```
VLA_ROBOT/
├── base_robot_ws/src/
│   ├── base_robot/               # 主 ROS2 包
│   │   ├── launch/               # 启动文件 (*.launch.py)
│   │   ├── scripts/              # Python 脚本
│   │   ├── urdf/4_robot/         # 机器人 URDF/Xacro 模型
│   │   ├── world/                # Gazebo 世界文件
│   │   ├── config/               # RViz / Nav2 / SLAM 配置文件
│   │   └── maps/                 # 地图文件 (room1.yaml / room1.pgm)
│   └── kdl_parser_ros2/          # KDL 运动学解析器（ROS2 分支）
└── openvla/                      # OpenVLA 模型与部署脚本
```
