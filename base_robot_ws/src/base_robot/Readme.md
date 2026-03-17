# ROS2仿真项目环境配置文档
## 一、基础编译与环境激活
```bash
# 定位到工作空间
cd /home/lhy/Desktop/VLA_ROBOT/base_robot_ws

# 编译工作空间
colcon build

# 激活编译后的环境
source install/setup.bash
```

## 二、机器人启动命令
```bash
# 启动机器人核心代码
ros2 launch base_robot base_robot.launch.py

# 导航功能测试
ros2 launch base_robot navigation.launch.py

# 机械臂控制测试（⚠️ 必须在启动机器人代码前执行）
python3 /home/lhy/Desktop/VLA_ROBOT/base_robot_ws/src/base_robot/scripts/simple_commander.py

# RViz 模拟控制（机械臂+底盘，无实际效果）
ros2 launch base_robot move.launch.py
```

## 三、Gazebo 缓存修复命令
```bash
# 安装 Gazebo
sudo apt install gazebo

# 创建缓存目录并下载模型文件
mkdir -p ~/.gazebo
cd ~/.gazebo
git clone https://gitee.com/ohhuo/gazebo_models.git ~/.gazebo/models

# 删除多余的 git 配置文件
rm -rf ~/.gazebo/models/.git
```

## 四、依赖环境安装
### 1. 基础功能依赖
```bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-gazebo-ros2-control
```

### 2. 建图功能依赖
```bash
sudo apt install ros-humble-slam-toolbox
```

### 3. 导航功能依赖
```bash
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
```

### 4. 控制器依赖
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
