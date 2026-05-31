# 项目版本更新日志
---
## 📋 版本格式说明
- `✨` 新增功能 / 特性
- `🛠️` 优化 / 改进
- `🐛` 修复问题
- `📌` 重要说明
- `⚠️` 注意事项
- `✅` 完成 / 可用
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

### 📌 核心命令
```bash
参照 Launch.md 文件
```

### 🐛 已知问题
- `simple_commander.py` 暂不支持连续发布机械臂关节状态
- Gazebo 在高负载下有小概率崩溃
- OpenVLA 4bit 量化推理延迟约 2-3 秒/step

### 📦 包依赖补充
- `kdl_parser_ros2`：URDF → KDL 运动学链解析（已包含在工作空间中）
- `ros-humble-orocos-kdl-vendor`：KDL 运动学库
---

## 🚀 V1.0.0 | 初始正式版
**发布日期**：2026-03-16
**版本状态**：✅ 正式发布 / 功能待完善

### ✨ 新增功能
- 集成 **SLAM 建图** 功能（slam_toolbox）
- 集成 **Nav2 导航** 功能栈，支持导航功能
- 提供一键启动 launch 文件（base_robot / navigation / move）
- 提供机械臂简易控制脚本 `simple_commander.py`，支持机械臂关节控制
- 完成智能车简单建模，成功将机械臂文件兼容工作空间格式

### 🛠️ 环境支持
- 适配 **ROS2 Humble** 环境

### 🐛 已知问题
- `simple_commander.py` 暂不支持连续发布机械臂关节状态
- Gazebo 环境在高负载下有小概率崩溃
---
