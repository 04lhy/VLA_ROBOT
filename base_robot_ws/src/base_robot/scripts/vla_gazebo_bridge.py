#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from cv_bridge import CvBridge

import PyKDL
from urdf_parser_py import urdf
from kdl_parser_py.urdf import treeFromString

import numpy as np
import requests
import json_numpy
from transforms3d.axangles import axangle2mat, mat2axangle

json_numpy.patch()


class KDLKinematics:
    """封装 KDL 正/逆运动学求解"""

    def __init__(self, urdf_string, arm_joint_names, base_link="arm_base_link", tip_link="link6", node=None):
        self.node = node  # 用于日志
        # 从 URDF 构建 KDL 树
        self.tree = self._tree_from_string(urdf_string)
        self.chain = self.tree.getChain(base_link, tip_link)

        self.num_joints = self.chain.getNrOfJoints()
        self.joint_names = arm_joint_names
        
        # 验证关节数量匹配
        # 注意：KDL chain 可能只包含移动关节，不包括固定关节
        if self.num_joints != len(arm_joint_names):
            if self.node:
                self.node.get_logger().warn(
                    f"Joint count mismatch: chain has {self.num_joints}, "
                    f"provided {len(arm_joint_names)}. Using chain joints count."
                )
            # 使用 chain 的实际关节数
            self.num_joints = min(self.num_joints, len(arm_joint_names))

        # FK 与 IK 求解器
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver_pos = PyKDL.ChainIkSolverPos_NR(
            self.chain, self.fk_solver, self.ik_solver_vel,
            maxiter=500, eps=1e-5
        )
        
        # 备选 LMA 求解器
        self.ik_solver_lma = PyKDL.ChainIkSolverPos_LMA(self.chain)

        if self.node:
            self.node.get_logger().info(
                f"KDL initialized: {self.num_joints} joints, "
                f"chain from {base_link} to {tip_link}"
            )

    def _tree_from_string(self, urdf_string):
        """从 URDF 字符串创建 KDL Tree（使用 kdl_parser_py）"""
        ok, tree = treeFromString(urdf_string)
        if not ok:
            raise RuntimeError("Failed to parse URDF into KDL tree")
        return tree

    def get_logger(self):
        import logging
        return logging.getLogger("KDLKinematics")

    def forward_kinematics(self, joint_angles):
        """
        正运动学：由关节角度 -> 末端位姿
        joint_angles: list of 6 float (弧度)
        return: (position [x,y,z], rotation_matrix [3x3])
        """
        kdl_array = PyKDL.JntArray(self.num_joints)
        for i, ang in enumerate(joint_angles[:self.num_joints]):
            kdl_array[i] = ang

        end_frame = PyKDL.Frame()
        self.fk_solver.JntToCart(kdl_array, end_frame)

        pos = end_frame.p
        rot = end_frame.M

        position = np.array([pos.x(), pos.y(), pos.z()])
        rotation = np.array([
            [rot[0, 0], rot[0, 1], rot[0, 2]],
            [rot[1, 0], rot[1, 1], rot[1, 2]],
            [rot[2, 0], rot[2, 1], rot[2, 2]],
        ])
        return position, rotation

    def inverse_kinematics(self, target_pos, target_rot, initial_angles):
        """
        逆运动学：由目标位姿 -> 关节角度
        target_pos: [x, y, z]
        target_rot: 3x3 旋转矩阵
        initial_angles: list of 6 float (当前关节角度，作为 IK 初始猜测)
        return: list of 6 float (弧度) 或 None（无解时）
        """
        kdl_initial = PyKDL.JntArray(self.num_joints)
        for i, ang in enumerate(initial_angles[:self.num_joints]):
            kdl_initial[i] = ang

        target_frame = PyKDL.Frame(
            PyKDL.Rotation(
                target_rot[0, 0], target_rot[0, 1], target_rot[0, 2],
                target_rot[1, 0], target_rot[1, 1], target_rot[1, 2],
                target_rot[2, 0], target_rot[2, 1], target_rot[2, 2],
            ),
            PyKDL.Vector(target_pos[0], target_pos[1], target_pos[2])
        )

        result = PyKDL.JntArray(self.num_joints)
        
        # 尝试 NR 求解器
        if self.ik_solver_pos.CartToJnt(kdl_initial, target_frame, result) >= 0:
            angles = [result[i] for i in range(self.num_joints)]
            # 补齐到 6 个关节
            while len(angles) < len(initial_angles):
                angles.append(initial_angles[len(angles)])
            return angles
        
        # 尝试 LMA 求解器
        if self.ik_solver_lma.CartToJnt(kdl_initial, target_frame, result) >= 0:
            angles = [result[i] for i in range(self.num_joints)]
            while len(angles) < len(initial_angles):
                angles.append(initial_angles[len(angles)])
            return angles
        
        return None


class VLAGazeboBridge(Node):
    def _init_kinematics(self, urdf_string):
        """初始化 KDL 运动学"""
        try:
            self.kinematics = KDLKinematics(
                urdf_string,
                self.arm_joint_names,
                base_link="arm_base_link",
                tip_link="link6",
                node=self
            )
            self.get_logger().info("KDL kinematics initialized successfully")
        except Exception as e:
            self.get_logger().error(f"KDL init failed: {e}")
            import traceback
            traceback.print_exc()

    def __init__(self):
        super().__init__('vla_gazebo_bridge')
        self.bridge = CvBridge()

        # 声明参数
        self.declare_parameter('instruction', 'stay still')
        self.declare_parameter('camera_topic', '/wrist_camera_sensor/image_raw')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/act')
        self.declare_parameter('server_timeout', 15.0)  # 推理延迟 2-3s/step，超时需留足余量
        self.declare_parameter('position_scale', 0.5)
        self.declare_parameter('rotation_scale', 0.5)
        self.declare_parameter('max_linear_step', 0.05)  # 最大线位移步长（米）
        self.declare_parameter('max_angular_step', 0.1)   # 最大角位移步长（弧度）
        self.declare_parameter('trajectory_time', 1.0)    # 轨迹执行时间（秒）
        self.declare_parameter('max_joint_delta', 0.5)    # 单步关节最大变化量（弧度）
        self.declare_parameter('action_smooth', 0.15)     # VLA 模式平滑系数（0=不平滑，1=极慢）
        self.declare_parameter('robot_description', '')

        # 获取参数
        self.instruction = self.get_parameter('instruction').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.server_timeout = self.get_parameter('server_timeout').get_parameter_value().double_value
        self.position_scale = self.get_parameter('position_scale').get_parameter_value().double_value
        self.rotation_scale = self.get_parameter('rotation_scale').get_parameter_value().double_value
        self.max_linear_step = self.get_parameter('max_linear_step').get_parameter_value().double_value
        self.max_angular_step = self.get_parameter('max_angular_step').get_parameter_value().double_value
        self.trajectory_time = self.get_parameter('trajectory_time').get_parameter_value().double_value
        self.max_joint_delta = self.get_parameter('max_joint_delta').get_parameter_value().double_value
        self.action_smooth = self.get_parameter('action_smooth').get_parameter_value().double_value

        # VLA 动作平滑（指数移动平均）
        self.last_action = None      # VLA 输出 EMA 平滑缓存

        # 机械臂关节名称
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joint_names = ['joint7', 'joint8']

        # 当前关节状态
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.joint_states_ready = False

        # 从 parameter 初始化 KDL（不依赖 topic）
        self.kinematics = None
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
        if urdf_string:
            self.get_logger().info("Initializing KDL from robot_description parameter...")
            self._init_kinematics(urdf_string)
        else:
            self.get_logger().warn(
                "robot_description parameter is empty. Pass it in the launch file."
            )

        # 订阅者
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 动态切换 VLA 指令的 topic
        self.instruction_sub = self.create_subscription(
            String,
            '/vla_instruction',
            self.instruction_callback,
            10
        )

        # 发布者
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.processing = False

    def joint_state_callback(self, msg):
        """缓存最新的关节状态"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]
            if len(msg.velocity) > i:
                self.current_joint_velocities[name] = msg.velocity[i]
        
        # 检查是否所有需要的关节都有了
        all_joints = self.arm_joint_names + self.gripper_joint_names
        if not self.joint_states_ready:
            ready = all(name in self.current_joint_positions for name in all_joints)
            if ready:
                self.joint_states_ready = True
                self.get_logger().info("All joint states received")

    def instruction_callback(self, msg):
        """运行时通过 topic 动态切换 VLA 指令"""
        new_instruction = msg.data.strip()
        if new_instruction and new_instruction != self.instruction:
            self.get_logger().info(f"VLA instruction changed: '{self.instruction}' -> '{new_instruction}'")
            self.instruction = new_instruction
            self.last_action = None  # 切指令时重置平滑状态

    def get_current_arm_angles(self):
        """获取当前 6 个关节角度（弧度）"""
        angles = []
        for name in self.arm_joint_names:
            if name in self.current_joint_positions:
                angles.append(self.current_joint_positions[name])
            else:
                return None
        return angles

    def image_callback(self, msg):
        """处理图像并调用 VLA 服务"""
        if self.processing:
            return
        
        if self.kinematics is None:
            self.get_logger().debug("Kinematics not ready yet")
            return
        
        if not self.joint_states_ready:
            self.get_logger().debug("Joint states not ready yet")
            return

        self.processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            payload = {
                "image": cv_image.tolist(),  # numpy 数组需要转换为 list
                "instruction": self.instruction
            }

            self.get_logger().info("Sending request to VLA server...")
            response = requests.post(self.server_url, json=payload, timeout=self.server_timeout)
            
            if response.status_code == 200:
                result = response.json()
                raw_action = np.array(result["action"])

                # EMA 平滑：抑制 VLA 输出的高频抖动
                if self.last_action is not None:
                    alpha = self.action_smooth
                    smoothed = alpha * raw_action[:7] + (1.0 - alpha) * self.last_action
                else:
                    smoothed = raw_action[:7]
                self.last_action = smoothed.copy()

                self.get_logger().info(
                    f"VLA action raw={np.round(raw_action, 3)}, "
                    f"smooth={np.round(smoothed, 3)}"
                )
                self.publish_ik_action(smoothed)
            else:
                self.get_logger().error(f"VLA server error: {response.status_code}")

        except requests.exceptions.Timeout:
            self.get_logger().error("VLA server timeout")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Cannot connect to VLA server")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.processing = False

    def publish_ik_action(self, action_array):
        """
        OpenVLA action (7-DoF end-effector delta):
          [0:3] = 位置增量 (dx, dy, dz)
          [3:6] = 姿态增量 (轴角: rx, ry, rz)
          [6]   = 夹爪 (通常在 [0,1] 或 [-1,1] 范围)
        """
        # Step 1: 获取当前关节角度
        current_angles = self.get_current_arm_angles()
        if current_angles is None:
            self.get_logger().error("Cannot get current joint angles!")
            return

        # Step 2: FK 计算当前末端位姿
        try:
            current_pos, current_rot = self.kinematics.forward_kinematics(current_angles)
        except Exception as e:
            self.get_logger().error(f"FK calculation failed: {e}")
            return

        # Step 3: 将 action 增量叠加到当前位姿得到目标位姿
        # 位置增量（应用缩放和限制）
        delta_pos = action_array[0:3] * self.position_scale
        
        # 限制最大步长
        if np.linalg.norm(delta_pos) > self.max_linear_step:
            delta_pos = delta_pos / np.linalg.norm(delta_pos) * self.max_linear_step
        
        target_pos = current_pos + delta_pos

        # 姿态增量：轴角 -> 旋转矩阵 -> 叠加
        delta_axis_angle = action_array[3:6] * self.rotation_scale
        
        # 限制最大角步长
        delta_angle_norm = np.linalg.norm(delta_axis_angle)
        if delta_angle_norm > self.max_angular_step:
            delta_axis_angle = delta_axis_angle / delta_angle_norm * self.max_angular_step
            delta_angle_norm = self.max_angular_step
        
        if delta_angle_norm > 1e-6:
            delta_rot_mat = axangle2mat(
                delta_axis_angle / delta_angle_norm,
                delta_angle_norm
            )
            target_rot = delta_rot_mat @ current_rot
        else:
            target_rot = current_rot

        # Step 4: IK 求解目标关节角度
        ik_angles = self.kinematics.inverse_kinematics(
            target_pos, target_rot, current_angles
        )

        if ik_angles is None:
            self.get_logger().error("IK solver failed to find solution!")
            self.get_logger().info(f"Target position: {target_pos}")
            self.get_logger().info(f"Current position: {current_pos}")
            return

        # 关节级限幅：单步变化不超过 max_joint_delta，防止突变掀翻小车
        target_angles = []
        for i, (cur, tgt) in enumerate(zip(current_angles, ik_angles)):
            delta = tgt - cur
            if abs(delta) > self.max_joint_delta:
                delta = max(-self.max_joint_delta, min(self.max_joint_delta, delta))
            target_angles.append(cur + delta)

        # Step 5: 夹爪控制
        gripper_cmd = float(action_array[6])
        # bridge_orig 归一化下夹爪输出为 [0,1]：0=闭合，1=张开。
        # 直接映射到 joint7/joint8 开度 [0.0 (闭合), 0.04 (张开)]，
        # 不要做 (x+1)/2 变换——那会把 [0,1] 压缩到 [0.5,1]，夹爪永远半开夹不住。
        gripper_cmd = max(0.0, min(1.0, gripper_cmd))
        grip_open = gripper_cmd * 0.04

        # Step 6: 发布关节指令
        self.get_logger().info(
            f"Current pos: {current_pos.round(3)}, "
            f"Target pos: {target_pos.round(3)}, "
            f"Delta pos: {delta_pos.round(3)}"
        )
        self.get_logger().info(
            f"Joint angles: {np.round(current_angles, 3)} -> {np.round(target_angles, 3)}"
        )

        # 机械臂关节轨迹（用 trajectory_time 参数控制速度）
        traj_sec = int(self.trajectory_time)
        traj_nsec = int((self.trajectory_time - traj_sec) * 1e9)

        arm_traj = JointTrajectory()
        arm_traj.joint_names = self.arm_joint_names
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(x) for x in target_angles[:6]]
        arm_point.velocities = [0.0] * len(self.arm_joint_names)
        arm_point.time_from_start.sec = traj_sec
        arm_point.time_from_start.nanosec = traj_nsec
        arm_traj.points.append(arm_point)
        self.arm_pub.publish(arm_traj)

        # 夹爪轨迹
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = self.gripper_joint_names
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [grip_open, -grip_open]
        gripper_point.velocities = [0.0, 0.0]
        gripper_point.time_from_start.sec = traj_sec
        gripper_point.time_from_start.nanosec = traj_nsec
        gripper_traj.points.append(gripper_point)
        self.gripper_pub.publish(gripper_traj)

        self.get_logger().info(
            f"IK action sent! Angles: {np.round(target_angles[:6], 3)}, Gripper: {grip_open:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VLAGazeboBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
