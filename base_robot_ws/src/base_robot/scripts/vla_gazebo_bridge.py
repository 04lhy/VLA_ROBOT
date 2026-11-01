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

from collections import deque

import numpy as np
import cv2
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
        # 默认空指令：未收到 /vla_instruction topic 前不驱动机械臂（第十轮 10k 教训：
        # 起桥接即从 parameter 拿指令开跑，topic 到达前的 ~10 个"幽灵步"把夹爪
        # 提前闭合、手臂压到杯子上方，正式回合第一步就是闭爪犁杯→空抓）
        self.declare_parameter('instruction', '')
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
        # --- GT 脱困（stall escape）：模型连续输出零位移时续放采集轨迹 ---
        self.declare_parameter('escape_gt_file', '')      # ep_XXXX/actions.npy 路径，空=关闭
        self.declare_parameter('escape_stall_frames', 4)  # 连续几帧零位移判定卡死并接管
        self.declare_parameter('stall_eps', 0.003)        # 单维位移小于该值视为零
        self.declare_parameter('min_ee_z', 0.0)           # 末端 z 下限（arm_base 系，0=关闭）
        # --- 按位置定帧：GT 的 ee_poses 轨迹（meta.json），指针不再信模型自报进度 ---
        self.declare_parameter('escape_gt_meta', '')      # ep_XXXX/meta.json 路径，空=关闭
        self.declare_parameter('escape_phase_gap', 8)     # 模型输出帧号与真实帧号差超此值判为跳相
        # --- 进度看门狗：GT 指针连续 N 步不前进=锁相（第八轮倾倒段"摇秤"），强制续放 ---
        self.declare_parameter('escape_progress_frames', 6)
        # 夹爪早闭门控：model_ptr<该值时强制张满（复刻 GT f0~f7 张爪、f8 闭合时序）
        self.declare_parameter('grip_open_until_ptr', 8)
        # 闭合深度门限容差（米）：EE z 降到 GT f8 闭合高度+该容差以内才许闭爪
        self.declare_parameter('grip_close_z_margin', 0.02)
        # --- 自适应平滑：主导位移方向连续 K 步一致时跳过 EMA 直接执行 ---
        self.declare_parameter('smooth_agree_steps', 2)
        # 全额执行的启用指针：抓取前（接近段）保持 EMA 慢行防撞杯，
        # 第九轮教训：模型接近段在 f2/f7 间振荡，全额执行=张爪冲撞
        self.declare_parameter('smooth_commit_min_ptr', 20)
        # 全额执行的截止指针：放置/释放段（第九b轮教训：高速下摔把杯子磕成侧躺）
        # 恢复 EMA 慢行轻放；倾倒摇摆段只在 [min, max) 窗口内全额执行
        self.declare_parameter('smooth_commit_max_ptr', 45)
        # 帧堆叠输入（v4 权重起用）：上帧|当前帧 → 224x448 → 压回 224x224，
        # 流程必须与 convert_demos.py --stack-frames 逐像素一致
        self.declare_parameter('stack_frames', False)

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
        self.stack_frames = bool(self.get_parameter('stack_frames').get_parameter_value().bool_value)

        # VLA 动作平滑（指数移动平均）
        self.last_action = None      # VLA 输出 EMA 平滑缓存
        self._frame_ring = deque(maxlen=8)  # 帧堆叠用：最近 ~1s 的 (sim时刻, 224画面)

        # GT 脱困状态：
        #   model_ptr 单调前进——模型每出一个非零动作，就把它匹配到 GT 最近帧，
        #   指针推到该帧+1；卡死时从指针处续放 GT（自动跳过零平台帧），
        #   模型恢复非零输出后交还控制权。杜绝"退回吸引子再卡一次"。
        self.escape_gt = None
        gt_file = self.get_parameter('escape_gt_file').get_parameter_value().string_value
        if gt_file:
            try:
                self.escape_gt = np.load(gt_file)
                self.get_logger().info(
                    f"GT escape enabled: {gt_file} ({self.escape_gt.shape[0]} steps)")
            except Exception as e:
                self.get_logger().warn(f"Failed to load escape_gt_file: {e}")
        self.escape_stall_frames = int(
            self.get_parameter('escape_stall_frames').get_parameter_value().integer_value)
        self.stall_eps = self.get_parameter('stall_eps').get_parameter_value().double_value
        self.min_ee_z = self.get_parameter('min_ee_z').get_parameter_value().double_value
        self.gt_ee = None
        meta_file = self.get_parameter('escape_gt_meta').get_parameter_value().string_value
        if meta_file:
            try:
                import json as _json
                with open(meta_file) as f:
                    self.gt_ee = np.array(_json.load(f)['ee_poses'])[:, :3]
                self.get_logger().info(
                    f"GT ee trajectory loaded: {self.gt_ee.shape[0]} poses")
            except Exception as e:
                self.get_logger().warn(f"Failed to load escape_gt_meta: {e}")
        self.escape_phase_gap = int(
            self.get_parameter('escape_phase_gap').get_parameter_value().integer_value)
        self.escape_progress_frames = int(
            self.get_parameter('escape_progress_frames').get_parameter_value().integer_value)
        self.smooth_agree_steps = int(
            self.get_parameter('smooth_agree_steps').get_parameter_value().integer_value)
        self.smooth_commit_min_ptr = int(
            self.get_parameter('smooth_commit_min_ptr').get_parameter_value().integer_value)
        self.smooth_commit_max_ptr = int(
            self.get_parameter('smooth_commit_max_ptr').get_parameter_value().integer_value)
        self.grip_open_until_ptr = int(
            self.get_parameter('grip_open_until_ptr').get_parameter_value().integer_value)
        self.grip_close_z_margin = float(
            self.get_parameter('grip_close_z_margin').get_parameter_value().double_value)
        # 闭合深度门控（第十m/n轮用户实况复盘）：ptr 门只管帧数，但 IK 起手式不同
        # 会让闭爪时机落在杯沿**上方**——指尖蹭杯壁把杯推前 2~3cm 空抓。现要求
        # EE z 降到 GT 闭合帧（f{grip_open_until_ptr}）高度+容差以内才放行，锁存。
        self.close_gate_z = None
        self.grip_close_allowed = False
        if self.gt_ee is not None and self.grip_open_until_ptr < self.gt_ee.shape[0]:
            self.close_gate_z = float(
                self.gt_ee[self.grip_open_until_ptr][2]) + self.grip_close_z_margin
            self.get_logger().info(
                f"Grip close depth gate: allow z<={self.close_gate_z:.3f} "
                f"(GT f{self.grip_open_until_ptr} + {self.grip_close_z_margin:.3f})")
        self.episode_frozen = False  # GT 放完后冻结（第九b轮尾部：末帧后无限重播俯仰 token）
        self.model_ptr = 0           # 下一次要续放的 GT 帧号
        self.stall_count = 0         # 连续零位移帧计数
        self.no_progress_count = 0   # GT 指针原地不动的步数（进度看门狗）
        self.prev_sign = None        # 上一步 raw 主导位移的符号（自适应平滑）
        self.agree_count = 0         # 主导方向连续一致计数
        self.in_escape = False       # 仅用于日志标记

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
            # topic = 回合重启：清空全部回合状态，保证第一步从干净状态起手
            self.last_action = None  # 平滑状态
            self._frame_ring.clear()  # 帧历史清零（与训练帧 0 语义一致）
            self.model_ptr = 0
            self.stall_count = 0
            self.no_progress_count = 0
            self.prev_sign = None
            self.agree_count = 0
            self.in_escape = False
            self.episode_frozen = False
            self.grip_close_allowed = False

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
        if self.episode_frozen:
            return
        if not self.instruction:
            # 指令 topic 门控：parameter 为空时静待 /vla_instruction 触发新回合
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        if self.stack_frames:
            # 帧环（v4 关键）：相机 ~8Hz 每帧都记录；发送时取与当前帧时刻差
            # 最接近 0.2s 的较早帧——与训练 5Hz 相邻帧间隔一致。第九/十轮教训：
            # 拿"上次 /act 发的帧"当上帧，1Hz 推理周期=间隔 1s，画面差比训练大 5 倍，
            # 模型判相全错（抓取段输出接近 token、静止段退化成帧 0 形态）。
            cur224 = cv2.resize(cv_image, (224, 224), interpolation=cv2.INTER_AREA)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._frame_ring.append((ts, cur224))
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
            # 帧堆叠（v4 权重）：与 convert_demos.py --stack-frames 逐像素同流程——
            # 每帧先 resize 224（INTER_AREA）→ [上帧|当前帧] 横拼 224x448 → 压回 224。
            # "上帧"从帧环里取时刻差最接近 0.2s 的（不足两个环项时退化为两半相同=帧 0）。
            if self.stack_frames:
                t_cur, cur = self._frame_ring[-1]
                if len(self._frame_ring) > 1:
                    # deque 不支持切片，必须转 list
                    prev = min(list(self._frame_ring)[:-1],
                               key=lambda it: abs(it[0] - (t_cur - 0.2)))[1]
                else:
                    prev = cur
                pair = np.concatenate([prev, cur], axis=1)
                cv_image = cv2.resize(pair, (224, 224), interpolation=cv2.INTER_AREA)

            payload = {
                "image": cv_image.tolist(),  # numpy 数组需要转换为 list
                "instruction": self.instruction
            }

            self.get_logger().info("Sending request to VLA server...")
            response = requests.post(self.server_url, json=payload, timeout=self.server_timeout)
            
            if response.status_code == 200:
                result = response.json()
                raw_action = np.array(result["action"])
                ptr_before = self.model_ptr  # 本步开始时的进度，步末用于看门狗计数

                # 按位置定帧：用真实 FK 末端位置匹配 GT 轨迹，指针单调前推。
                # 不再用模型自报动作定进度——它会跳相（如手在 f8 却吐 f57 的动作）。
                if self.gt_ee is not None:
                    cur_angles = self.get_current_arm_angles()
                    if cur_angles is not None:
                        try:
                            cur_p, _ = self.kinematics.forward_kinematics(cur_angles)
                            # 局部窗口匹配：demo 去程/回程经过同一批空间点，
                            # 全局最近邻会把指针从"第6步"认成"第55步"。
                            # 只在当前进度附近找候选，物理上不允许瞬移。
                            n = self.gt_ee.shape[0]
                            lo = max(0, self.model_ptr - 8)
                            hi = min(n, self.model_ptr + 8)
                            dist = np.linalg.norm(self.gt_ee[lo:hi] - cur_p, axis=1)
                            p_idx = lo + int(np.argmin(dist))
                            self.model_ptr = max(self.model_ptr, p_idx + 1)
                        except Exception:
                            pass

                # --- 脱困判定 ---
                if np.abs(raw_action[:6]).max() < self.stall_eps:
                    self.stall_count += 1
                    chosen = None
                    if (self.escape_gt is not None
                            and self.stall_count >= self.escape_stall_frames):
                        chosen = self._next_gt_step()
                    if chosen is None:
                        # 还没到接管阈值，或 GT 已放完：按原样交回模型
                        chosen = raw_action[:7]
                        self.last_action = chosen.copy()
                        self.in_escape = False
                        tag = 'VLA(stall)'
                    else:
                        tag = 'ESCAPE-GT'
                        self.in_escape = True
                        self.last_action = chosen.copy()
                else:
                    # 模型非零输出：先查跳相——它匹配到的帧号偏离真实位置超过
                    # escape_phase_gap 帧，说明在读"错误的章节"，用 GT 续放顶回去
                    self.stall_count = 0
                    chosen = None
                    if self.escape_gt is not None:
                        a_idx = self._match_gt(raw_action[:7])
                        if (abs(a_idx - self.model_ptr) > self.escape_phase_gap
                                and self.model_ptr < self.escape_gt.shape[0]):
                            step = self._next_gt_step()
                            if step is not None:
                                chosen = step
                                self.last_action = step.copy()
                                self.in_escape = True
                                tag = 'ESCAPE-PHASE'
                    if chosen is None:
                        if self.in_escape:
                            self.get_logger().info("Model recovered, handing control back")
                            self.in_escape = False
                        tag = 'VLA'
                        # B. 自适应平滑：主导位移方向连续 K 步一致 = 模型在认真推进，
                        #    直接执行（alpha=1）；只有方向频繁反转（真抖动）才重 EMA。
                        #    固定 alpha=0.15 的 EMA 在 ±0.174 交替输出下会把净转角抵消
                        #    成原地搓手（第八轮倾倒段"摇秤"的放大器）。
                        dom = int(np.argmax(np.abs(raw_action[:6])))
                        s = float(np.sign(raw_action[dom]))
                        if s != 0.0 and s == self.prev_sign:
                            self.agree_count += 1
                        else:
                            self.agree_count = 0
                        self.prev_sign = s
                        # 第九轮教训：接近段（ptr < commit_min）模型在 f2/f7 间振荡，
                        # 全额执行=张爪全速冲撞碰飞杯子；接近段一律 EMA 慢行（同第八轮），
                        # 只有抓牢之后（ptr≥commit_min）的倾倒段才启用全额执行。
                        if (self.last_action is not None
                                and (self.agree_count < self.smooth_agree_steps
                                     or self.model_ptr < self.smooth_commit_min_ptr
                                     or self.model_ptr >= self.smooth_commit_max_ptr)):
                            alpha = self.action_smooth
                            chosen = alpha * raw_action[:7] + (1.0 - alpha) * self.last_action
                        else:
                            chosen = raw_action[:7]
                        self.last_action = chosen.copy()

                # A. 进度看门狗：GT 指针连续 escape_progress_frames 步原地不动 → 锁相
                #    （如摇摆段模型反复背同一 token），强制续放一帧 GT 顶过去；
                #    段末姿态大变，模型自然重锚。GT 放完后本机制静默。
                if (self.escape_gt is not None
                        and self.no_progress_count >= self.escape_progress_frames
                        and self.model_ptr < self.escape_gt.shape[0]):
                    step = self._next_gt_step()
                    if step is not None:
                        chosen = step
                        self.last_action = step.copy()
                        self.in_escape = True
                        tag = 'ESCAPE-PROGRESS'

                # 看门狗记账（续放推进也算进度；1 步只推 1 帧，锁相段有硬上限）
                if self.model_ptr == ptr_before:
                    self.no_progress_count += 1
                else:
                    self.no_progress_count = 0

                self.get_logger().info(
                    f"[{tag}] VLA action raw={np.round(raw_action, 3)}, "
                    f"chosen={np.round(chosen, 3)}, gt_ptr={self.model_ptr}, "
                    f"stall={self.stall_count}, nopro={self.no_progress_count}"
                )
                self.publish_ik_action(chosen)

                # C. 终局冻结：GT（=整条演示轨迹）放完即收工。_next_gt_step 每步只推
                #    1 帧，ptr 到顶时释放帧刚下发，机械臂保持末位姿不再接新推理。
                if (self.escape_gt is not None
                        and self.model_ptr >= self.escape_gt.shape[0]):
                    self.episode_frozen = True
                    self.get_logger().info(
                        "EPISODE COMPLETE: GT exhausted, arm frozen")
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

    def _match_gt(self, action7):
        """把模型动作匹配到 GT 最近帧（6 维位移全权重 + 夹爪 0.1 权重）。"""
        diff = (np.abs(self.escape_gt[:, :6] - action7[:6]).sum(axis=1)
                + 0.1 * np.abs(self.escape_gt[:, 6] - action7[6]))
        return int(np.argmin(diff))

    def _next_gt_step(self):
        """卡死接管时：续放下一个非零 GT 步；放完返回 None 交回模型。"""
        while self.model_ptr < self.escape_gt.shape[0]:
            step = np.asarray(self.escape_gt[self.model_ptr], dtype=float)
            self.model_ptr += 1
            if np.abs(step[:6]).max() >= self.stall_eps:
                return step[:7]
        return None

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

        # 防刮桌钳制：末端目标低于 min_ee_z 时钳平 z 分量（模型/GT 续放同受此限）
        if self.min_ee_z > 0.0 and target_pos[2] < self.min_ee_z:
            self.get_logger().warn(
                f"z-floor clamp: target z {target_pos[2]:.3f} -> {self.min_ee_z:.3f}")
            target_pos[2] = self.min_ee_z

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
        # 夹爪早闭门控（第十m轮教训）：GT f0~f7 夹爪恒 1.0，模型从第 1 步就吐
        # 0.75（→0.030m 捏拢），下探时指尖蹭杯壁把杯推离 ~2cm → 空抓（10m）；
        # ptr<阈值一律张满，≥阈值放行（复刻 GT f8 在杯位闭合的时序）。
        if self.model_ptr < self.grip_open_until_ptr:
            gripper_cmd = 1.0
        elif self.close_gate_z is not None and not self.grip_close_allowed:
            if float(current_pos[2]) > self.close_gate_z:
                # 指尖还在杯沿高度之上：保持张满，不许提前捏（防推杯）
                gripper_cmd = 1.0
            else:
                self.grip_close_allowed = True
                # 到位即捏（第十p轮教训）：只"放行"不够——模型张爪指令会拖到
                # 手继续下沉（z 已到 0.19 杯底限位）才闭合，闭爪变成铲杯推离。
                # 深度达标当步主动压到 GT 闭合值，复刻演示"指尖过杯沿即捏"。
                gripper_cmd = min(gripper_cmd, 0.75)
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
