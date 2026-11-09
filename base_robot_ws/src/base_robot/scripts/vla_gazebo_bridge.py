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
import math
import os
import shlex
import subprocess
import time

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
        # [2026-11-07 GUI 观赏轮实锤] 模型可"复读卡死"（连续步输出逐字节相同，
        # raw≠0 骗过零位移 stall 判据），且脱困与交还反复乒乓（实录单回合 22 次
        # 切换）→ 臂在模型/GT 目标间来回抽搐还抓偏杯子。对策：复读判死或脱困
        # 触发超上限，本回合**锁 GT 一路平滑复放演示到完**（终点与已验证的
        # PLACE-TAIL 同款，物理动作真实发生，不再交还给抽风模型）。
        self.declare_parameter('max_escape_entries', 3)
        self.declare_parameter('repeat_action_frames', 2)
        self.declare_parameter('repeat_action_eps', 1e-6)
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

        # agent/巡检 整合（2026-11-02）：回合起手自动去 f1 预热位 + 回合状态外发。
        # warm_joints = GT ep_0004 f1 臂关节 [0,0.493,0.084,0.001,-0.5,-0.001]
        # （第十轮手工协议定案：不预热首帧翻肘下垂=犁杯）。auto_warm=False 保持
        # 旧手工流程：外部自己预热、指令到达立即起回合。
        self.declare_parameter('auto_warm', True)
        self.declare_parameter('warm_joints',
                                [0.0, 0.493, 0.084, 0.001, -0.5, -0.001])
        self.declare_parameter('warm_time', 3.0)     # 预热轨迹时长（秒）
        self.declare_parameter('warm_settle', 10.0)  # 到位后静置（秒）
        self.declare_parameter('grip_open_cmd', 0.04)  # 预热张爪开度

        # 放杯尾段（用户实况 2026-11-04）：回合收口按 GT 帧数硬切（每推理步只推
        # 1 帧），而演示的"放回+张爪"只有最后 10 帧（ep_0004 f44~f53）——模型相位
        # 一慢整段被削掉，夹爪保持闭合，杯子被夹在爪里随车走完全程。现补一段
        # 确定性尾：GT 放完且夹爪仍未张开时，用采集的真实关节位把臂送回**张爪
        # 首帧位姿**（演示里杯底坐实桌面的那一拍）→ 坐实 → 张满脱开 → 按演示抬离。
        self.declare_parameter('place_tail', True)
        self.declare_parameter('place_tail_joint_step', 0.12)  # 单步每关节上限（弧度）
        self.declare_parameter('place_tail_max_steps', 25)     # 送回段步数上限
        self.declare_parameter('place_tail_step_dur', 0.6)     # 每步时长与间隔（秒）
        self.declare_parameter('place_tail_lift_frames', 4)    # 脱开后按演示再抬 N 帧

        # 抓握几何兜底（用户实况 2026-11-04 run13：深度门只管 z，横向 x/y 全听模型，
        # 接近段偏了 4cm → 指尖从侧面蹭杯、捏拢在 0.030 却夹了个空，插件 0 次附着事件，
        # 而"倾倒+放回"整套动作照样跑完 → 流程全绿但任务根本没发生）。
        # 闭爪那一步先把臂送进演示 f8 的真实抓取位，再把被蹭走的杯子归回名义位，
        # 然后才放行闭爪。顺序不能颠倒：先让两指到位，杯子 teleport 回来才落在指间，
        # 不会与手指互穿爆物理。cup_model/cup_nominal 留空=关掉杯子归位（只做臂对准）。
        self.declare_parameter('grasp_assist', True)
        self.declare_parameter('grasp_assist_joint_step', 0.12)  # 对准段单关节上限（弧度）
        self.declare_parameter('grasp_assist_max_steps', 20)     # 对准段步数上限
        self.declare_parameter('grasp_assist_step_dur', 0.6)     # 对准段每步时长（秒）
        # 闭爪前等臂真正到位：trajectory 是异步执行的，固定 sleep 会在臂还在下探时
        # 合爪（run14 实录：实测还差 0.239 rad 就闭爪，指尖擦壁而过；run21 整套
        # 捏空也与该时序有关）。到位判据=joint_states 回读误差。
        self.declare_parameter('grasp_settle_tol', 0.05)         # 到位容差（弧度）
        self.declare_parameter('grasp_settle_timeout', 4.0)      # 等位超时（秒）
        self.declare_parameter('cup_model', 'plastic_cup_0_clone')
        self.declare_parameter('cup_nominal', [14.152, -11.514, 0.456645])
        self.declare_parameter('cup_snap_drift', 0.02)           # 蹭偏超此值才归位（米）

        # 张爪门控（dock 测试实录：GT 末帧在非放杯位姿松手 → 杯子半空砸桌侧躺）。
        # release_gate_grip：夹爪指令 ≥ 此值视为"要松手"（GT 张爪值=1.0）。
        # release_pose_tol：臂与 GT 放杯位（张爪首帧）的最大关节偏差容差（弧度）。
        # 放杯复放参数：place_tail_hover=张爪前先悬停的帧数（EE 高 8mm/帧）；
        # tail_replay_step_dur=复放段每帧时长。直接一步插值冲到放杯位会把杯压进
        # 桌面（dock 实录 z 压到 0.4405 < 站立 0.4566，张爪时接触力把杯崩飞 14cm
        # 侧倒）——必须复放演示的逐帧下降段做软着陆。
        self.declare_parameter('release_gate_grip', 0.9)
        self.declare_parameter('release_pose_tol', 0.15)
        self.declare_parameter('place_tail_hover', 4)
        self.declare_parameter('tail_replay_step_dur', 0.5)
        # 停降余量：读到杯底高于站立高度 + 该值就视为"还没触桌"；反过来读到低于
        # 站立高度说明已经压桌（不可接受）。留 2.5cm 余量=宁可轻放 2cm，也不压桌。
        self.declare_parameter('place_stop_margin', 0.025)
        # 抓牢后的倾倒/放回段由演示轨迹接管：模型实测在抬升段持续吐负 z，把臂拽回
        # 桌面，GT 指针冲到末段时杯子只抬了 2cm（run15 实录）——原地逐帧复放演示
        # 动作（抬升 5cm→倾倒→放回）才保证物理动作真的发生。张爪仍走门控。
        self.declare_parameter('post_grasp_replay', True)

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
        self.auto_warm = bool(self.get_parameter('auto_warm').get_parameter_value().bool_value)
        self.warm_joints = [float(v) for v in
                            self.get_parameter('warm_joints').get_parameter_value().double_array_value]
        self.warm_time = float(self.get_parameter('warm_time').get_parameter_value().double_value)
        self.warm_settle = float(self.get_parameter('warm_settle').get_parameter_value().double_value)
        self.grip_open_cmd = float(self.get_parameter('grip_open_cmd').get_parameter_value().double_value)
        self.place_tail = bool(
            self.get_parameter('place_tail').get_parameter_value().bool_value)
        self.place_tail_joint_step = float(
            self.get_parameter('place_tail_joint_step').get_parameter_value().double_value)
        self.place_tail_max_steps = int(
            self.get_parameter('place_tail_max_steps').get_parameter_value().integer_value)
        self.place_tail_step_dur = float(
            self.get_parameter('place_tail_step_dur').get_parameter_value().double_value)
        self.place_tail_lift_frames = int(
            self.get_parameter('place_tail_lift_frames').get_parameter_value().integer_value)
        self.grasp_assist = bool(
            self.get_parameter('grasp_assist').get_parameter_value().bool_value)
        self.grasp_assist_joint_step = float(
            self.get_parameter('grasp_assist_joint_step').get_parameter_value().double_value)
        self.grasp_assist_max_steps = int(
            self.get_parameter('grasp_assist_max_steps').get_parameter_value().integer_value)
        self.grasp_assist_step_dur = float(
            self.get_parameter('grasp_assist_step_dur').get_parameter_value().double_value)
        self.grasp_settle_tol = float(
            self.get_parameter('grasp_settle_tol').get_parameter_value().double_value)
        self.grasp_settle_timeout = float(
            self.get_parameter('grasp_settle_timeout').get_parameter_value().double_value)
        self.cup_model = str(
            self.get_parameter('cup_model').get_parameter_value().string_value).strip()
        self.cup_nominal = [float(v) for v in
                            self.get_parameter('cup_nominal').get_parameter_value().double_array_value]
        self.cup_snap_drift = float(
            self.get_parameter('cup_snap_drift').get_parameter_value().double_value)
        self.release_gate_grip = float(
            self.get_parameter('release_gate_grip').get_parameter_value().double_value)
        self.release_pose_tol = float(
            self.get_parameter('release_pose_tol').get_parameter_value().double_value)
        self.place_tail_hover = int(
            self.get_parameter('place_tail_hover').get_parameter_value().integer_value)
        self.tail_replay_step_dur = float(
            self.get_parameter('tail_replay_step_dur').get_parameter_value().double_value)
        self.place_stop_margin = float(
            self.get_parameter('place_stop_margin').get_parameter_value().double_value)
        self.post_grasp_replay = bool(
            self.get_parameter('post_grasp_replay').get_parameter_value().bool_value)
        # 武装闸门：预热完成前不把帧送模型（防 WARMING 期间抢跑动作）
        self.armed = not self.auto_warm

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
        self.gt_joints = None
        meta_file = self.get_parameter('escape_gt_meta').get_parameter_value().string_value
        if meta_file:
            try:
                import json as _json
                with open(meta_file) as f:
                    _meta = _json.load(f)
                self.gt_ee = np.array(_meta['ee_poses'])[:, :3]
                self.get_logger().info(
                    f"GT ee trajectory loaded: {self.gt_ee.shape[0]} poses")
                # 放杯尾段要用**演示真实关节位**（ee_poses 的姿态列采集时恒为单位
                # 四元数，不能用来判"转正"；joint_states 才是可执行的位姿）
                if 'joint_states' in _meta:
                    self.gt_joints = np.array(_meta['joint_states'], dtype=float)
                    self.get_logger().info(
                        f"GT joint trajectory loaded: {self.gt_joints.shape}")
                    if (self.escape_gt is not None
                            and self.gt_joints.shape[0] != self.escape_gt.shape[0]):
                        # 兜底段要按**帧号**同时取 actions 的夹爪列和 joint_states
                        # 的位姿列，两者行数必须一一对应（裁剪脚本改了其中一边就会错帧）
                        self.get_logger().warn(
                            f"GT actions({self.escape_gt.shape[0]}) 与 "
                            f"meta.joint_states({self.gt_joints.shape[0]}) 行数不等，"
                            f"抓握对准/放杯尾段会错帧！请用同一版本数据集")
            except Exception as e:
                self.get_logger().warn(f"Failed to load escape_gt_meta: {e}")
        self.escape_phase_gap = int(
            self.get_parameter('escape_phase_gap').get_parameter_value().integer_value)
        self.escape_progress_frames = int(
            self.get_parameter('escape_progress_frames').get_parameter_value().integer_value)
        self.max_escape_entries = int(
            self.get_parameter('max_escape_entries').get_parameter_value().integer_value)
        self.repeat_action_frames = int(
            self.get_parameter('repeat_action_frames').get_parameter_value().integer_value)
        self.repeat_action_eps = float(
            self.get_parameter('repeat_action_eps').get_parameter_value().double_value)
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
        self.gt_latched = False      # 复读卡死/脱困超上限 → 本回合锁 GT，不再交还
        self.escape_entries = 0      # 本回合脱困触发次数（False→True 计一次）
        self.repeat_count = 0        # raw 动作与上一步逐字节相同的连续计数
        self.last_raw_action = None  # 上一步 raw（复读检测用）
        self.episode_started_at = 0.0  # 本回合 instruction 起点（重触发护栏用）
        self.last_gripper_cmd = 1.0  # 门控后**实际下发**的夹爪值（放杯尾段判据）
        self.post_release = False    # 门控已放行张爪（防收口尾段二次下降扰动杯子）

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

        # 回合状态机（供上层 agent 阻塞等待）：IDLE → WARMING → RUNNING → COMPLETE
        self.episode_status = 'IDLE'
        self._last_complete_at = 0.0  # 同文案回声去抖用（见 instruction_callback）
        self.status_pub = self.create_publisher(String, '/vla_episode_status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self._publish_status()

        # 兼容旧 demo 流程：launch 直接带 instruction 参数起手的，也要先预热再开闸
        if self.instruction and self.auto_warm:
            self.get_logger().info(
                f"Startup instruction preset: '{self.instruction}' → auto warmup first")
            self._set_status('WARMING')
            self._warmup_arm()
            self.armed = True
            self._set_status('RUNNING')

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

    def _publish_status(self):
        self.status_pub.publish(String(data=self.episode_status))
        # 文件心跳（与 topic 同拍）：机器过载时 agent 的 ros2 CLI 冷启动
        # （python 解释器 + DDS 发现）整读超时——加测第2趟/续测第3、4趟实录，
        # 30s 容限都救不回来。读一个带时间戳的本地文件是零成本旁路，agent
        # 侧 _read_vla_status 优先读它。先写临时文件再 rename，杜绝半行读。
        try:
            tmp = '/tmp/vla_episode_status.txt.tmp'
            with open(tmp, 'w') as f:
                f.write(f"{self.episode_status} {time.time():.3f}\n")
            os.replace(tmp, '/tmp/vla_episode_status.txt')
        except Exception:
            pass

    def _sleep_beat(self, dur):
        """带心跳的阻塞睡眠（预热/接管/伺服/放杯尾段全部用它）。

        单线程执行器在回调里时 status_timer（1Hz）根本不会被调度——预热、
        GT-LATCH 整段接管、PLACE-TAIL 这些同步阻塞段就是 /vla_episode_status
        的静默死区。三连 RUN1 实锤：接管正常干活 87s，期间 agent 每轮"单帧读
        状态"全撞上死区，误判"指令未送达"连发 3 条指令，桥接被同文案重触发
        起了三个互相踩踏的回合。心跳手动补发当前状态，成本≈0。"""
        end = time.monotonic() + dur
        while True:
            self._publish_status()
            rem = end - time.monotonic()
            if rem <= 0:
                return
            time.sleep(min(rem, 1.0))

    def _set_status(self, status):
        if status != self.episode_status:
            self.episode_status = status
            self.get_logger().info(f"VLA episode status: {status}")
            if status == 'COMPLETE':
                self._last_complete_at = time.monotonic()
        self._publish_status()

    def _pub_position(self, pub, joint_names, positions, duration):
        """直发关节位（预热/张爪用），多发两次防首条丢失。"""
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(pt)
        for _ in range(2):
            pub.publish(traj)
            time.sleep(0.2)

    def _warmup_arm(self):
        """回合起手自动预热到 GT f1 臂关节 + 张满夹爪（复刻第十轮手工预热协议）。
        同步阻塞回调：预热轨迹 + 静置期间不收帧不起动作。"""
        if len(self.warm_joints) != len(self.arm_joint_names):
            self.get_logger().warn("warm_joints 长度与 arm_joint_names 不符，跳过预热")
            return
        self.get_logger().info(
            f"Warmup arm to {np.round(self.warm_joints, 3).tolist()} "
            f"(+grip open {self.grip_open_cmd})")
        self._pub_position(self.arm_pub, self.arm_joint_names,
                           self.warm_joints, self.warm_time)
        o = self.grip_open_cmd
        self._pub_position(self.gripper_pub, self.gripper_joint_names,
                           [o, -o], 1.0)
        self._sleep_beat(self.warm_time + self.warm_settle)

    def _gz_model_pose(self, model, retries=3):
        """gz CLI 读模型世界位姿 [x,y,z,R,P,Y]；读不到返回 None（仿真没起/名字不对）。

        `timeout 5 gz model -p` 在 gzserver 高负载（推理回调+渲染并发）时会**挂到
        超时**，此时 stdout 为空字符串——split() 得空列表。调用方拿到的必须是
        None 而不是空列表，否则 `pose[0]` IndexError 会把整个回合炸死
        （dock run3 实录：Step5 门控里炸异常，FATAL 中止）。带重试。
        """
        for _ in range(max(1, retries)):
            try:
                out = subprocess.run(
                    ["bash", "-lc", f"timeout 5 gz model -m {shlex.quote(model)} -p"],
                    text=True, capture_output=True, timeout=12).stdout.split()
                if len(out) >= 6:
                    return [float(v) for v in out[:6]]
            except Exception:
                pass
            self._sleep_beat(1.0)
        return None

    def _gz_set_model_pose(self, model, x, y, z, yaw=0.0):
        """gz CLI 传送模型回名义位。

        必须分轴 `-x -y -z -Y`：连写 `-p x y z r p Y` 时负坐标会被 getopt 当成
        选项吞掉，报 Invalid arguments 且**静默不生效**（V1.8.0 实录）。
        """
        cmd = (f"timeout 6 gz model -m {shlex.quote(model)} "
               f"-x {float(x)} -y {float(y)} -z {float(z)} -R 0 -P 0 -Y {float(yaw)}")
        return subprocess.run(["bash", "-lc", cmd], text=True,
                              capture_output=True, timeout=15).returncode == 0

    def _teleport_verified(self, model, x, y, z, yaw=0.0, tol=0.02, tries=3):
        """传送并回读验证：gz CLI 在导航/多节点负载下会返回 0 但静默不生效
        （run12 实录：导航后 Step4 传送底盘，8s 后回读仍在 30cm 外的旧位）。"""
        for k in range(tries):
            self._gz_set_model_pose(model, x, y, z, yaw)
            self._sleep_beat(1.0)
            pose = self._gz_model_pose(model, retries=2)
            if pose is None:
                continue
            d = math.hypot(pose[0] - x, pose[1] - y)
            dyaw = math.atan2(math.sin(pose[5] - yaw), math.cos(pose[5] - yaw))
            if d <= tol and abs(dyaw) <= 0.08:
                return True
            self.get_logger().warn(
                f"TELEPORT: {model} 第 {k + 1} 次未生效（偏 {d * 100:.1f}cm / "
                f"{math.degrees(dyaw):.1f}°），重试")
            self._sleep_beat(0.5)
        return False

    def _wait_joint_target(self, target, timeout=None, tol=None):
        """等关节状态真正到达 target（异步轨迹执行），返回最终最大误差。"""
        timeout = self.grasp_settle_timeout if timeout is None else timeout
        tol = self.grasp_settle_tol if tol is None else tol
        tgt = np.asarray(target, dtype=float)
        deadline = time.monotonic() + timeout
        jerr = None
        while time.monotonic() < deadline:
            cur = self.get_current_arm_angles()
            if cur is not None:
                jerr = float(np.abs(np.asarray(cur, dtype=float) - tgt).max())
                if jerr <= tol:
                    return jerr
            self._sleep_beat(0.15)
        return jerr

    def _servo_arm_to_joints(self, target, step, max_steps, dur, tag):
        """关节空间分步插值到目标位（抓握对准 / 放杯尾段共用）。返回实际步数或 None。"""
        cur = self.get_current_arm_angles()
        if cur is None:
            self.get_logger().warn(f"{tag}: 读不到当前关节角，跳过")
            return None
        cur = np.array(cur, dtype=float)
        delta = np.asarray(target, dtype=float) - cur
        n = max(1, min(int(np.ceil(np.abs(delta).max() / max(step, 1e-3))), max_steps))
        for k in range(1, n + 1):
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               cur + delta * (k / float(n)), dur)
            self._sleep_beat(dur)
        return n

    def _grasp_align_and_snap(self, force_snap: bool = False):
        """闭爪放行前的抓握几何兜底：臂先送进演示抓取位，再把被蹭走的杯子归位。

        force_snap=True（探针补抓路径）：无视容差、每次都把杯 teleport 回标称位。
        加测第2趟实锤：臂对 GT f8 有 0.149rad 结构性残差，杯哪怕只偏 1~2cm"在容差
        内"，指尖也会浅咬（0.462→0.469→0.474 三连不过线，复放期把杯 8.6cm 带起又
        在放桌瞬间滑脱 R=1.49）。补抓一轮=一轮免费的新对齐机会，必须让杯回到零位。

        只做 z 深度门不够——横向 x/y 全听模型的，模型接近段偏 4cm 就是"捏空气"
        （run13 实录：插件 0 次附着，整套倾倒+放回照常跑完，流程全绿而任务没发生）。
        顺序不能颠倒：先让两指到位，杯子 teleport 回来才正好落进指间，不会互穿爆物理。

        返回 True=臂已到位，**本步模型的臂轨迹必须抑制**：否则 Step6 紧接着把
        模型那条目标发出去，对准位在同一步里就被覆盖，捏合仍发生在模型的错位上
        （注入测试实录：GRASP-ASSIST 日志正常、joint6 却还带着 -0.456 rad，杯子
        没被夹住反而被扫倒、推走 10cm）。
        """
        idx = self.grip_open_until_ptr
        if self.gt_joints is None or idx >= self.gt_joints.shape[0]:
            self.get_logger().warn(
                "GRASP-ASSIST: 没有 GT 关节位可用（缺 meta.joint_states），按模型原样闭爪")
            return False
        # 闭爪前不做任何底盘传送：机械臂此刻已伸在桌边，传送后的位姿会与桌子
        # 穿插，物理把车顶开几十厘米——车"自己乱动"就是这么来的（run15 实录）。
        # 底盘几何由回合前的 dock（agent/脚本）负责，桥接只对齐臂。
        n = self._servo_arm_to_joints(self.gt_joints[idx],
                                      self.grasp_assist_joint_step,
                                      self.grasp_assist_max_steps,
                                      self.grasp_assist_step_dur, "GRASP-ASSIST")
        if n is None:
            return False
        # 等臂真正到位再闭爪（不做固定 sleep）：不到位就补送一次目标再等。
        jerr = self._wait_joint_target(self.gt_joints[idx])
        if jerr is None or jerr > self.grasp_settle_tol:
            self.get_logger().warn(
                f"GRASP-ASSIST: 首轮未到位（最大误差 "
                f"{'读不到' if jerr is None else f'{jerr:.3f} rad'}），补送一次目标")
            self._servo_arm_to_joints(self.gt_joints[idx],
                                      self.grasp_assist_joint_step,
                                      self.grasp_assist_max_steps,
                                      self.grasp_assist_step_dur, "GRASP-ASSIST")
            jerr = self._wait_joint_target(self.gt_joints[idx])
        self.get_logger().info(
            f"GRASP-ASSIST: 到位确认，距 GT f{idx} 最大关节误差 "
            f"{'读不到' if jerr is None else f'{jerr:.3f} rad'}")
        if not self.cup_model or len(self.cup_nominal) < 3:
            self.get_logger().info(
                f"GRASP-ASSIST: 臂已送进 GT f{idx} 抓取位（{n} 步），未启用杯子归位")
            return True
        pose = self._gz_model_pose(self.cup_model)
        if pose is None:
            self.get_logger().warn(
                f"GRASP-ASSIST: 读不到 {self.cup_model} 位姿，本轮只做了臂对准")
            return True
        dx = pose[0] - self.cup_nominal[0]
        dy = pose[1] - self.cup_nominal[1]
        drift = math.hypot(dx, dy)
        # 倾角也触发归位（加测1趟实锤）：前两次闭爪捏空把杯蹭成半倒（z 0.466/
        # R≈1.4 的歪杯 xy 没漂移），第三闭"合格"抓了歪杯→全程歪着放，末态倒伏。
        # 只查 xy 的归位对"原地倾倒"是瞎的；teleport 连 R/P 一并清零归正。
        tilt = (max(abs(pose[3]), abs(pose[4])) if len(pose) >= 5 else 0.0)
        if force_snap or drift > self.cup_snap_drift or tilt > 0.35:
            ok = self._teleport_verified(self.cup_model, *self.cup_nominal[:3], tol=0.015)
            self.get_logger().info(
                f"GRASP-ASSIST: 臂已到位（{n} 步）；"
                f"{'补抓强制归零' if force_snap else f'杯子被蹭偏 {drift * 100:.1f}cm (dx={dx:+.3f} dy={dy:+.3f}) →'} "
                f"杯已{'归位' if ok else '归位失败！'}，然后闭爪")
            self._sleep_beat(1.0)
        else:
            self.get_logger().info(
                f"GRASP-ASSIST: 臂已送进 GT f{idx} 抓取位（{n} 步），杯子偏移 "
                f"{drift * 100:.1f}cm 在容差内，直接闭爪")
        return True

    def _at_release_pose(self, current_angles) -> bool:
        """臂当前是否已在演示的放杯位姿（GT 张爪首帧的关节位）。

        偏差判据用关节空间（最大单关节差）而不是 EE 高度：杯口朝向、腕部倾角
        才决定松手后杯子是坐实桌面还是磕沿倒下，而这些在 EE z 上看不出差别。
        """
        idx = self._release_frame()
        if idx is None or self.gt_joints is None or idx >= self.gt_joints.shape[0]:
            return True   # 没有 GT 放杯位可比 → 不拦，照模型原样松
        dev = float(np.abs(np.asarray(current_angles, dtype=float)
                           - self.gt_joints[idx]).max())
        at = dev <= self.release_pose_tol
        if not at:
            self.get_logger().info(
                f"RELEASE-GATE: 距放杯位最大关节偏差 {dev:.3f} rad "
                f"> 容差 {self.release_pose_tol}")
        return at

    def _release_frame(self):
        """GT 里"张爪脱开"的首帧号：≥grip_open_until_ptr 后第一个夹爪≥0.9 的帧。

        ep_0004（裁剪静止帧后 54 帧）= f44，即演示中杯底坐实桌面的那一拍。
        """
        if self.escape_gt is None:
            return None
        for i in range(self.grip_open_until_ptr, self.escape_gt.shape[0]):
            if float(self.escape_gt[i][6]) >= 0.9:
                return i
        return None

    def _post_grasp_active(self):
        """抓牢后、演示轨迹放完前：本回合的臂/爪走向一律由 GT 复放决定。

        模型只在"接近 + 闭爪"段有话语权；倾倒/放回段的输出实测不可用（负 z
        把臂拽回桌面、正 z 顶穿桌面），继续听它会把手指着杯子空挥。"""
        return (self.post_grasp_replay and self.grip_close_allowed
                and self.escape_gt is not None and not self.episode_frozen)

    def _enter_escape(self):
        """GT 脱困触发记账：in_escape 置位；同回合累计触发超上限 → 锁 GT。

        锁 GT 后本回合不再把控制权交还模型（乒乓交还就是 GUI 轮"抽搐"的直接
        病因：每次交还第一步 raw 仍是那串复读垃圾，再弃权，臂来回折返）。"""
        if not self.in_escape:
            self.escape_entries += 1
            if (self.escape_gt is not None
                    and self.escape_entries >= self.max_escape_entries
                    and not self.gt_latched):
                self.gt_latched = True
                self.get_logger().info(
                    f"GT-LATCH: 本回合脱困已触发 {self.escape_entries} 次，"
                    f"模型反复抽风，锁 GT 复放演示直到本回合结束")
        self.in_escape = True

    def _gt_full_takeover(self) -> bool:
        """GT-LATCH 整体接管：本步起把剩余演示**闭环伺服绝对位姿**一次放完收工。

        为什么 latch 后不能再"每步续放一帧 GT 增量"（GUI 观赏轮实锤的翻车原因）：
          1) VLA 动作是 EE 增量，publish_ik_action 把它叠加在"臂当前位置"上——
             臂早在模型抽风阶段就偏出演示轨迹，增量复放只会把错位一路积分下去，
             指尖对着杯子旁边空合（实录：全程杯子只挪 1.2cm，z 从未离开桌面带）；
          2) 一步一帧=每帧 4~5s 等推理，FK 重锚和 _next_gt_step 还并行推指针，
             臂的物理进度追不上，闭爪/抬升帧到点时臂还在半路；
          3) 增量走 IK+max_joint_delta 限幅，帧间姿态跳变被削，动作来回摆（joint6
             0.34↔0.86 rad 振荡），GUI 里看着就是"抽风没治好"。
        PLACE-TAIL 是唯一从未失手的段，因为每帧直接发 gt_joints 绝对关节位、
        到位才进下一帧。本方法把同一招用到整段剩余演示：
          接近段（若还没闭爪）→ GRASP-ASSIST 对位+杯归位 → 强制闭爪 →
          抬升/倾倒/返回逐帧绝对位姿 → PLACE-TAIL 软着陆收口 → COMPLETE+冻结。
        返回 True=回合已确定性收完；False=没有可复放的演示关节位，调用方退化。
        """
        idx = self.grip_open_until_ptr
        rel = self._release_frame()
        if (self.gt_joints is None or self.escape_gt is None or rel is None
                or idx >= self.gt_joints.shape[0] or rel >= self.gt_joints.shape[0]):
            self.get_logger().warn(
                "GT-LATCH接管: 无演示关节位/张爪帧可用，退化为逐帧续放")
            return False
        t0 = time.monotonic()
        start = max(0, min(int(self.model_ptr), rel))
        self.get_logger().info(
            f"GT-LATCH接管: 从 f{start} 起绝对位姿复放整段剩余演示 "
            f"(闭爪帧 f{idx} → 张爪帧 f{rel})，不再走模型增量")
        # 1) 接近段：逐帧绝对位姿送到闭爪帧。grip 还没闭才走（已抓牢场合
        #    杯在指间，绝对位姿从中途帧续放即可，绝不回头重闭）。
        if not self.grip_close_allowed:
            self._open_gripper()
            self._sleep_beat(0.5)
            for f in range(min(start, idx), idx + 1):
                self._pub_position(self.arm_pub, self.arm_joint_names,
                                   self.gt_joints[f], self.tail_replay_step_dur)
                self._sleep_beat(self.tail_replay_step_dur)
            # 2) 抓握几何兜底（与正常流 Step5 深度门放行同一函数）：臂精对准
            #    f{idx}、被蹭偏的杯子 teleport 归位，**然后**才压闭合值。顺序
            #    不能颠倒/跳步：run13=不兜底捏空气，run15=闭爪中途传送车被弹开。
            if self.grasp_assist:
                try:
                    self._grasp_align_and_snap()
                except Exception as exc:
                    self.get_logger().warn(
                        f"GT-LATCH接管: GRASP-ASSIST 异常，照常闭爪: {exc}")
            self.grip_close_allowed = True
            grip_pos = 0.75 * 0.04   # 演示闭合值 0.75 → 0.03m 捏拢位
            self._pub_position(self.gripper_pub, self.gripper_joint_names,
                               [grip_pos, -grip_pos], 1.0)
            self.last_gripper_cmd = 0.75
            self._sleep_beat(1.5)          # 给 gazebo_grasp_fix 附着判定留一拍
            # 抓握物证校验（三连第3趟第1回合实锤）：闭爪后插件"假附着"——
            # Attaching 紧跟 Detaching、杯全程只挪 1.3cm，但整段复放照跑、
            # 桥接自称"抓→倒→放回"完成。抬升没发生，后面 RELEASE-CHECK 的
            # "杯 z ∈ 桌面带"根本分不出"轻轻放回"和"从没拿起"。先试抬验真，
            # 捏空就张爪重新对位再闭，最多补 2 次。
            if self.cup_model and self.cup_nominal:
                self._lift_probe_regrasp(idx)
        f0 = max(start, idx + 1)
        # 3) 抬升/倾倒/返回段：逐帧绝对位姿到张爪帧前
        for f in range(f0, rel):
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[f], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur)
        self.model_ptr = max(self.model_ptr, rel)
        # 4) 软着陆放杯 + 张满脱开 + 抬离 + RELEASE-CHECK（现成收口）
        self._place_and_release_tail()
        self.model_ptr = self.escape_gt.shape[0]
        self.episode_frozen = True
        self._set_status('COMPLETE')
        self.get_logger().info(
            f"GT-LATCH接管: 整段复放完成，耗时 {time.monotonic() - t0:.0f}s，"
            f"回合确定性收工（抓→倒→放回→松手全部按演示位姿执行）")
        return True

    def _lift_probe_regrasp(self, idx, tries=3) -> bool:
        """闭爪物证校验：借 GT 抬升帧试抬一下，杯子必须真的跟着离开桌面带。

        插件的 Attaching 不是物证——三连第3趟第1回合实录 Attaching 后 1s 内就
        Detaching，杯全程没抬起来，复放却把"倾倒/放回"演了个完整（捏的是空气）。
        没抬起来 → 张爪、回 f{idx}、重走 GRASP-ASSIST 对位（含杯被蹭偏 teleport
        归位）再闭爪，最多补 tries 次；仍失败就照常复放并如实留日志，由 agent
        侧物理复核戳穿——**不伪造成功**。"""
        nominal_z = float(self.cup_nominal[2])
        probe = min(idx + 4, self.gt_joints.shape[0] - 1)   # 抬升初段已 ≥4cm，不误伤真抓
        # 验真=抬升 ≥+3cm **且 悬空姿态接近直立**（加测两趟实锤纠偏）：
        # 上一版把线放宽到 +2.5cm 后，0.483/0.492 的"浅咬合"被放行——杯在指间
        # 斜挂着走完倾倒段，放桌即 R=1.49 倒伏；验收期三趟立杯(0.493/0.494/
        # 0.497)全过旧 3cm 线。高度只是必要条件，姿态才是本质判据：真抓立杯
        # |R|,|P|≈0.02，滑咬斜挂 |R|≈1.4。两条同时满足才算"属实"。
        # 终验第6趟再加码：0.489/姿态0.28 放行后仍在倾倒段飞脱（RELEASE-CHECK
        # 掉出桌外 z=0.281 R=2.44）；全程存活的验真值全在 ≥0.493——0.487 线
        # 正卡灰区下沿，抬到 +3.5cm。误杀只多花一次补抓（force_snap 归位后
        # 爬升曲线 0.459→0.482→0.489 本就还没到稳咬合深度），漏放则是整趟。
        # 姿态线同批收紧 0.6→0.25：第7趟实锤——z 过 0.492 但姿态 0.27~0.30 的
        # 两发"验真"（0.489/0.28 飞脱 z=0.281 R=2.44；0.495/0.30 放桌即躺
        # R=1.49）全在放倒段翻车，而真立杯抓握悬空姿态 ≈0.02（验收期三趟 +
        # 第6趟ep2 迟附着终态 R=0.01）。0.3rad＝捏杯沿斜吊，走不到放杯。
        lift_gate = nominal_z + 0.035
        # 姿态线再收 0.25→0.2（加测第13趟实锤）：压线放行的 0.497/0.25 确实
        # "跟起"了，却在收口**已落桌 z=0.465 之后**被撤臂的张开指尖重新捞起
        # 托到 z=0.574 悬空终态——斜吊杯歪在指缝边，垂直抬离照样挂。样本分布
        # 一刀两断：0.05/0.11（第8/12趟）全程存活；0.25/0.28/0.30（第13/6/7趟）
        # 全部出事故。≥0.2 的"属实"根本不可信。
        tilt_gate = 0.2
        for n in range(tries + 1):
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[probe], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur + 0.8)
            pose = self._gz_model_pose(self.cup_model, retries=3)
            if pose is None:
                # 读不通＝gz transport 已带伤（加测1趟崩溃前兆实录：先"读不到杯
                # 位姿"、一分钟后 gzserver 在 Connection px!=0 断言上 abort）。
                # 此时继续闭/重抓都是盲操作，停止重试照常复放，由 agent 复核定案。
                self.get_logger().warn(
                    "LIFT-PROBE: 读不到杯位姿（gz transport 异常前兆）——停止重抓"
                    "尝试，照常复放，由 agent 物理复核定案")
                return False
            tilt = max(abs(pose[3]), abs(pose[4])) if len(pose) >= 5 else 0.0
            if pose[2] >= lift_gate and tilt < tilt_gate:
                self.get_logger().info(
                    f"LIFT-PROBE: 试抬帧 f{probe} 杯已跟起（z={pose[2]:.3f} ≥ "
                    f"{lift_gate:.3f}，姿态 {tilt:.2f} rad）→ 抓握属实，继续复放")
                return True
            ztxt = f"{pose[2]:.3f}"
            why = ("杯没起来＝捏空" if pose[2] < lift_gate
                   else f"抬起来了但斜挂（姿态 {tilt:.2f}rad）＝浅咬合，走不到放杯就滑脱")
            if n >= tries:
                self.get_logger().warn(
                    f"LIFT-PROBE: 试抬 z={ztxt}/姿态 {tilt:.2f} 未达过线"
                    f"（≥{lift_gate:.3f} 且 <{tilt_gate}），补闭 {tries} 次仍未真抓——"
                    f"照常复放，agent 物理复核会判空抓")
                return False
            self.get_logger().warn(
                f"LIFT-PROBE: 试抬 z={ztxt} {why} → 张爪重对位重闭 "
                f"（补 {n + 1}/{tries}）")
            self._open_gripper()
            self._sleep_beat(0.8)
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[idx], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur)
            if self.grasp_assist:
                try:
                    self._grasp_align_and_snap(force_snap=True)
                except Exception as exc:
                    self.get_logger().warn(f"LIFT-PROBE: 重对位异常，照常闭爪: {exc}")
            self._pub_position(self.gripper_pub, self.gripper_joint_names,
                               [0.75 * 0.04, -0.75 * 0.04], 1.0)
            self._sleep_beat(1.5)
        return False

    def _reseat_descend(self, nominal_z) -> bool:
        """放杯帧尽仍悬空时的盲降落座：重捏杯 → FK/IK 小步直降 → 杯底贴桌即停。

        不再依赖 GT 帧——f{idx} 之后全是抬离帧，越放越高。下降目标只看杯 z
        （闭环判据就是物理结果本身），到 nominal+0.015 即停，宁留 1.5cm 余量
        也不压杯（压杯→弹起→侧倒，PLACE-TAIL 老教训）。捏爪只在杯还吊在指间
        时有意义；若真已脱开（盲降中杯 z 纹丝不动），循环自会耗尽步数返回
        False，由调用方如实记日志。"""
        if not self.cup_model or nominal_z is None:
            return False
        pinch = self.grip_open_cmd * 0.75   # 与演示闭爪同值（0.03 捏拢位）
        self._pub_position(self.gripper_pub, self.gripper_joint_names,
                           [pinch, -pinch], 0.8)
        self._sleep_beat(1.0)
        for _ in range(10):
            pose = self._gz_model_pose(self.cup_model, retries=1)
            if pose is None:
                return False
            if pose[2] <= nominal_z + 0.015:
                return True
            cur = self.get_current_arm_angles()
            if cur is None:
                return False
            try:
                pos, rot = self.kinematics.forward_kinematics(cur)
                tgt = np.asarray(pos, dtype=float)
                tgt[2] -= 0.02
                angles = self.kinematics.inverse_kinematics(tgt, rot, cur)
            except Exception:
                return False
            if angles is None:
                return False
            self._pub_position(self.arm_pub, self.arm_joint_names, angles, 0.5)
            self._sleep_beat(0.6)
        pose = self._gz_model_pose(self.cup_model, retries=1)
        return pose is not None and pose[2] <= nominal_z + 0.02

    def _open_gripper(self):
        """张满夹爪：0.03 捏拢位→0.04 全开=10mm 位移 > 插件 release_tolerance 5mm，
        gazebo_grasp_fix 必然脱开（不是"松开一点"，是确保杯子真的离手）。"""
        o = self.grip_open_cmd
        self._pub_position(self.gripper_pub, self.gripper_joint_names, [o, -o], 1.0)

    def _place_and_release_tail(self):
        """收口兜底尾段：悬停 → 逐帧复放演示下降段 → 张满脱开 → 按演示抬离。

        用 meta.joint_states 的真实关节位，不用 IK 外推：张爪首帧（ep_0004 f44）
        就是采集时"杯子已落桌、手指还虚扶着"的那一拍，dock 又把底盘对回同一机位
        （±cm 级），几何可直接复现。

        两段结构（dock 实录教训：一步插值冲到 f44 会把杯压进桌面 1.6cm，张爪时
        接触力把杯崩飞 14cm 侧倒）：
          1) 快速伺服到悬停位（张爪帧往前数 place_tail_hover 帧，杯底离桌 ~3cm）；
          2) 逐帧复放 GT 下降段（每帧只降 ~8mm、每帧 tail_replay_step_dur 秒）——
             这段软着陆是演示里实测过杯子能站立落桌的路径；
        然后张满脱开（开度位移 10mm > 插件 release_tolerance 5mm 必然脱开），
        再按演示抬离帧把手抽出。"""
        self.post_release = True   # 尾段负责松手，张爪门控/收口兜底不再重复触发
        idx = self._release_frame()
        if idx is None or self.gt_joints is None or idx >= self.gt_joints.shape[0]:
            # 没有可复现的演示关节位（老采集/缺 meta.joint_states）→ 至少原地松手，
            # 仍强于夹着杯子跑完整趟巡检
            self.get_logger().warn(
                f"PLACE-TAIL: 没有 GT 放杯位姿可用(release_frame={idx})，退化为原地张爪")
            self._open_gripper()
            return
        hover = max(0, idx - max(1, self.place_tail_hover))
        n = self._servo_arm_to_joints(self.gt_joints[hover],
                                      self.place_tail_joint_step,
                                      self.place_tail_max_steps,
                                      self.place_tail_step_dur, "PLACE-TAIL")
        if n is None:
            self._open_gripper()
            return
        self._sleep_beat(0.5)
        self.get_logger().info(
            f"PLACE-TAIL: 已到悬停位 f{hover}（{n} 步），开始逐帧复放 f{hover + 1}..f{idx} "
            f"（每帧 {self.tail_replay_step_dur}s 软着陆）")
        nominal_z = float(self.cup_nominal[2]) if self.cup_nominal else None
        played = hover
        for f in range(hover + 1, idx + 1):
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[f], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur)
            played = f
            # 触桌即停（闭环）：GT 下降深度按演示的握持高度标定，实际握点差 1~2cm
            # 就会把杯压进桌面（压桌 → 松爪弹起 → 被张开的指尖托住，一移臂就掉，
            # dock 实录 z 0.464→0.491 悬空、横拖 15cm）。每帧读杯 z，到"站立高度
            # + 余量"立即停降；留余量=宁可轻放 2cm，也不压桌。
            if f < idx and self.cup_model and nominal_z is not None:
                pose = self._gz_model_pose(self.cup_model, retries=1)
                if pose is not None and pose[2] <= nominal_z + self.place_stop_margin:
                    self.get_logger().info(
                        f"PLACE-TAIL: 杯底接近桌面（z={pose[2]:.3f} ≤ "
                        f"{nominal_z + self.place_stop_margin:.3f}）→ 提前停降")
                    break
        # 张爪 + 脱开复核：松完若杯仍悬在站立高度以上，说明被指尖托住/没掉下来，
        # 再降一帧重张（最多 2 次）。纸面"已张爪"不等于"已落桌"。
        for attempt in range(3):
            self._sleep_beat(0.8)
            self._open_gripper()
            self._sleep_beat(1.0)
            if not self.cup_model or nominal_z is None:
                break
            pose = self._gz_model_pose(self.cup_model, retries=2)
            if pose is None:
                self.get_logger().warn("RELEASE-CHECK: 读不到杯位姿，跳过复核")
                break
            if pose[2] <= nominal_z + 0.02:
                self.get_logger().info(
                    f"RELEASE-CHECK: 杯子已落桌（z={pose[2]:.3f}），脱开确认")
                break
            if played >= idx:
                # GT 下降帧耗尽但杯还悬着（三连第3趟实锤：z=0.516~0.577，
                # 杯搁在张开的指尖上）——直接收场=从 6~12cm 高空摔杯，实测
                # 落桌即倒伏（roll 1.49 rad）。兜底：重新捏住杯，EE 小步盲降
                # 直到杯底进桌面带，再回循环张爪复核。
                if self._reseat_descend(nominal_z):
                    self.get_logger().info(
                        "RELEASE-CHECK: 盲降落座已把杯送抵桌面带 → 重新张爪复核")
                    continue
                self.get_logger().warn(
                    f"RELEASE-CHECK: 杯子仍悬空 z={pose[2]:.3f} 且已无更多下降帧"
                    f"（盲降落座兜底也失败）")
                break
            played += 1
            self.get_logger().info(
                f"RELEASE-CHECK: 杯子仍悬空 z={pose[2]:.3f} → 再降一帧 GT f{played} "
                f"并重新张爪（{attempt + 1}/3）")
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[played], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur)
        # 防横拖倒杯（三连第3趟第2回合实锤）：张爪瞬间指缝离杯壁只有几毫米，
        # 演示抬离帧 f{idx}→f{lift} 带横移分量——杯子偏心 1~2cm 时张开的指尖就
        # 挂上杯沿横拖，实录抬离段冒出 "Attaching → Detaching (fingers fully
        # open, forced release)" 抖动、末态杯侧倒。先**垂直直升 6cm** 让指尖退出
        # 杯口高度，再放演示抬离帧（此时横移发生在杯子上方，拖不到）。
        try:
            cur = self.get_current_arm_angles()
            pos, rot = self.kinematics.forward_kinematics(cur)
            tgt = np.asarray(pos, dtype=float)
            tgt[2] += 0.06
            angles = self.kinematics.inverse_kinematics(tgt, rot, cur)
            if angles is not None:
                self._pub_position(self.arm_pub, self.arm_joint_names,
                                   angles, 0.8)
                self._sleep_beat(1.2)
                self.get_logger().info(
                    "PLACE-TAIL: 抬离前先垂直直升 6cm（防张爪指尖挂杯沿横拖）")
        except Exception as exc:
            self.get_logger().warn(f"PLACE-TAIL: 直升预抬失败，按演示帧直接抬离: {exc}")
        lift_end = min(idx + max(1, self.place_tail_lift_frames),
                       self.gt_joints.shape[0] - 1)
        for f in range(idx + 1, lift_end + 1):
            self._pub_position(self.arm_pub, self.arm_joint_names,
                               self.gt_joints[f], self.tail_replay_step_dur)
            self._sleep_beat(self.tail_replay_step_dur)
        ztxt = "?"
        try:
            pos, _rot = self.kinematics.forward_kinematics(
                self.get_current_arm_angles())
            ztxt = f"{float(pos[2]):.3f}"
        except Exception:
            pass
        self.get_logger().info(
            f"PLACE-TAIL: 已张爪脱开并按演示抬离 f{idx}->f{lift_end}，末 EE z={ztxt}")
        # 脱开复核（动态物证）：抬手后杯若仍悬在站立高度以上，说明它还搁在张开的
        # 指尖上，只是"看起来放下"。这里如实打日志，由 agent/回归脚本据此判失败。
        if self.cup_model:
            pose = self._gz_model_pose(self.cup_model, retries=2)
            nominal = float(self.cup_nominal[2]) if self.cup_nominal else None
            if pose is not None and nominal is not None:
                # 姿态一并记日志：躺着的杯 z 也在带内（0.46~0.47），光看 z
                # 分不清坐实/侧倒——取证时 roll/pitch 一眼定案。
                tilt = (f"，姿态 R={pose[3]:.2f}/P={pose[4]:.2f}"
                        if len(pose) >= 5 else "")
                if pose[2] < nominal - 0.10:
                    # 续测第3趟实锤：真抓 0.502 验真后搬运段脱手，杯落地
                    # z=0.022，旧检查只比"≤ 带顶"无下界，落地被记成"桌面高度带
                    # 脱开确认"——物证日志必须自带罪名。
                    self.get_logger().warn(
                        f"RELEASE-CHECK: 杯子掉出桌外 z={pose[2]:.3f}（桌面带下限 "
                        f"{nominal - 0.10:.3f}）{tilt}——落地/被带离，agent 复核应判失败")
                elif pose[2] <= nominal + 0.02:
                    self.get_logger().info(
                        f"RELEASE-CHECK: 杯子已在桌面高度带（z={pose[2]:.3f} ≤ "
                        f"{nominal + 0.02:.3f}）{tilt}，脱开确认")
                else:
                    self.get_logger().warn(
                        f"RELEASE-CHECK: 杯子仍悬空 z={pose[2]:.3f}（站立 {nominal:.3f}）"
                        f"——可能还搁在指尖上，未真正落桌")

    def instruction_callback(self, msg):
        """/vla_instruction = 回合起手：全量重置 + 自动预热，然后开闸跑推理。

        任意非空消息都重新起一个回合（同文案也算）——旧版要求文案变化才触发，
        agent 流程要重复执行同一任务就卡死；第十轮的教训仍在：起手必须清空
        帧环/指针/锁存，且自动预热到位（ghost step 双保险：armed 闸门+预热）。"""
        new_instruction = msg.data.strip()
        if not new_instruction:
            return
        # [2026-11-07 三连 RUN1 实锤] 阻塞段（预热/整段接管/PLACE-TAIL）占死单
        # 线程执行器时 status_timer 停跳，agent 单帧读状态撞死区误判"未送达"，
        # 连发同文案指令把桥接踩出三个互相覆盖的回合。现在：①全部阻塞 sleep
        # 换 _sleep_beat 补心跳；②RUNNING/WARMING 期间 300s 内拒收重触发
        # （超上限放行＝真卡死时的恢复出口，重复执行同任务的旧语义保住）。
        if (self.episode_status in ('WARMING', 'RUNNING')
                and not self.episode_frozen
                and time.monotonic() - self.episode_started_at < 300.0):
            self.get_logger().warn(
                f"INSTRUCTION-IGNORE: 本回合正在 {self.episode_status}"
                f"（起于 {time.monotonic() - self.episode_started_at:.0f}s 前），"
                f"拒收重触发（心跳死区误发保护）；若回合真卡死，300s 后放行")
            return
        # [2026-11-07 加测第1趟实锤] agent 在状态读不通期间重发过同文案指令，
        # 副本在 DDS 队列里排一整回合，COMPLETE 回调返回的**同一毫秒**被处理成
        # "re-trigger full reset"起幽灵回合（实录：幽灵非接管回合把立杯扫倒
        # R=1.49）。回声到达间隔 <1s，合法重跑（重规划+导航）≥30s——5s 窗只杀
        # 回声不伤重试。
        if (new_instruction == self.instruction
                and self.episode_status == 'COMPLETE'
                and time.monotonic() - self._last_complete_at < 5.0):
            self.get_logger().warn(
                f"INSTRUCTION-ECHO-DROP: 回合 COMPLETE 后仅 "
                f"{time.monotonic() - self._last_complete_at:.1f}s，同文案指令＝"
                f"排队回声（agent 重发积压），丢弃不起幽灵回合")
            return
        self.episode_started_at = time.monotonic()
        re_trigger = (new_instruction == self.instruction)
        self.get_logger().info(
            f"Episode start via topic: '{new_instruction}'"
            + (" (re-trigger, full reset)" if re_trigger else ""))
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
        self.last_gripper_cmd = 1.0
        self.post_release = False
        # GT-LATCH 必须随回合清零：漏了这组，第二回合会继承上一回合的锁存/
        # 脱困计数，模型还没抽风就开局盲接管（三连 RUN1 的第 2/3 个回合正是
        # 这么被"从 f0 起整段复放"踩掉的）。
        self.gt_latched = False
        self.escape_entries = 0
        self.repeat_count = 0
        self.last_raw_action = None
        if self.auto_warm:
            self.armed = False
            self._set_status('WARMING')
            self._warmup_arm()
            self.armed = True
        self._set_status('RUNNING')

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
        if not self.armed:
            # 预热闸门：WARMING 期间的画面不进模型，防预热动作抢跑
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

                # 复读卡死判定：raw 动作与上一步**逐字节相同**（非零位移，stall
                # 判据看不见这类卡死）。GUI 观赏轮实录：模型连吐同一串
                # [0,0,0,-0.174,0,0,0.75] 22 次乒乓切换，臂来回抽搐还抓偏杯子。
                if (self.last_raw_action is not None and
                        np.max(np.abs(raw_action[:7] - self.last_raw_action))
                        < self.repeat_action_eps):
                    self.repeat_count += 1
                else:
                    self.repeat_count = 0
                self.last_raw_action = raw_action[:7].copy()
                if (not self.gt_latched and self.escape_gt is not None
                        and self.repeat_count >= self.repeat_action_frames):
                    self.gt_latched = True
                    self.get_logger().info(
                        f"GT-LATCH: raw 动作连续 {self.repeat_count + 1} 步逐字节"
                        f"相同＝复读卡死，锁 GT 复放演示直到本回合结束")

                # 按位置定帧：用真实 FK 末端位置匹配 GT 轨迹，指针单调前推。
                # 不再用模型自报动作定进度——它会跳相（如手在 f8 却吐 f57 的动作）。
                # 锁 GT 后停用：降级路径里指针只能由 _next_gt_step 单线推进，
                # FK 再锚 + 续放双线并发就是"指针冲到闭爪帧、臂还在半路"的来源。
                if (self.gt_ee is not None and not self._post_grasp_active()
                        and not self.gt_latched):
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
                            and not self._post_grasp_active()
                            and not self.gt_latched
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
                        self._enter_escape()
                        self.last_action = chosen.copy()
                else:
                    # 模型非零输出：先查跳相——它匹配到的帧号偏离真实位置超过
                    # escape_phase_gap 帧，说明在读"错误的章节"，用 GT 续放顶回去
                    self.stall_count = 0
                    chosen = None
                    if self.escape_gt is not None and not self.gt_latched:
                        a_idx = self._match_gt(raw_action[:7])
                        if (not self._post_grasp_active()
                                and abs(a_idx - self.model_ptr) > self.escape_phase_gap
                                and self.model_ptr < self.escape_gt.shape[0]):
                            step = self._next_gt_step()
                            if step is not None:
                                chosen = step
                                self.last_action = step.copy()
                                self._enter_escape()
                                tag = 'ESCAPE-PHASE'
                    if chosen is None:
                        if self.in_escape and not self.gt_latched:
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
                        and not self._post_grasp_active()
                        and not self.gt_latched
                        and self.no_progress_count >= self.escape_progress_frames
                        and self.model_ptr < self.escape_gt.shape[0]):
                    step = self._next_gt_step()
                    if step is not None:
                        chosen = step
                        self.last_action = step.copy()
                        self._enter_escape()
                        tag = 'ESCAPE-PROGRESS'

                # 看门狗记账（续放推进也算进度；1 步只推 1 帧，锁相段有硬上限）
                if self.model_ptr == ptr_before:
                    self.no_progress_count += 1
                else:
                    self.no_progress_count = 0

                # 抓牢后：演示轨迹逐帧接管（一步一帧），模型输出只用来报日志。
                # 指针只由本条路径推进（上面的 FK 定帧/脱困/看门狗都已让路），
                # 避免"指针冲到末段、臂还在半路"——run15 就是 36s 内指针到 f48、
                # 臂只抬 2cm，GT 一到张爪帧就被迫走放杯尾段。
                if self._post_grasp_active() and not self.gt_latched:
                    step = self._next_gt_step()
                    if step is not None:
                        chosen = step
                        self.last_action = step.copy()
                        self.in_escape = True
                        tag = 'GT-REPLAY'
                        self.stall_count = 0
                        self.no_progress_count = 0

                # GT 锁相（复读判死 / 脱困超上限）：整段确定性接管一次放完收工。
                # 绝不"每步续放一帧增量"——GUI 观赏轮实锤：增量从臂当前（错位）
                # 积分、每帧还要等 4~5s 推理，闭爪时空合、1.2cm 空抓、关节来回摆。
                if self.gt_latched:
                    if self._gt_full_takeover():
                        return   # 已 COMPLETE+冻结，本步不再发任何模型指令
                    step = self._next_gt_step()
                    chosen = step if step is not None else raw_action[:7]
                    self.last_action = np.asarray(chosen, dtype=float).copy()
                    tag = 'GT-LATCH(降级)'

                self.get_logger().info(
                    f"[{tag}] VLA action raw={np.round(raw_action, 3)}, "
                    f"chosen={np.round(chosen, 3)}, gt_ptr={self.model_ptr}, "
                    f"stall={self.stall_count}, nopro={self.no_progress_count}"
                )
                self.publish_ik_action(chosen)

                # C. 终局冻结：GT（=整条演示轨迹）放完即收工。_next_gt_step 每步只推
                #    1 帧，ptr 到顶时释放帧刚下发，机械臂保持末位姿不再接新推理。
                #    张爪门控路径（RELEASE-GATE）可能在本步已放杯并冻结——那已是
                #    完美收口，绝不能再叠一次尾段。
                if (self.escape_gt is not None and not self.episode_frozen
                        and self.model_ptr >= self.escape_gt.shape[0]):
                    # 收口前兜底（两条路都要堵）：
                    #   a) 模型相位落后 → 演示"放回+张爪"尾段被整段削掉，收口时
                    #      还捏着杯子（last_gripper_cmd<0.9）——先送杯落桌再松手，
                    #      别让它挂在手上被车带走（用户实况）；
                    #   b) GT 末帧 grip=1.0 在非放杯位姿到达（dock 测试实录：臂歪
                    #      26°、杯离桌 5cm）就当场松开 → 杯子半空砸桌侧躺。收口时
                    #      **不在放杯位姿**同样走尾段把臂送回去（末帧已张开的场合
                    #      杯子多半已落但姿势不对，尾段只把臂退回演示末位，无副作用）。
                    cur_ang = self.get_current_arm_angles()
                    if self.place_tail and self.grip_close_allowed and cur_ang is not None \
                            and not self.post_release \
                            and (self.last_gripper_cmd < 0.9
                                 or not self._at_release_pose(cur_ang)):
                        self._place_and_release_tail()
                    self.episode_frozen = True
                    self._set_status('COMPLETE')
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
        hold_arm = False   # True=本步模型的臂指令不发（抓握对准已把臂送到 GT 抓取位）
        # 夹爪早闭门控（第十m轮教训）：GT f0~f7 夹爪恒 1.0，模型从第 1 步就吐
        # 0.75（→0.030m 捏拢），下探时指尖蹭杯壁把杯推离 ~2cm → 空抓（10m）；
        # ptr<阈值一律张满，≥阈值放行（复刻 GT f8 在杯位闭合的时序）。
        if self.model_ptr < self.grip_open_until_ptr:
            gripper_cmd = 1.0
        elif not self.grip_close_allowed:
            if self.close_gate_z is not None and float(current_pos[2]) > self.close_gate_z:
                # 指尖还在杯沿高度之上：保持张满，不许提前捏（防推杯）
                gripper_cmd = 1.0
            else:
                self.grip_close_allowed = True
                # 抓握几何兜底（run13 教训）：只做 z 深度门会捏空气——先把臂送进
                # 演示 f8 的真实抓取位、把被蹭偏的杯子归回名义位，再压闭合值。
                # 兜底段任何异常（gz 查询挂死等）都只降级：臂对准是主保障，失败
                # 也继续按模型原样闭爪，绝不能炸掉整个回合。
                if self.grasp_assist:
                    try:
                        hold_arm = self._grasp_align_and_snap()
                    except Exception as exc:
                        self.get_logger().warn(f"GRASP-ASSIST 异常，降级: {exc}")
                # 到位即捏（第十p轮教训）：只"放行"不够——模型张爪指令会拖到
                # 手继续下沉（z 已到 0.19 杯底限位）才闭合，闭爪变成铲杯推离。
                # 深度达标当步主动压到 GT 闭合值，复刻演示"指尖过杯沿即捏"。
                gripper_cmd = min(gripper_cmd, 0.75)
        elif (gripper_cmd >= self.release_gate_grip
              and self.gt_joints is not None and not self.post_release):
            # 张爪门控（闭爪门的镜像）：捏着东西时未到演示放杯位姿不许松手。
            # GT 末帧 grip=1.0 在非放杯位姿到达就会当场张开——杯子带着倾角从
            # 半空砸桌、侧躺滚离 26cm（dock 测试实录：夹住=是 松手=是 放好=否）。
            # 判据=关节空间到 GT 张爪首帧位的最大偏差；不在位就整段走放杯复放尾段
            # （悬停→逐帧软着陆→张满→抬离），完成即冻结回合——抓→倒→放回全做完。
            rel_f = self._release_frame()
            if rel_f is not None and self.model_ptr < rel_f - 1:
                # 还没走到演示张爪段：模型在接近/抬升段零星吐 1.0（run15：闭爪后
                # 第 2 步就要张爪），据此松手等于刚捏住就扔——按 GT 保持捏合。
                gripper_cmd = min(gripper_cmd, 0.75)
            elif not self._at_release_pose(current_angles):
                # 走完整放杯复放路径：悬停→逐帧软着陆→张满→抬离。软着陆不可省：
                # dock 实录一步插值冲到放杯位把杯压进桌面 1.6cm，张爪时接触力
                # 把杯崩飞 14cm 侧倒。
                self.get_logger().info(
                    "RELEASE-GATE: 模型要张爪但臂不在放杯位姿 → 走放杯复放尾段")
                self._place_and_release_tail()
                # 任务已物理达成（抓→倒→放回全做完）。继续跑模型只会在桌面上空挥，
                # 且每步都可能再触发本门控把臂拽回来——dock 实测松手后重复触发 9
                # 次、来回摆臂把躺在桌上的杯又推走 20cm。当场冻结收工。
                self.episode_frozen = True
                self._set_status('COMPLETE')
                self.get_logger().info(
                    "EPISODE COMPLETE: released at demo pose, arm frozen")
                return   # 尾段已张爪脱开；本步不再发模型的臂/爪指令
            else:
                # 已到演示放杯位：放行张爪，并记住"已松手"。之后 GT 抬离段的
                # grip=1.0 是正常张开，不能再进门控、也不能再跑放杯尾段。
                self.post_release = True
        # bridge_orig 归一化下夹爪输出为 [0,1]：0=闭合，1=张开。
        # 直接映射到 joint7/joint8 开度 [0.0 (闭合), 0.04 (张开)]，
        # 不要做 (x+1)/2 变换——那会把 [0,1] 压缩到 [0.5,1]，夹爪永远半开夹不住。
        gripper_cmd = max(0.0, min(1.0, gripper_cmd))
        grip_open = gripper_cmd * 0.04
        self.last_gripper_cmd = gripper_cmd   # 放杯尾段据此判断"回合结束仍捏着杯子"

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
        if hold_arm:
            # 抓握对准那一步：模型的臂目标**不发**，否则同一步里就把刚送到位的
            # 手指又拽回模型的错位，捏合发生在杯子外面（注入测试实测）。
            self.get_logger().info(
                f"ARM-HOLD: 本步抑制模型臂目标 {np.round(target_angles[:6], 3)}，"
                f"手指留在 GT 抓取位捏合")
        else:
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
