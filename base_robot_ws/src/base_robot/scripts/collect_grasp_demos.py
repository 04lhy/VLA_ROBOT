#!/usr/bin/env python3
"""自动化抓取 + OpenVLA LoRA 微调数据采集脚本（仿真专用）。

思路：用脚本（而非 VLA）驱动机械臂完成一次完整的“伸到杯子 -> 抓取 -> 抬起”，
同时在每个控制步把「腕部相机图像 + 末端 7 维增量动作 + 语言指令」记录下来，
产出 OpenVLA LoRA 微调所需的 (image, action, instruction) 三元组。

动作约定（与 vla_gazebo_bridge.py 完全一致，保证微调后推理可直接对接）：
  action = [dx, dy, dz, rx, ry, rz, gripper]
    - dx/dy/dz  : arm_base_link 系下的末端位置增量（米）
    - rx/ry/rz  : arm_base_link 系下的轴角姿态增量（弧度），满足 R_next = delta_R @ R_cur
    - gripper   : [0,1]，0=闭合，1=张开（bridge_orig 约定）

输出：每个 episode 一个目录
  demos/ep_0000/
      images.npy    (T, H, W, 3) uint8
      actions.npy   (T, 7) float32
      meta.json     {instruction, ee_poses, joint_states}

用法：
  # 预览规划结果（不动机器人，只打印 FK/IK 位姿）
  ros2 run base_robot collect_grasp_demos.py --dry-run

  # 采集 20 条演示（默认）
  ros2 run base_robot collect_grasp_demos.py --episodes 20 --out ~/vla_demos

前置：base_robot.launch.py 已启动（Gazebo + 控制器 + robot_state_publisher）。
"""

import argparse
import json
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cv_bridge import CvBridge
import cv2

import PyKDL
from kdl_parser_py.urdf import treeFromString
from transforms3d.axangles import mat2axangle

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False

# 关节限位（来自 arm.xacro），IK 结果会被夹到这个范围内
JOINT_LIMITS = {
    "joint1": (-2.618, 2.618),
    "joint2": (0.0, 3.14),
    "joint3": (-2.697, 0.0),
    "joint4": (-1.832, 1.832),
    "joint5": (-1.22, 1.22),
    "joint6": (-3.14, 3.14),
}


class KDLKinematics:
    """KDL 正/逆运动学（arm_base_link -> link6）。"""

    def __init__(self, urdf_string, arm_joint_names, base_link="arm_base_link",
                 tip_link="link6"):
        ok, tree = treeFromString(urdf_string)
        if not ok:
            raise RuntimeError("URDF -> KDL tree 解析失败")
        self.chain = tree.getChain(base_link, tip_link)
        self.num_joints = min(self.chain.getNrOfJoints(), len(arm_joint_names))
        self.joint_names = arm_joint_names
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver_pos = PyKDL.ChainIkSolverPos_NR(
            self.chain, self.fk_solver, self.ik_solver_vel, maxiter=500, eps=1e-5)
        self.ik_solver_lma = PyKDL.ChainIkSolverPos_LMA(self.chain)

    def forward(self, angles):
        arr = PyKDL.JntArray(self.num_joints)
        for i, a in enumerate(angles[:self.num_joints]):
            arr[i] = a
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(arr, frame)
        pos = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        rot = np.array([[frame.M[i, j] for j in range(3)] for i in range(3)])
        return pos, rot

    def inverse(self, target_pos, target_rot, seed_angles):
        seed = PyKDL.JntArray(self.num_joints)
        for i, a in enumerate(seed_angles[:self.num_joints]):
            seed[i] = a
        frame = PyKDL.Frame(
            PyKDL.Rotation(target_rot[0, 0], target_rot[0, 1], target_rot[0, 2],
                           target_rot[1, 0], target_rot[1, 1], target_rot[1, 2],
                           target_rot[2, 0], target_rot[2, 1], target_rot[2, 2]),
            PyKDL.Vector(target_pos[0], target_pos[1], target_pos[2]),
        )
        result = PyKDL.JntArray(self.num_joints)
        if self.ik_solver_pos.CartToJnt(seed, frame, result) >= 0:
            return [result[i] for i in range(self.num_joints)]
        if self.ik_solver_lma.CartToJnt(seed, frame, result) >= 0:
            return [result[i] for i in range(self.num_joints)]
        return None


def clamp_angles(angles):
    out = list(angles)
    for i, name in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
        lo, hi = JOINT_LIMITS[name]
        out[i] = max(lo, min(hi, out[i]))
    return out


def pose_delta_action(pos_cur, rot_cur, pos_next, rot_next, gripper_cmd):
    """计算 arm_base_link 系下的 7 维增量动作，与 bridge 的叠加方向一致。"""
    dpos = pos_next - pos_cur
    dR = rot_next @ rot_cur.T
    axis, angle = mat2axangle(dR)
    drot = axis * angle
    return np.array([dpos[0], dpos[1], dpos[2],
                     drot[0], drot[1], drot[2], float(gripper_cmd)], dtype=np.float32)


class DemoCollector(Node):
    def __init__(self, args):
        super().__init__("collect_grasp_demos")
        self.args = args

        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 运动学：从 /robot_description 话题取 URDF（robot_state_publisher 会 latch 发布）。
        # 必须用 TRANSIENT_LOCAL durability 才能收到早于本节点启动的那条 latch 消息。
        self._robot_description = None
        desc_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(String, "/robot_description", self._desc_cb, desc_qos)
        self.get_logger().info("等待 /robot_description ...")
        t0 = time.time()
        while self._robot_description is None and rclpy.ok():
            if time.time() - t0 > 30.0:
                # 回退：直接从安装目录读预生成的 URDF 文件
                self._robot_description = self._read_urdf_file()
                if not self._robot_description:
                    self.get_logger().error("/robot_description 超时且无本地 URDF 文件")
                    raise RuntimeError("无法获取 robot_description")
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.kin = KDLKinematics(self._robot_description, self.arm_joint_names)

        # 发布者
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        # 订阅者
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(Image, args.camera_topic, self._image_cb, 10)

        self.joint_pos = {}
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

        # TF（用于把杯子世界坐标转到 arm_base_link）
        self.tf_buffer = None
        self.tf_listener = None
        if HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # 等待关节状态就绪
        self.get_logger().info("等待 /joint_states 就绪...")
        self._wait_until(lambda: all(n in self.joint_pos for n in self.arm_joint_names),
                         "关节状态就绪")

    # ---------- 回调 ----------
    def _desc_cb(self, msg):
        if self._robot_description is None:
            self._robot_description = msg.data
            self.get_logger().info(f"/robot_description 已获取（{len(msg.data)} 字节）")

    def _read_urdf_file(self):
        """回退：读安装目录里预生成的 URDF（并用 xacro 处理宏，若仍是 .xacro）。"""
        import subprocess
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("base_robot")
            candidates += [
                os.path.join(share, "urdf", "4_robot", "4_robot.urdf"),
                os.path.join(share, "urdf", "4_robot", "4_robot.urdf.xacro"),
            ]
        except Exception:
            pass
        candidates += [
            "/home/lhy/Desktop/VLA_ROBOT/base_robot_ws/install/base_robot/share/base_robot/urdf/4_robot/4_robot.urdf",
        ]
        for path in candidates:
            if os.path.exists(path):
                try:
                    if path.endswith(".xacro"):
                        out = subprocess.run(["xacro", path], capture_output=True, text=True, timeout=30)
                        if out.returncode == 0:
                            return out.stdout
                    else:
                        with open(path) as f:
                            return f.read()
                except Exception:
                    continue
        return None

    def _image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_pos[name] = msg.position[i]

    # ---------- 工具 ----------
    def _wait_until(self, cond, what, timeout=30.0):
        t0 = time.time()
        while rclpy.ok() and not cond():
            if time.time() - t0 > timeout:
                self.get_logger().warn(f"等待超时：{what}")
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def get_angles(self):
        return [self.joint_pos[n] for n in self.arm_joint_names]

    def get_camera_image(self, timeout=1.0):
        t0 = time.time()
        while self.latest_image is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.latest_image

    # ---------- 杯子位姿 ----------
    def _gz_pose(self, model_name):
        """用 `gz model -m <name> -i` 读取世界位姿 (x,y,z,roll,pitch,yaw)。"""
        import re
        import subprocess
        try:
            out = subprocess.run(
                ["gz", "model", "-m", model_name, "-i"],
                capture_output=True, text=True, timeout=5.0).stdout
        except (subprocess.TimeoutExpired, OSError):
            return None
        m = re.search(r"pose \{\s*position \{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)",
                      out)
        if not m:
            return None
        xyz = [float(m.group(i)) for i in (1, 2, 3)]
        r = re.search(r"orientation \{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)\s*w:\s*([-\d.eE+]+)", out)
        if r:
            q = [float(r.group(i)) for i in (1, 2, 3, 4)]
        else:
            q = [0.0, 0.0, 0.0, 1.0]
        return xyz + q

    def _quat_to_rot(self, q):
        from transforms3d.quaternions import quat2mat
        return np.array(quat2mat([q[3], q[0], q[1], q[2]]))

    def get_cup_in_arm_base(self):
        """把杯子世界坐标转到 arm_base_link 系。优先用 TF，失败退回 gz pose。"""
        cup = self._gz_pose(self.args.cup_model)
        if cup is None:
            self.get_logger().error(f"读不到 {self.args.cup_model} 位姿")
            return None, None
        cup_p = np.array(cup[:3])
        cup_R = self._quat_to_rot(cup[3:])

        # 尝试 TF：world -> arm_base_link
        if self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform("world", "arm_base_link", rclpy.time.Time())
                trans = t.transform.translation
                rot = t.transform.rotation
                from transforms3d.quaternions import quat2mat
                R_w2a = np.array(quat2mat([rot.w, rot.x, rot.y, rot.z]))
                p_w2a = np.array([trans.x, trans.y, trans.z])
                cup_arm = R_w2a.T @ (cup_p - p_w2a)
                # 杯子姿态也转过去（用于定义抓取朝向时可能需要）
                cup_R_arm = R_w2a.T @ cup_R
                self.get_logger().info(f"[TF] 杯子在 arm_base_link: {np.round(cup_arm, 3)}")
                return cup_arm, cup_R_arm
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF 查 world->arm_base_link 失败：{e}，退回静态假设")

        # 退回：机器人固定出生，杯子在正前方 ~0.354m（本世界默认）
        self.get_logger().warn("使用硬编码的杯子相对位置（arm_base_link 前方 +X）")
        return np.array([self.args.cup_default_x, self.args.cup_default_y, self.args.cup_default_z]), np.eye(3)

    # ---------- 抓取规划 ----------
    def plan_grasp(self):
        """返回关节空间 waypoints（针对当前固定场景标定，可用 --joint-* 覆盖）。"""
        cup_arm, _ = self.get_cup_in_arm_base()
        if cup_arm is not None:
            self.get_logger().info(f"杯子@arm_base {np.round(cup_arm, 3)}")

        def _parse(s):
            return [float(x) for x in s.split(",")]

        waypoints = {
            "pre_grasp": _parse(self.args.joint_approach),
            "grasp": _parse(self.args.joint_grasp),
            "lift": _parse(self.args.joint_lift),
        }
        # 倾倒位姿：在抬升位姿基础上旋转腕部 joint6（原地 roll，末端位置不变），把杯子倾倒
        tilt = list(waypoints["lift"])
        tilt[5] = tilt[5] + self.args.tilt_angle
        waypoints["tilt"] = tilt
        for k, v in waypoints.items():
            p, _ = self.kin.forward(v)
            self.get_logger().info(f"  {k}: joints={np.round(v, 3)} -> eef={np.round(p, 3)}")
        return waypoints

    # ---------- 运动执行 ----------
    def publish_arm(self, angles, duration=1.0):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in angles]
        pt.velocities = [0.0] * len(angles)
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec
        traj.points.append(pt)
        self.arm_pub.publish(traj)

    def publish_gripper(self, opening, duration=0.5):
        """opening: [0, 0.04]，0.04=张开。"""
        traj = JointTrajectory()
        traj.joint_names = ["joint7", "joint8"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(opening), float(-opening)]
        pt.velocities = [0.0, 0.0]
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec
        traj.points.append(pt)
        self.grip_pub.publish(traj)

    def _interp(self, a, b, steps):
        """线性插值（关节空间），含端点。steps>=1。"""
        a = np.array(a)
        b = np.array(b)
        return [a + (b - a) * (i / max(1, steps - 1)) for i in range(steps)]

    # ---------- 主流程 ----------
    def run_episode(self, ep_idx, instruction):
        home = [0.0] * 6
        # 先回 home 并张开夹爪
        self.publish_arm(home, 3.0)
        self.publish_gripper(0.04, 1.0)
        time.sleep(4.0)

        waypoints = self.plan_grasp()
        if waypoints is None or "grasp" not in waypoints:
            self.get_logger().error("规划失败，跳过本 episode")
            return None

        # 组装 waypoint 序列：(关节角, 夹爪开度 0/0.04)
        # 开度 0.04 -> gripper 动作 1.0；0.0 -> 0.0
        grip = self.args.grip_open  # 夹到杯子宽度，而不是夹到 0（否则会把 6.6cm 杯子撞倒）
        # 水平前伸：夹爪已在杯子前方、与杯子同高，直接平移把杯子套进夹爪中间，
        # 不再先抬到杯子上方再压下去
        seq = [
            (home, 0.04),
            (waypoints["grasp"], 0.04),   # 水平前伸（平移），杯子进入夹爪中间
            (waypoints["grasp"], grip),   # 闭合夹住
            (waypoints["lift"], grip),    # 抬起
            (waypoints["tilt"], grip),    # 倾斜（倾倒）
            (waypoints["tilt"], grip),    # 保持倾斜（本段 a==b，原地停留 ~seg_duration 秒）
            (waypoints["lift"], grip),    # 复原竖直
            (waypoints["grasp"], grip),   # 下降放回桌面
            (waypoints["grasp"], 0.04),   # 松开夹爪（0.04=张到最大），杯子落回桌面
            (home, 0.04),                 # 机械臂原路退回 home
        ]
        if "lift" not in waypoints:
            seq = [s for s in seq if s[0] is not None]

        # 每个 waypoint 段插值的步数（控制采样率约 1-2 Hz）
        steps_per_seg = max(2, int(round(self.args.seg_duration / self.args.control_dt)))

        images = []
        actions = []
        ee_poses = []
        joints = []

        cur_angles = self.get_angles()
        last_grip = None

        for wi in range(len(seq) - 1):
            a_angles, a_grip = seq[wi]
            b_angles, b_grip = seq[wi + 1]
            interp = self._interp(a_angles, b_angles, steps_per_seg)

            for step in range(len(interp)):
                next_angles = list(interp[step])
                # 夹爪在段开始时即切换（闭合段会立即闭合）
                grip_open = b_grip

                # 记录：当前图像 + 当前->下一步的末端增量
                image = self.get_camera_image()
                p_cur, R_cur = self.kin.forward(cur_angles)
                p_next, R_next = self.kin.forward(next_angles)
                # 夹爪动作值映射到 [0,1]（bridge_orig：0=闭合，1=张开）
                grip_cmd = 1.0 if grip_open > 0.035 else 0.0
                action = pose_delta_action(p_cur, R_cur, p_next, R_next, grip_cmd)

                if self.args.record and image is not None:
                    images.append(image)
                    actions.append(action)
                    ee_poses.append(np.concatenate([p_cur, [0, 0, 0, 1.0]]))
                    joints.append(cur_angles)

                # 执行（夹爪只在开度变化时下发）
                self.publish_arm(next_angles, self.args.control_dt)
                if last_grip is None or abs(grip_open - last_grip) > 1e-4:
                    self.publish_gripper(grip_open, 0.3)
                    last_grip = grip_open
                time.sleep(self.args.control_dt)
                rclpy.spin_once(self, timeout_sec=0.01)

                cur_angles = next_angles

        # 结束：最后一个 seq 步骤已松开夹爪，杯子落回原位

        if not self.args.record:
            return None

        return {
            "images": np.stack(images) if images else None,
            "actions": np.stack(actions) if actions else None,
            "ee_poses": np.stack(ee_poses) if ee_poses else None,
            "joints": np.stack(joints) if joints else None,
            "instruction": instruction,
        }

    def save_episode(self, ep_idx, data):
        out_dir = os.path.join(self.args.out, f"ep_{ep_idx:04d}")
        os.makedirs(out_dir, exist_ok=True)
        # 缩小到 320x240 存，OpenVLA 反正会 resize 到 224，省空间
        if data["images"] is not None:
            imgs = data["images"]
            small = np.array([cv2.resize(im, (320, 240)) for im in imgs], dtype=np.uint8)
            np.save(os.path.join(out_dir, "images.npy"), small)
        np.save(os.path.join(out_dir, "actions.npy"), data["actions"].astype(np.float32))
        meta = {
            "instruction": data["instruction"],
            "ee_poses": data["ee_poses"].tolist() if data["ee_poses"] is not None else None,
            "joints": data["joints"].tolist() if data["joints"] is not None else None,
        }
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f"已保存 episode {ep_idx}: {out_dir} "
                               f"({data['actions'].shape[0]} 步)")

    def run(self):
        if self.args.dry_run:
            self.get_logger().info("=== dry-run：仅打印规划结果 ===")
            home = [0.0] * 6
            self.publish_arm(home, 3.0)
            time.sleep(4.0)
            self.plan_grasp()
            return

        os.makedirs(self.args.out, exist_ok=True)
        for ep in range(self.args.episodes):
            self.get_logger().info(f"===== Episode {ep} =====")
            if self.args.randomize_cup:
                self._randomize_cup(ep)
            data = self.run_episode(ep, self.args.instruction)
            if data is not None:
                self.save_episode(ep, data)

    def _randomize_cup(self, seed_idx):
        """在默认位置附近小范围随机平移杯子，增加数据多样性。"""
        import subprocess
        rng = np.random.default_rng(seed_idx)
        dx = float(rng.uniform(-0.05, 0.05))
        dy = float(rng.uniform(-0.05, 0.05))
        # 用 gz model 设置模型位姿（保持默认朝向）
        base = self._gz_pose(self.args.cup_model)
        if base is None:
            return
        x, y, z = base[0] + dx, base[1] + dy, base[2]
        subprocess.run(["gz", "model", "-m", self.args.cup_model,
                        "-x", f"{x}", "-y", f"{y}", "-z", f"{z}"],
                       capture_output=True, timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description="自动化抓取 + LoRA 数据采集")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--out", default=os.path.expanduser("~/vla_demos"))
    parser.add_argument("--instruction", default="pick up the cup")
    parser.add_argument("--camera-topic", default="/camera_sensor/image_raw")
    parser.add_argument("--cup-model", default="plastic_cup_0")
    parser.add_argument("--cup-half-height", type=float, default=0.065,
                        help="杯子模型原点到杯体中心的高度偏移（米）")
    parser.add_argument("--palm-offset", type=float, default=0.135,
                        help="掌心(link6)到抓取点的竖直距离（手指长度）")
    parser.add_argument("--approach-height", type=float, default=0.15,
                        help="预抓取在抓取点上方的高度（米）")
    parser.add_argument("--lift-height", type=float, default=0.15,
                        help="抬起高度（米）")
    parser.add_argument("--seg-duration", type=float, default=2.0,
                        help="每段轨迹时长（秒）")
    parser.add_argument("--control-dt", type=float, default=0.5,
                        help="控制/采样周期（秒）")
    parser.add_argument("--cup-default-x", type=float, default=0.354)
    parser.add_argument("--cup-default-y", type=float, default=0.0)
    parser.add_argument("--cup-default-z", type=float, default=0.137)
    parser.add_argument("--joint-approach", default="0,1.34,-0.80,0,-0.45,0",
                        help="预抓取关节角（6 个，逗号分隔）：手指水平前伸、位于杯子上方")
    parser.add_argument("--joint-grasp", default="0,1.66,-0.52,0,-1.05,0",
                        help="抓取关节角（6 个，逗号分隔）：手指水平，杯子在夹爪中间")
    parser.add_argument("--joint-lift", default="0,1.42,-0.68,0,-0.65,0",
                        help="抬起关节角（6 个，逗号分隔）")
    parser.add_argument("--tilt-angle", type=float, default=1.57,
                        help="倾倒时腕部 joint6 的旋转角（弧度，默认 1.57≈90°；负号反方向）")
    parser.add_argument("--grip-open", type=float, default=0.03,
                        help="闭合夹爪时的开度（0.04=8cm 全开，0.03=6cm 夹住 6.6cm 杯子）")
    parser.add_argument("--randomize-cup", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args()
    args.record = not args.no_record

    rclpy.init()
    node = DemoCollector(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
