#!/usr/bin/env python3
"""多房间循环抓取 + OpenVLA LoRA 微调数据采集脚本（仿真专用）。

思路：
  组长的死动作（固定关节角）只在「杯子相对 arm_base 位置固定」时有效——
  即杯子必须落在 arm_base 正前方 ~0.354m 处。本脚本把采集组织成「循环」：

    一个循环 = 16 次夹取（4 张桌子 × 4 条边：东/南/西/北）
    每次：先把杯子摆到某条边 -> 把机器人 teleport 到对应「停靠位」
          （杯子正好在正前方 0.354m）-> 跑一遍死动作 -> 边跑边记录三元组

  停靠位表 GRASP_STATIONS = 4 张桌子 × 4 条边，共 16 个。每条边的朝向：
    南边朝北(yaw=+90°)、北边朝南(yaw=-90°)、东边朝西(yaw=180°)、西边朝东(yaw=0°)

动作约定（与 vla_gazebo_bridge.py 完全一致）：
  action = [dx, dy, dz, rx, ry, rz, gripper]
    - dx/dy/dz  : arm_base_link 系下的末端位置增量（米）
    - rx/ry/rz  : arm_base_link 系下的轴角姿态增量（弧度），R_next = delta_R @ R_cur
    - gripper   : [0,1]，0=闭合，1=张开

输出：每个 episode 一个目录
  demos/ep_XXXX/
      images.npy    (T, H, W, 3) uint8
      actions.npy   (T, 7) float32
      meta.json     {instruction, ee_poses, joint_states, is_first, is_last, station}

用法：
  # 干跑：只打印规划和停靠位，不动机器人
  ros2 run base_robot collect_4room_demos.py --dry-run

  # 采集 1 个循环（16 条 demo，覆盖 4 桌 × 4 边）
  ros2 run base_robot collect_4room_demos.py --cycles 1 --out ~/vla_demos

  # 只采集某个停靠位（调试用，--stations 取停靠位表下标 0-15）
  ros2 run base_robot collect_4room_demos.py --cycles 1 --stations 0

前置：base_robot.launch.py 已启动（Gazebo + 控制器 + robot_state_publisher）。
"""

import argparse
import json
import math
import os
import subprocess
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

# 机械臂在 Gazebo 里的 model 名（teleport 目标）
ROBOT_MODEL = "base_robot"

# 16 个抓取停靠位表 = 4 张桌子 × 4 条边（东/南/西/北）。
# 每条边保证：把杯子摆到该边后，杯子正好落在 arm_base 正前方 0.354m（死动作可命中）。
# 几何：以桌心 (cx, cy) 为基准，杯子到桌心距离 d=0.35m，机器人到杯子距离 g=0.354m。
# 元组：(站名, 杯子 model 名, 机器人停靠 x,y,z,yaw弧度, 杯子摆放 x,y,z,杯子yaw弧度)
# 朝向：南边朝北(+90°)、北边朝南(-90°)、东边朝西(180°)、西边朝东(0°)
GRASP_STATIONS = [
    # ---- 西南(红) cafe_table (-17.023, -10.1785) ----
    ("southwest_red_south", "plastic_cup_0",        -17.023, -10.883, 0.0,  1.5708,  -17.023, -10.529, 0.456645, 0.0),
    ("southwest_red_north", "plastic_cup_0",        -17.023,  -9.475, 0.0, -1.5708,  -17.023,  -9.829, 0.456645, 0.0),
    ("southwest_red_east",  "plastic_cup_0",        -16.319, -10.179, 0.0,  3.1416,  -16.673, -10.179, 0.456645, 0.0),
    ("southwest_red_west",  "plastic_cup_0",        -17.727, -10.179, 0.0,  0.0,     -17.373, -10.179, 0.456645, 0.0),
    # ---- 东南(蓝) cafe_table_0 (14.1519, -11.1643) ----
    ("southeast_blue_south", "plastic_cup_0_clone",  14.152, -11.868, 0.0,  1.5708,   14.152, -11.514, 0.456645, 0.0),
    ("southeast_blue_north", "plastic_cup_0_clone",  14.152, -10.460, 0.0, -1.5708,   14.152, -10.814, 0.456645, 0.0),
    ("southeast_blue_east",  "plastic_cup_0_clone",  14.856, -11.164, 0.0,  3.1416,   14.502, -11.164, 0.456645, 0.0),
    ("southeast_blue_west",  "plastic_cup_0_clone",  13.448, -11.164, 0.0,  0.0,      13.802, -11.164, 0.456645, 0.0),
    # ---- 西北(绿) cafe_table_1 (-17.2417, 12.498) ----
    ("northwest_green_south", "plastic_cup_0_clone_1", -17.242, 11.794, 0.0,  1.5708,  -17.242, 12.148, 0.456645, 0.0),
    ("northwest_green_north", "plastic_cup_0_clone_1", -17.242, 13.202, 0.0, -1.5708,  -17.242, 12.848, 0.456645, 0.0),
    ("northwest_green_east",  "plastic_cup_0_clone_1", -16.538, 12.498, 0.0,  3.1416,  -16.892, 12.498, 0.456645, 0.0),
    ("northwest_green_west",  "plastic_cup_0_clone_1", -17.946, 12.498, 0.0,  0.0,     -17.592, 12.498, 0.456645, 0.0),
    # ---- 东北(黄) cafe_table_2 (14.7256, 12.265) ----
    ("northeast_yellow_south", "plastic_cup_0_clone_0", 14.726, 11.561, 0.0,  1.5708,  14.726, 11.915, 0.456645, 0.0),
    ("northeast_yellow_north", "plastic_cup_0_clone_0", 14.726, 12.969, 0.0, -1.5708,  14.726, 12.615, 0.456645, 0.0),
    ("northeast_yellow_east",  "plastic_cup_0_clone_0", 15.430, 12.265, 0.0,  3.1416,  15.076, 12.265, 0.456645, 0.0),
    ("northeast_yellow_west",  "plastic_cup_0_clone_0", 14.022, 12.265, 0.0,  0.0,     14.376, 12.265, 0.456645, 0.0),
]


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
        super().__init__("collect_4room_demos")
        self.args = args

        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 运动学：从 /robot_description 话题取 URDF（robot_state_publisher 会 latch 发布）。
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
        """回退：读安装目录里预生成的 URDF。"""
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

    # ---------- 位姿读取 & 停靠验证 ----------
    def _gz_pose(self, model_name):
        """用 `gz model -m <name> -i` 读取世界位姿，返回 (x, y, z, yaw)。"""
        import re
        try:
            out = subprocess.run(
                ["gz", "model", "-m", model_name, "-i"],
                capture_output=True, text=True, timeout=5.0).stdout
        except (subprocess.TimeoutExpired, OSError):
            return None
        m = re.search(r"position \{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)", out)
        if not m:
            return None
        x, y, z = [float(m.group(i)) for i in (1, 2, 3)]
        yaw = 0.0
        q = re.search(r"orientation \{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)"
                      r"\s*z:\s*([-\d.eE+]+)\s*w:\s*([-\d.eE+]+)", out)
        if q:
            qx, qy, qz, qw = [float(q.group(i)) for i in (1, 2, 3, 4)]
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return (x, y, z, yaw)

    def verify_station(self, cup_model):
        """读机器人+杯子的实际位姿，算杯子相对 arm_base 的位置（验证停靠位是否精确）。"""
        robot = self._gz_pose(ROBOT_MODEL)
        cup = self._gz_pose(cup_model)
        if robot is None or cup is None:
            self.get_logger().warn("无法读机器人/杯子位姿，跳过停靠验证")
            return
        rx, ry, rz, ryaw = robot
        cx, cy, cz, _ = cup
        dx, dy = cx - rx, cy - ry
        cos, sin = math.cos(ryaw), math.sin(ryaw)
        # 世界 -> arm_base 的旋转（绕 z 轴转 -ryaw），把杯子偏移投到 arm_base 系
        cup_arm_x = dx * cos + dy * sin
        cup_arm_y = -dx * sin + dy * cos
        cup_arm_z = cz - rz
        self.get_logger().info(f"[验证] {cup_model} 相对 arm_base: "
                               f"x={cup_arm_x:.3f} y={cup_arm_y:.3f} z={cup_arm_z:.3f} "
                               f"（期望 x≈0.354, y≈0）")

    # ---------- 停靠 / teleport / 杯子复位 ----------
    def teleport_robot(self, x, y, z, yaw):
        """用 gz model 命令把机器人瞬间移到停靠位。"""
        cmd = ["gz", "model", "-m", ROBOT_MODEL,
               "-x", f"{x}", "-y", f"{y}", "-z", f"{z}", "-Y", f"{yaw}"]
        self.get_logger().info(f"teleport {ROBOT_MODEL} -> x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
        subprocess.run(cmd, capture_output=True, timeout=10.0)
        # 等 Gazebo 物理稳定 + 关节状态/TF 更新
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

    def reset_cup(self, cup_model, x, y, z, yaw):
        """用 gz model 把杯子摆到指定边（位置 + 朝向）。"""
        cmd = ["gz", "model", "-m", cup_model,
               "-x", f"{x}", "-y", f"{y}", "-z", f"{z}",
               "-R", "0", "-P", "0", "-Y", f"{yaw}"]
        self.get_logger().info(f"摆杯子 {cup_model} -> x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
        subprocess.run(cmd, capture_output=True, timeout=10.0)
        time.sleep(1.0)  # 等杯子落稳

    # ---------- 抓取规划（死动作） ----------
    def plan_grasp(self):
        """返回关节空间 waypoints（固定关节角，死动作）。"""
        def _parse(s):
            return [float(x) for x in s.split(",")]

        waypoints = {
            "grasp": _parse(self.args.joint_grasp),
            "lift": _parse(self.args.joint_lift),
        }
        # 倾倒位姿：在抬升位姿基础上旋转腕部 joint6（原地 roll），把杯子倾倒
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

    # ---------- 单次夹取（死动作） ----------
    def run_episode(self, ep_idx, instruction, station):
        home = [0.0] * 6
        cup_model = station[1]

        # 记录杯子初始 z（用于抓取成功判定）
        cup_z0 = None
        cup_pose = self._gz_pose(cup_model)
        if cup_pose is not None:
            cup_z0 = cup_pose[2]

        waypoints = self.plan_grasp()
        grip = self.args.grip_open  # 夹到杯子宽度，而非夹到 0

        # 组装 waypoint 序列：(关节角, 夹爪开度 0/0.04)
        seq = [
            (home, 0.04),
            (waypoints["grasp"], 0.04),   # 水平前伸，杯子进入夹爪中间
            (waypoints["grasp"], grip),   # 闭合夹住
            (waypoints["lift"], grip),    # 抬起
            (waypoints["tilt"], grip),    # 倾斜（倾倒）
            (waypoints["tilt"], grip),    # 保持倾斜（原地停留）
            (waypoints["lift"], grip),    # 复原竖直
            (waypoints["grasp"], grip),   # 下降放回桌面
            (waypoints["grasp"], 0.04),   # 松开夹爪，杯子落回桌面
            (home, 0.04),                 # 机械臂退回 home
        ]

        steps_per_seg = max(2, int(round(self.args.seg_duration / self.args.control_dt)))

        images = []
        actions = []
        ee_poses = []
        joints = []

        cur_angles = self.get_angles()
        last_grip = None
        grasp_success = False

        for wi in range(len(seq) - 1):
            a_angles, a_grip = seq[wi]
            b_angles, b_grip = seq[wi + 1]
            interp = self._interp(a_angles, b_angles, steps_per_seg)

            for step in range(len(interp)):
                next_angles = list(interp[step])
                grip_open = b_grip

                image = self.get_camera_image()
                p_cur, R_cur = self.kin.forward(cur_angles)
                p_next, R_next = self.kin.forward(next_angles)
                grip_cmd = 1.0 if grip_open > 0.035 else 0.0
                action = pose_delta_action(p_cur, R_cur, p_next, R_next, grip_cmd)

                if self.args.record and image is not None:
                    images.append(image)
                    actions.append(action)
                    ee_poses.append(np.concatenate([p_cur, [0, 0, 0, 1.0]]))
                    joints.append(cur_angles)

                self.publish_arm(next_angles, self.args.control_dt)
                if last_grip is None or abs(grip_open - last_grip) > 1e-4:
                    self.publish_gripper(grip_open, 0.3)
                    last_grip = grip_open
                time.sleep(self.args.control_dt)
                rclpy.spin_once(self, timeout_sec=0.01)

                cur_angles = next_angles

            # 抓取检测：抬起段（wi==2，grasp->lift）执行完后，杯子应被抬离桌面
            if wi == 2 and cup_z0 is not None:
                cup_pose = self._gz_pose(cup_model)
                if cup_pose is not None and cup_pose[2] > cup_z0 + 0.05:
                    grasp_success = True
                    self.get_logger().info(f"✅ 杯子抬升 {cup_z0:.3f} -> {cup_pose[2]:.3f}")
                else:
                    cur_z = cup_pose[2] if cup_pose is not None else -1.0
                    self.get_logger().warn(f"❌ 杯子未抬升（z0={cup_z0:.3f}，当前={cur_z:.3f}）")

        if not self.args.record:
            return None

        T = len(actions)
        is_first = [False] * T
        is_last = [False] * T
        if T > 0:
            is_first[0] = True
            is_last[-1] = True

        return {
            "images": np.stack(images) if images else None,
            "actions": np.stack(actions) if actions else None,
            "ee_poses": np.stack(ee_poses) if ee_poses else None,
            "joints": np.stack(joints) if joints else None,
            "is_first": is_first,
            "is_last": is_last,
            "instruction": instruction,
            "station": station[0],
            "grasp_success": grasp_success,
        }

    def save_episode(self, ep_idx, data):
        out_dir = os.path.join(self.args.out, f"ep_{ep_idx:04d}")
        os.makedirs(out_dir, exist_ok=True)
        if data["images"] is not None:
            imgs = data["images"]
            small = np.array([cv2.resize(im, (320, 240)) for im in imgs], dtype=np.uint8)
            np.save(os.path.join(out_dir, "images.npy"), small)
        np.save(os.path.join(out_dir, "actions.npy"), data["actions"].astype(np.float32))
        meta = {
            "instruction": data["instruction"],
            "station": data["station"],
            "ee_poses": data["ee_poses"].tolist() if data["ee_poses"] is not None else None,
            "joint_states": data["joints"].tolist() if data["joints"] is not None else None,
            "is_first": data["is_first"],
            "is_last": data["is_last"],
            "grasp_success": data["grasp_success"],
        }
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f"已保存 episode {ep_idx}: {out_dir} "
                               f"({data['actions'].shape[0]} 步, 房间 {data['station']})")

    def run(self):
        stations = [GRASP_STATIONS[i] for i in self.args.stations]

        if self.args.dry_run:
            self.get_logger().info("=== dry-run：仅打印停靠位与规划 ===")
            for st in stations:
                self.get_logger().info(f"停靠位 {st[0]}: 杯子={st[1]} "
                                       f"机器人 x={st[2]} y={st[3]} yaw={st[5]:.3f} "
                                       f"杯子 x={st[6]} y={st[7]} yaw={st[9]:.3f}")
            self.plan_grasp()
            return

        os.makedirs(self.args.out, exist_ok=True)
        ep_idx = 24
        for cycle in range(self.args.cycles):
            self.get_logger().info(f"===== 循环 {cycle} =====")
            for st in stations:
                room, cup_model, x, y, z, yaw, cup_x, cup_y, cup_z, cup_yaw = st
                self.get_logger().info(f"----- 停靠位 {room}（杯子 {cup_model}）-----")

                # 0. teleport 机器人到停靠位
                self.teleport_robot(x, y, z, yaw)

                # 1. 机械臂先回 home + 张开夹爪（先收好机械臂，避免收臂时扫到杯子）
                self.publish_arm([0.0] * 6, 3.0)
                self.publish_gripper(0.04, 1.0)
                time.sleep(4.0)
                self._wait_until(lambda: all(abs(self.joint_pos.get(n, 1.0)) < 0.02
                                             for n in self.arm_joint_names),
                                 "回 home 到位", timeout=6.0)

                # 2. 机械臂收好后，再摆杯子（此时收臂不会扫到杯子）
                self.reset_cup(cup_model, cup_x, cup_y, cup_z, cup_yaw)

                # 3. 验证杯子相对 arm_base 的位置（应为前方 ~0.354m）
                self.verify_station(cup_model)

                # 4. 跑死动作 + 记录 + 抓取检测
                data = self.run_episode(ep_idx, self.args.instruction, st)
                if data is None:
                    continue  # --no-record 模式，仅演示不落盘
                if data.get("grasp_success", False):
                    self.save_episode(ep_idx, data)
                    ep_idx += 1
                else:
                    self.get_logger().warn(f"停靠位 {room} 夹取失败，跳过保存")


def main():
    parser = argparse.ArgumentParser(description="多房间循环抓取 + LoRA 数据采集")
    parser.add_argument("--cycles", type=int, default=5,
                        help="循环次数（每循环 16 条，默认 5 循环 = 80 条）")
    parser.add_argument("--stations", type=lambda s: [int(x) for x in s.split(",")],
                        default=list(range(16)),
                        help="采集哪些停靠位（下标 0-15，逗号分隔，默认全部 16 个）")
    parser.add_argument("--out", default="/home/ljh/robot2/NOW/VLA_ROBOT-main/vla_demos")
    parser.add_argument("--instruction", default="pick up the cup")
    parser.add_argument("--camera-topic", default="/camera_sensor/image_raw")
    parser.add_argument("--seg-duration", type=float, default=2.0,
                        help="每段轨迹时长（秒）")
    parser.add_argument("--control-dt", type=float, default=0.5,
                        help="控制/采样周期（秒），微调建议 0.1~0.2（5-10Hz）")
    parser.add_argument("--joint-grasp", default="0,1.66,-0.52,0,-1.05,0",
                        help="抓取关节角（6 个，逗号分隔）")
    parser.add_argument("--joint-lift", default="0,1.42,-0.68,0,-0.65,0",
                        help="抬起关节角（6 个，逗号分隔）")
    parser.add_argument("--tilt-angle", type=float, default=1.57,
                        help="倾倒时腕部 joint6 的旋转角（弧度）")
    parser.add_argument("--grip-open", type=float, default=0.03,
                        help="闭合夹爪时的开度（0.04=8cm 全开，0.03=6cm）")
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
