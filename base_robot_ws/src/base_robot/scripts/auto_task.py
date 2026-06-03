#!/usr/bin/env python3
"""
自动任务脚本：机械臂运动到目标姿态，夹爪独立控制。
运行前确保 base_robot.launch.py 已启动。
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import subprocess
import time


class AutoTask(Node):
    def __init__(self):
        super().__init__('auto_task')

        # 机械臂 & 夹爪发布者
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.get_logger().info("自动任务节点已启动")

    def move_arm(self, positions, duration=2.0):
        """控制机械臂到指定关节角度"""
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        self.arm_pub.publish(msg)
        self.get_logger().info(f"机械臂运动: {[round(p, 2) for p in positions]}")

    def move_gripper(self, finger1, finger2, duration=1.0):
        """独立控制两个手指，finger1→joint7, finger2→joint8(取反)"""
        msg = JointTrajectory()
        msg.joint_names = ['joint7', 'joint8']
        point = JointTrajectoryPoint()
        point.positions = [float(finger1), float(-finger2)]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        self.grip_pub.publish(msg)
        self.get_logger().info(f"夹爪: 手指1={finger1:.3f}, 手指2={finger2:.3f}")

    def run(self):
        object_name = "coke_can"
        topic = "/gazebo/default/grasp_events"

        # ========================
        # 启动附着监听（从一开始就监听，避免漏消息）
        # 用 shell 管道：gz topic -e | grep -m1 持久订阅，匹配到第一条就退出
        # ========================
        self.get_logger().info(f"启动附着监听: {topic}")
        cmd = (
            f"gz topic -e {topic} 2>/dev/null | "
            f"grep --line-buffered -m1 'attached: true' | "
            f"grep -q '{object_name}'"
        )
        listener_proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.get_logger().info("监听进程已启动 (持久订阅，不丢消息)")

        # ========================
        # 等发布者发现订阅者，避免第一条指令丢失
        # ========================
        self.get_logger().info("等待发布者连接...")
        time.sleep(1.0)

        # ========================
        # 第一步：夹爪先完全张开（多发几次确保收到）
        # ========================
        self.get_logger().info("--- 夹爪全开 ---")
        for _ in range(3):
            self.move_gripper(0.04, 0.04, duration=0.5)
            time.sleep(0.3)
        time.sleep(1.0)

        # ========================
        # 第二步：机械臂运动到目标位姿1
        # ========================
        self.get_logger().info("--- 机械臂运动到目标位姿1 ---")
        self.move_arm([-0.35, 0.81, -1.01, 0.0, 0.0, 0.0], duration=2.0)
        time.sleep(2.5)

        # ========================
        # 第三步：机械臂运动到目标位姿2
        # ========================
        self.get_logger().info("--- 机械臂运动到目标位姿2 ---")
        self.move_arm([-0.36, 1.07, -1.01, 0.0, 0.0, 0.0], duration=2.0)
        time.sleep(2.5)

        # ========================
        # 第四步：夹爪闭合
        # ========================
        self.get_logger().info("--- 夹爪闭合 ---")
        self.move_gripper(0.030, 0.032, duration=1.0)
        time.sleep(1.5)

        # ========================
        # 第五步：等待附着（最多 90s）
        # ========================
        self.get_logger().info("--- 等待附着 (最多 90s) ---")
        try:
            listener_proc.wait(timeout=90)
            attached = True
        except subprocess.TimeoutExpired:
            listener_proc.kill()
            attached = False

        if attached:
            self.get_logger().info(f"✅ 检测到 {object_name} 附着成功！关节2 → 0.81")
            self.move_arm([-0.36, 0.81, -1.01, 0.0, 0.0, 0.0], duration=1.5)
            time.sleep(2.0)
        else:
            self.get_logger().warn(f"超时 90s 未检测到 {object_name} 附着")

        self.get_logger().info("任务完成！")


def main():
    rclpy.init()
    node = AutoTask()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
