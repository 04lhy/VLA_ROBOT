#!/usr/bin/env python3
"""VLA 抓取前机械臂预置位姿。

在 arm_controller 激活后，将机械臂移动到腕部相机正对杯子的位姿，
确保 VLA 第一帧画面就能看到杯子，避免冷启动死锁（相机对着地板/墙）。

目标：机械臂保持默认零位（home），此位姿下腕部相机正好能看到桌上的杯子（用户已在 RViz 中确认）。
关节角（弧度，全零即机械臂归零位）：
  joint1..joint6 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"""
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VLAPresetPose(Node):
    def __init__(self):
        super().__init__('vla_preset_pose')
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # 等待 arm_controller 就绪
        self.get_logger().info("等待 arm_controller 就绪...")
        self._wait_for_controller()
        self.get_logger().info("arm_controller 就绪，发送预置位姿")

    def _wait_for_controller(self):
        """简单等待控制器 topic 存在（通过订阅者数量判断）。"""
        while rclpy.ok():
            if self.pub.get_subscription_count() > 0:
                return
            time.sleep(0.5)

    def send_pose(self, angles, duration=3.0):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.velocities = [0.0] * 6
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(point)

        # 多发几次，避免首条丢失
        for _ in range(3):
            self.pub.publish(traj)
            time.sleep(0.3)
        self.get_logger().info(f"预置位姿已发送: {angles}")


def main():
    rclpy.init()
    node = VLAPresetPose()
    angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.send_pose(angles)
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
