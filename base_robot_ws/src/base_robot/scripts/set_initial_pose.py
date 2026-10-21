#!/usr/bin/env python3
"""
发布 AMCL 初始位姿。

Nav2 Humble 的 AMCL 已不再读取 yaml 参数里的 set_initial_pose / initial_pose，
必须通过 /initialpose 话题手动发布初始位姿（等价于 RViz 的 2D Pose Estimate）。

本节点在延迟 delay 秒后（等待 AMCL 启动）重复发布若干次初始位姿，确保定位正确。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', 2.0279)
        self.declare_parameter('y', 1.8948)
        self.declare_parameter('yaw', 1.7867)
        self.declare_parameter('delay', 15.0)
        self.declare_parameter('repeat', 3)

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        self._delay = self.get_parameter('delay').value
        self._repeat = int(self.get_parameter('repeat').value)

        self._publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = 'map'
        self._msg.pose.pose.position.x = float(x)
        self._msg.pose.pose.position.y = float(y)
        self._msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 初始协方差：位置 ~0.5m，朝向 ~0.26rad
        self._msg.pose.covariance = [0.0] * 36
        self._msg.pose.covariance[0] = 0.25
        self._msg.pose.covariance[7] = 0.25
        self._msg.pose.covariance[35] = 0.0685

    def run(self):
        self._sleep(self._delay)
        for _ in range(self._repeat):
            # 注意：header.stamp 保持为 0，让 tf2 使用最新可用的 TF，
            # 避免因时间戳与 TF 缓存未对齐导致 "Failed to transform initial pose in time"。
            self._publisher.publish(self._msg)
            self.get_logger().info(
                f"已发布初始位姿: ({self._msg.pose.pose.position.x:.2f}, "
                f"{self._msg.pose.pose.position.y:.2f})"
            )
            self._sleep(1.0)
        self.get_logger().info("初始位姿发布完成")

    def _sleep(self, seconds):
        """带 spin 的睡眠，保持仿真时钟持续更新"""
        end = self.get_clock().now() + rclpy.duration.Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
