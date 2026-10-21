#!/usr/bin/env python3
"""
自动任务脚本：机械臂运动到目标姿态，夹爪独立控制，自主导航。
运行前确保 base_robot.launch.py 已启动。
导航任务需要同时运行 navigation.launch.py。
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
        self.get_logger().info("等待控制器就绪...")
        time.sleep(5.0)

        # ========================
        # 第一步：夹爪先完全张开（多发几次确保收到）
        # ========================
        self.get_logger().info("--- 夹爪全开 ---")
        for _ in range(5):
            self.move_gripper(0.04, 0.04, duration=0.5)
            time.sleep(0.3)
        time.sleep(1.0)
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
            self.get_logger().info(f"✅ 检测到 {object_name} 附着成功！抬升机械臂")
        else:
            self.get_logger().warn(f"超时 90s 未检测到 {object_name} 附着，仍然抬升")
        self.move_arm([-0.36, 0.81, -1.01, 0.0, 0.0, 0.0], duration=1.5)
        time.sleep(2.0)

        # 导航到原点
        self.get_logger().info("--- 自主导航到 (0, 0, 0) ---")
        self.navigate_to_pose(0.0, 0.0, 0.0)

        self.get_logger().info("任务完成！")

    def navigate_to_pose(self, x, y, yaw=0.0, timeout_sec=120.0):
        """通过 Nav2 自主导航到 (x, y, yaw)"""
        from rclpy.action import ActionClient
        from geometry_msgs.msg import PoseStamped, Quaternion
        from nav2_msgs.action import NavigateToPose
        import math

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server 不可用")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

        self.get_logger().info(f"导航目标: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝")
            return False

        self.get_logger().info("导航中...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result()
        if result is None:
            self.get_logger().warn("导航超时")
            return False
        self.get_logger().info("导航完成")
        return True


def main():
    rclpy.init()
    node = AutoTask()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
