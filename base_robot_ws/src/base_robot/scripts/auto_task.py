#!/usr/bin/env python3
"""
自动任务脚本：先导航到目标点，到达后机械臂运动到指定姿态。
运行前确保 base_robot.launch.py 和 navigation.launch.py 已启动。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import math
import time


def quaternion_to_euler(q):
    """四元数转 yaw"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = float(math.sin(yaw / 2.0))
    q.w = float(math.cos(yaw / 2.0))
    return q


class AutoTask(Node):
    def __init__(self):
        super().__init__('auto_task')

        # 导航 action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 机械臂 & 夹爪发布者
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # TF 缓存，用于获取当前位姿
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("自动任务节点已启动，等待导航服务...")

    def get_current_pose(self, timeout=1.0):
        """从 TF 获取当前机器人在 map 坐标系下的位姿，返回 (x, y, yaw) 或 None"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = quaternion_to_euler(q)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"无法获取当前位姿: {e}")
            return None

    def send_nav_goal(self, x, y, yaw):
        """发送导航目标并等待到达"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = euler_to_quaternion(yaw)

        self.get_logger().info(f"发送导航目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("导航服务 /navigate_to_pose 未就绪！")
            return False

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝！")
            return False

        self.get_logger().info("导航目标已接受，等待到达...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("已到达目标点！")
            return True
        else:
            self.get_logger().error(f"导航失败，状态码: {result.status}")
            return False

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

    def move_gripper(self, opening, duration=0.5):
        """控制夹爪开合，opening: 0.0(闭合) ~ 0.04(全开)"""
        msg = JointTrajectory()
        msg.joint_names = ['joint7', 'joint8']
        point = JointTrajectoryPoint()
        point.positions = [opening, -opening]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        self.grip_pub.publish(msg)
        self.get_logger().info(f"夹爪: {opening:.3f}")

    def run(self):
        # ========================
        # 第零步：获取当前位姿
        # ========================
        self.get_logger().info("正在获取当前位姿...")
        pose = self.get_current_pose()
        if pose:
            self.get_logger().info(f"当前位置: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={pose[2]:.3f}")
        else:
            self.get_logger().warn("无法获取当前位置，继续执行...")

        # ========================
        # 第一步：导航到目标点
        # ========================
        goal_x = 1.993   # 目标 x 坐标 (米)
        goal_y = 1.829   # 目标 y 坐标 (米)
        goal_yaw = 1.472  # 目标朝向 (弧度)

        success = self.send_nav_goal(goal_x, goal_y, goal_yaw)
        if not success:
            self.get_logger().error("导航失败，跳过机械臂任务。")
            return

        # 到达后稍等一下稳定
        time.sleep(1.0)

        # ========================
        # 第二步：机械臂运动
        # ========================

        # 关节顺序: [joint1, joint2, joint3, joint4, joint5, joint6]
        self.get_logger().info("--- 机械臂调整到目标位姿 ---")
        self.move_arm([0.23, 0.0, -0.84, 0.0, 0.84, -0.26], duration=2.0)
        time.sleep(2.5)

        # 夹爪全开
        self.move_gripper(0.04, duration=0.5)
        time.sleep(1.0)

        self.get_logger().info("任务完成！")


def main():
    rclpy.init()
    node = AutoTask()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
