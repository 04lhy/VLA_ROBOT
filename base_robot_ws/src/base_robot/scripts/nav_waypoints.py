#!/usr/bin/env python3
"""
多点自主导航脚本：让小车按顺序依次前往地图上的多个目标点。

运行前确保已启动：
  1. base_robot.launch.py   （核心仿真：Gazebo + 底盘差速驱动）
  2. navigation.launch.py   （Nav2 导航栈 + AMCL 定位）

目标点坐标在下方 WAYPOINTS 列表中修改（map 坐标系）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


# ==================== 在这里设置目标点 ====================
# 每个目标点: (x, y, yaw)   x/y 单位米，yaw 单位弧度（0 = 朝 +x 方向）
# 工厂地图（demo_scene）世界坐标范围: x ∈ [-22, 22], y ∈ [-17, 17]
# 机器人出生点 / 回家点 = 中央走廊 (0, 0, 0)
# 巡检路线：4 个房间的「指示牌前」坐标（世界坐标系）
#   room1 西南(红)  room2 东南(蓝)  room3 东北(黄)  room4 西北(绿)
# 每间房的「桌前」坐标（识别到红色目标时去抓杯，供决策模型 agent_demo 使用）：
#   room1 桌前 (-18.062, -10.526, 0.0)     room2 桌前 (15.082, -11.489, 3.1416)
#   room3 桌前 (15.687, 12.585, 3.1416)    room4 桌前 (-18.238, 12.883, 0.0)
WAYPOINTS = [
    (-20.447, -14.889, 3.1416),
    (21.333, -14.889, 0.0),
    (21.333, 14.436, 0.0),
    (-20.447, 14.122, 3.1416),
]

# 到达每个目标点后的停留时间（秒）
WAIT_AT_WAYPOINT = 2.0
# 单个目标点导航超时（秒）
NAV_TIMEOUT = 300.0
# 某个目标点导航失败时是否停止（False = 跳过失败点继续下一个）
STOP_ON_FAILURE = False
# ========================================================


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('nav_waypoints')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_for_server(self, timeout_sec=10.0):
        if not self._nav_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Nav2 'navigate_to_pose' action server 不可用，"
                                    "请确认 navigation.launch.py 已启动")
            return False
        return True

    def navigate_to_pose(self, x, y, yaw=0.0, timeout_sec=NAV_TIMEOUT):
        """导航到单个目标点，返回是否成功"""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

        self.get_logger().info(f"发送导航目标: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"目标 ({x:.2f}, {y:.2f}) 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result()
        if result is None:
            self.get_logger().warn(f"目标 ({x:.2f}, {y:.2f}) 导航超时")
            return False

        self.get_logger().info(f"✅ 到达目标 ({x:.2f}, {y:.2f})")
        return True

    def run(self):
        if not self.wait_for_server():
            return

        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            self.get_logger().info(f"--- 前往目标点 {i + 1}/{len(WAYPOINTS)} ---")
            ok = self.navigate_to_pose(x, y, yaw)

            if not ok and STOP_ON_FAILURE:
                self.get_logger().error("导航失败，按配置停止任务")
                return

            if WAIT_AT_WAYPOINT > 0:
                self.get_logger().info(f"到达后停留 {WAIT_AT_WAYPOINT}s ...")
                # 用 spin 方式等待，保持节点响应
                self._sleep(WAIT_AT_WAYPOINT)

        self.get_logger().info("全部目标点导航完成！")

    def _sleep(self, seconds):
        """带 spin 的睡眠，避免阻塞期间错过回调"""
        end = self.get_clock().now() + rclpy.duration.Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
