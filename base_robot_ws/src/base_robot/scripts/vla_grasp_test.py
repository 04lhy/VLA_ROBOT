#!/usr/bin/env python3
"""VLA 夹取测试脚本。

通过 /vla_instruction 触发 VLA 桥接节点驱动机械臂夹取物体，并轮询目标物体的
世界坐标 z 值确认是否被抬起。

注意：本机 gz topic -e / gz topic -i 无法读取任何话题（连 pose/info 都无输出、
Type 为空），因此不能靠 grasp_events 话题做判定，改用 gz model 查询模型位姿。

前置条件：
1. OpenVLA 服务已启动（deploy_4bit.py，监听 http://127.0.0.1:8000/act）
2. base_robot.launch.py 已启动（自动拉起 vla_gazebo_bridge + gazebo_grasp_plugin）
3. 世界中有目标物体（默认 coke_can）

用法：
    ros2 run base_robot vla_grasp_test.py
    ros2 run base_robot vla_grasp_test.py --instruction "pick up the coke can" \
        --object coke_can --timeout 120
"""

import argparse
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VLAGraspTest(Node):
    def __init__(self, instruction: str, object_name: str, timeout: float,
                 lift_threshold: float):
        super().__init__('vla_grasp_test')
        self.instruction = instruction
        self.object_name = object_name
        self.timeout = timeout
        self.lift_threshold = lift_threshold
        self.instr_pub = self.create_publisher(String, '/vla_instruction', 10)

    def publish_instruction(self, text: str, repeat: int = 5) -> None:
        """向 /vla_instruction 发指令，多发几次避免首条丢失。"""
        msg = String()
        msg.data = text
        for _ in range(repeat):
            self.instr_pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info(f"已发布 VLA 指令: '{text}'")

    def read_object_z(self):
        """用 gz model 读取目标物体世界坐标 z，失败返回 None。"""
        try:
            out = subprocess.run(
                ["gz", "model", "-m", self.object_name, "-i"],
                capture_output=True, text=True, timeout=5.0,
            ).stdout
        except (subprocess.TimeoutExpired, OSError) as e:
            self.get_logger().warn(f"gz model 调用失败: {e}")
            return None

        # 只取模型自身的 pose 块，避免误读 link 内部的 pose
        block = re.search(r"^pose \{\s*position \{(.*?)\}", out, re.S | re.M)
        if not block:
            return None
        z = re.search(r"z:\s*([-\d.eE+]+)", block.group(1))
        return float(z.group(1)) if z else None

    def run(self) -> bool:
        # 1. 记录初始高度作为基线
        baseline = self.read_object_z()
        if baseline is None:
            self.get_logger().error(
                f"读取不到 {self.object_name} 的位姿，请确认 Gazebo 在运行且模型名正确")
            return False
        self.get_logger().info(
            f"{self.object_name} 初始高度 z={baseline:.3f} m，"
            f"抬起阈值 {self.lift_threshold:.3f} m")

        # 2. 发布指令触发 VLA
        self.publish_instruction(self.instruction)

        # 3. 轮询高度判断是否被抬起
        self.get_logger().info(
            f"等待 {self.object_name} 被抬起（最多 {self.timeout:.0f}s）...")
        deadline = time.time() + self.timeout
        lifted = False
        while time.time() < deadline:
            time.sleep(1.0)
            z = self.read_object_z()
            if z is not None and z - baseline >= self.lift_threshold:
                lifted = True
                self.get_logger().info(
                    f"✅ 夹取成功！{self.object_name} z: {baseline:.3f} -> {z:.3f} m")
                break
            if z is not None:
                self.get_logger().info(
                    f"当前 z={z:.3f} m（抬升 {z - baseline:.3f} m）")

        # 4. 停止 VLA，避免机械臂继续运动
        self.publish_instruction("stay still", repeat=2)

        if not lifted:
            self.get_logger().warn(
                f"❌ 超时 {self.timeout:.0f}s 未检测到 {self.object_name} 被抬起")
        return lifted


def main():
    parser = argparse.ArgumentParser(description="VLA 夹取测试")
    parser.add_argument("--instruction", default="pick up the coke can",
                        help="发送给 VLA 的自然语言指令")
    parser.add_argument("--object", default="coke_can",
                        help="目标物体名（用于查询 gz model 位姿）")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="等待夹取的超时时间（秒）")
    parser.add_argument("--lift-threshold", type=float, default=0.05,
                        help="判定抬起的 z 抬升阈值（米）")
    args = parser.parse_args()

    rclpy.init()
    node = VLAGraspTest(args.instruction, args.object, args.timeout,
                        args.lift_threshold)
    success = node.run()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if success else 1)


if __name__ == "__main__":
    main()
