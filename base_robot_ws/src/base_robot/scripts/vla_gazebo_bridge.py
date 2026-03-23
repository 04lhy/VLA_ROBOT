import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import requests
import json_numpy
json_numpy.patch()

class VLAGazeboBridge(Node):
    def __init__(self):
        super().__init__('vla_gazebo_bridge')
        self.bridge = CvBridge()

        # 订阅相机
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # 发布控制
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

        self.server_url = "http://127.0.0.1:8000/act"
        self.instruction = "pick up the object"
        self.processing = False

    def image_callback(self, msg):
        if self.processing:
            return

        self.processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            payload = {
                "image": cv_image,
                "instruction": self.instruction
            }

            response = requests.post(self.server_url, json=payload)
            if response.status_code == 200:
                result = response.json()
                action_array = result["action"]
                self.get_logger().info(f"Received action: {action_array}")
                self.publish_arm_and_gripper(action_array)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.processing = False

    def publish_arm_and_gripper(self, action_array):
        # ===================== 【精准匹配你的机器人】 =====================
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        gripper_joints = ['joint7', 'joint8']  # 你的夹爪是 2 个关节！

        # 机械臂
        arm_traj = JointTrajectory()
        arm_traj.joint_names = arm_joints
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(x) for x in action_array[:6]]
        arm_point.time_from_start.nanosec = 500000000
        arm_traj.points.append(arm_point)
        self.arm_pub.publish(arm_traj)

        # 夹爪：两个关节用同一个值（标准平行夹爪）
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = gripper_joints
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [float(action_array[6]), float(action_array[6])]
        gripper_point.time_from_start.nanosec = 500000000
        gripper_traj.points.append(gripper_point)
        self.gripper_pub.publish(gripper_traj)

        self.get_logger().info("✅ 指令发送成功！Arm + Gripper 正常工作！")

def main(args=None):
    rclpy.init(args=args)
    node = VLAGazeboBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()