#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class FakeDiffDrive(Node):
    def __init__(self):
        super().__init__('fake_diff_drive')

        # ===== 差速轮参数 =====
        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        # ===== 机械臂关节参数（可选） =====
        # 仅用描述符指定类型，不提供默认值，避免类型冲突
        str_array_desc = ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value)
        self.declare_parameter('arm_joint_names', descriptor=str_array_desc)

        double_array_desc = ParameterDescriptor(type=Parameter.Type.DOUBLE_ARRAY.value)
        self.declare_parameter('arm_joint_positions', descriptor=double_array_desc)

        # 获取参数值，若未设置则默认为空列表
        self.arm_joint_names = self.get_parameter('arm_joint_names').value
        if self.arm_joint_names is None:
            self.arm_joint_names = []

        self.arm_joint_positions = self.get_parameter('arm_joint_positions').value
        if self.arm_joint_positions is None:
            self.arm_joint_positions = []

        # 检查长度是否匹配
        if len(self.arm_joint_names) != len(self.arm_joint_positions):
            self.get_logger().warn(
                'arm_joint_names and arm_joint_positions have different lengths! '
                'Mechanical arm joints will be ignored.'
            )
            self.arm_joint_names = []
            self.arm_joint_positions = []

        # 获取差速轮参数
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        # ===== 内部状态变量 =====
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ===== 创建订阅者、发布者、定时器 =====
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Fake Diff Drive node started')
        if self.arm_joint_names:
            self.get_logger().info(
                f'Will publish mechanical arm joints: {self.arm_joint_names} '
                f'with initial positions: {self.arm_joint_positions}'
            )

    def cmd_vel_callback(self, msg: Twist):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = msg.linear.x
        w = msg.angular.z

        left_vel = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
        right_vel = (v + w * self.wheel_sep / 2.0) / self.wheel_rad

        self.left_wheel_angle += left_vel * dt
        self.right_wheel_angle += right_vel * dt

        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += w * dt

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        joint_names = [self.left_joint, self.right_joint]
        joint_positions = [self.left_wheel_angle, self.right_wheel_angle]

        if self.arm_joint_names:
            joint_names.extend(self.arm_joint_names)
            joint_positions.extend(self.arm_joint_positions)

        js.name = joint_names
        js.position = joint_positions
        self.joint_pub.publish(js)

        self.publish_odometry()
        self.publish_tf()

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'   # 根据您的URDF调整
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDiffDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
