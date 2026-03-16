#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class FakeDiffDrive(Node):
    def __init__(self):
        super().__init__('fake_diff_drive')
        
        # 可调节参数（与URDF/gazebo插件一致）
        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        
        # 状态变量
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # 订阅 cmd_vel
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 发布 joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # 发布 odom 话题 + TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Fake Diff Drive node started')
        
    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        v = msg.linear.x
        w = msg.angular.z
        
        # 差速逆运动学
        left_vel = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
        right_vel = (v + w * self.wheel_sep / 2.0) / self.wheel_rad
        
        self.left_wheel_angle += left_vel * dt
        self.right_wheel_angle += right_vel * dt
        
        # 里程计积分
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += w * dt
        
    def publish_joint_states(self):
        # 关节状态
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)
        
        # 发布里程计消息
        self.publish_odometry()
        # 发布 TF：odom -> base_footprint
        self.publish_tf()
        
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'  # 必须与 URDF 中 root link 一致
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
