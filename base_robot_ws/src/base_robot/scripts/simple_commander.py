#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
import tkinter as tk
from tkinter import ttk
import time

class SimpleArmGUI(Node):
    def __init__(self):
        super().__init__('simple_arm_gui')
        
        # --- 1. 初始化 ROS 2 发布者 ---
        # 机械臂控制发布者
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # 底盘控制发布者 - Twist消息控制底盘
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)
        
        self.get_logger().info("✅ 机械臂+底盘控制端已启动！")

        # --- 2. 初始化 Tkinter 界面 ---
        self.root = tk.Tk()
        self.root.title("🦾 机械臂+底盘控制台")
        self.root.geometry("400x700")  # 增加高度以容纳底盘控制
        
        # === 底盘控制部分 ===
        tk.Label(self.root, text="底盘控制", font=('Arial', 14, 'bold')).pack(pady=10)
        
        # 创建底盘控制框架
        chassis_frame = tk.Frame(self.root, relief=tk.RAISED, borderwidth=2)
        chassis_frame.pack(fill='x', padx=20, pady=10)
        
        # 线速度控制
        vel_frame = tk.Frame(chassis_frame)
        vel_frame.pack(fill='x', padx=10, pady=5)
        tk.Label(vel_frame, text="前进/后退速度(m/s)", width=20, anchor='w').pack(side='left')
        self.linear_vel_slider = tk.Scale(
            vel_frame, 
            from_=-0.5, to=0.5, resolution=0.01, 
            orient=tk.HORIZONTAL, length=250,
            command=lambda _: self.send_chassis_cmd()
        )
        self.linear_vel_slider.set(0.0)
        self.linear_vel_slider.pack(side='left', expand=True)
        
        # 角速度控制
        ang_frame = tk.Frame(chassis_frame)
        ang_frame.pack(fill='x', padx=10, pady=5)
        tk.Label(ang_frame, text="旋转速度(rad/s)", width=20, anchor='w').pack(side='left')
        self.angular_vel_slider = tk.Scale(
            ang_frame, 
            from_=-1.0, to=1.0, resolution=0.05, 
            orient=tk.HORIZONTAL, length=250,
            command=lambda _: self.send_chassis_cmd()
        )
        self.angular_vel_slider.set(0.0)
        self.angular_vel_slider.pack(side='left', expand=True)
        
        # 停止按钮
        stop_btn = tk.Button(
            chassis_frame, 
            text="停止底盘", 
            command=self.stop_chassis,
            bg='#dc3545', fg='white', font=('Arial', 10)
        )
        stop_btn.pack(pady=5)
        
        tk.Frame(self.root, height=2, bg="grey").pack(fill='x', padx=20, pady=10)
        
        # === 机械臂控制部分 ===
        tk.Label(self.root, text="机械臂控制", font=('Arial', 14, 'bold')).pack(pady=5)
        
        self.arm_sliders = []
        
        # 创建 1-6 轴机械臂滑块 (-3.14 到 3.14 弧度)，绑定 command 回调
        for i in range(1, 7):
            frame = tk.Frame(self.root)
            frame.pack(fill='x', padx=20, pady=3)
            
            lbl = tk.Label(frame, text=f"关节 {i}", width=8, anchor='w')
            lbl.pack(side='left')
            
            # 滑块值变化时，调用 send_pose
            scale = tk.Scale(
                frame, 
                from_=-3.14, to=3.14, resolution=0.01, 
                orient=tk.HORIZONTAL, length=250,
                command=lambda _: self.send_pose()  # 绑定回调
            )
            scale.set(0.0)
            scale.pack(side='left', expand=True)
            self.arm_sliders.append(scale)
            
        # 创建夹爪手指滑块 (每个手指独立控制)
        tk.Frame(self.root, height=2, bg="grey").pack(fill='x', padx=20, pady=5)

        # --- 手指1: joint7 (范围 0.0 ~ 0.04) ---
        grip7_frame = tk.Frame(self.root)
        grip7_frame.pack(fill='x', padx=20, pady=3)
        tk.Label(grip7_frame, text="手指1", width=8, anchor='w').pack(side='left')
        self.grip7_slider = tk.Scale(
            grip7_frame,
            from_=0.0, to=0.04, resolution=0.001,
            orient=tk.HORIZONTAL, length=250,
            command=lambda _: self.send_pose()
        )
        self.grip7_slider.set(0.04)
        self.grip7_slider.pack(side='left', expand=True)

        # --- 手指2: joint8 (范围 0.0 ~ 0.04，内部映射为负值) ---
        grip8_frame = tk.Frame(self.root)
        grip8_frame.pack(fill='x', padx=20, pady=3)
        tk.Label(grip8_frame, text="手指2", width=8, anchor='w').pack(side='left')
        self.grip8_slider = tk.Scale(
            grip8_frame,
            from_=0.0, to=0.04, resolution=0.001,
            orient=tk.HORIZONTAL, length=250,
            command=lambda _: self.send_pose()
        )
        self.grip8_slider.set(0.04)
        self.grip8_slider.pack(side='left', expand=True)

        # 夹爪同步按钮
        sync_frame = tk.Frame(self.root)
        sync_frame.pack(fill='x', padx=20, pady=3)
        tk.Label(sync_frame, text="同步", width=8, anchor='w',
                 font=('Arial', 9)).pack(side='left')
        # 用 master 滑块同步两个手指
        self.grip_master_slider = tk.Scale(
            sync_frame,
            from_=0.0, to=0.04, resolution=0.001,
            orient=tk.HORIZONTAL, length=250,
            command=self.sync_grippers
        )
        self.grip_master_slider.set(0.04)
        self.grip_master_slider.pack(side='left', expand=True)

        # 发送按钮
        btn = tk.Button(
            self.root, 
            text="发送机械臂位姿", 
            command=self.send_pose, 
            bg='#0078d7', fg='white', 
            font=('Arial', 12, 'bold')
        )
        btn.pack(pady=20)
        
        # 窗口关闭事件绑定
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # 防抖计时器
        self.last_chassis_send_time = 0
        self.chassis_send_delay = 0.1  # 100ms
        
        # --- 3. 启动 ROS 2 刷新定时器 ---
        self.ros_spin_timer()

    def ros_spin_timer(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self.ros_spin_timer)

    def send_chassis_cmd(self):
        """发送底盘控制命令"""
        # 防抖处理
        current_time = time.time()
        if current_time - self.last_chassis_send_time < self.chassis_send_delay:
            return
        self.last_chassis_send_time = current_time
        
        # 创建 Twist 消息
        twist_msg = geometry_msgs.msg.Twist()
        
        # 设置线速度（前进/后退）
        twist_msg.linear.x = self.linear_vel_slider.get()
        
        # 设置角速度（旋转）
        twist_msg.angular.z = self.angular_vel_slider.get()
        
        # 发布控制命令
        self.cmd_vel_pub.publish(twist_msg)
        
        # 显示当前速度
        self.get_logger().info(f"底盘控制: 线速度={twist_msg.linear.x:.2f} m/s, 角速度={twist_msg.angular.z:.2f} rad/s")

    def stop_chassis(self):
        """停止底盘"""
        self.linear_vel_slider.set(0.0)
        self.angular_vel_slider.set(0.0)
        self.send_chassis_cmd()

    def sync_grippers(self, _=None):
        """同步两个手指：拖动主滑块时同时设置手指1和手指2"""
        val = self.grip_master_slider.get()
        self.grip7_slider.set(val)
        self.grip8_slider.set(val)
        self.send_pose()

    def send_pose(self):
        """发送机械臂位姿"""
        # 获取滑块数据（实时读取当前值）
        arm_positions = [s.get() for s in self.arm_sliders]
        grip7_position = self.grip7_slider.get()
        grip8_position = self.grip8_slider.get()

        # ==========================================
        # 组装并发布机械臂消息 (1-6轴)
        # ==========================================
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        arm_point = JointTrajectoryPoint()
        arm_point.positions = arm_positions
        arm_point.time_from_start.sec = 0
        arm_point.time_from_start.nanosec = 200000000  # 0.2秒
        arm_msg.points.append(arm_point)
        self.arm_pub.publish(arm_msg)

        # ==========================================
        # 组装并发布夹爪消息 (7-8轴，独立控制)
        # ==========================================
        grip_msg = JointTrajectory()
        grip_msg.joint_names = ['joint7', 'joint8']
        grip_point = JointTrajectoryPoint()
        # joint7: 0 ~ 0.04, joint8: 0 ~ -0.04 (取反)
        grip_point.positions = [grip7_position, -grip8_position]
        grip_point.time_from_start.sec = 0
        grip_point.time_from_start.nanosec = 200000000
        grip_msg.points.append(grip_point)
        self.grip_pub.publish(grip_msg)

        # 终端打印
        self.get_logger().info(
            f"机械臂位姿: {[round(p, 2) for p in arm_positions]} | "
            f"手指1: {grip7_position:.3f} | 手指2: {grip8_position:.3f}"
        )

    def on_close(self):
        """关闭程序"""
        # 停止底盘
        self.stop_chassis()
        
        self.get_logger().info("关闭控制台...")
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui = SimpleArmGUI()
    # 启动 Tkinter 的主循环
    gui.root.mainloop()

if __name__ == '__main__':
    main()