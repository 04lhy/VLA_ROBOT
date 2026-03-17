#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk

class SimpleArmGUI(Node):
    def __init__(self):
        super().__init__('simple_arm_gui')
        
        # --- 1. 初始化 ROS 2 发布者 ---
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.get_logger().info("✅ 极简 GUI 控制端已启动！")

        # --- 2. 初始化 Tkinter 界面 ---
        self.root = tk.Tk()
        self.root.title("🦾 机械臂位姿调整台")
        self.root.geometry("400x550")
        
        self.arm_sliders = []
        
        # 创建 1-6 轴机械臂滑块 (-3.14 到 3.14 弧度)
        for i in range(1, 7):
            frame = tk.Frame(self.root)
            frame.pack(fill='x', padx=20, pady=5)
            
            lbl = tk.Label(frame, text=f"Joint {i}", width=8, anchor='w')
            lbl.pack(side='left')
            
            scale = tk.Scale(frame, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, length=250)
            scale.set(0.0)
            scale.pack(side='left', expand=True)
            self.arm_sliders.append(scale)
            
        # 创建夹爪滑块 (0.0 到 0.04 米)
        tk.Frame(self.root, height=2, bg="grey").pack(fill='x', padx=20, pady=10) # 分割线
        grip_frame = tk.Frame(self.root)
        grip_frame.pack(fill='x', padx=20, pady=5)
        
        tk.Label(grip_frame, text="Gripper", width=8, anchor='w').pack(side='left')
        self.grip_slider = tk.Scale(grip_frame, from_=0.0, to=0.04, resolution=0.001, orient=tk.HORIZONTAL, length=250)
        self.grip_slider.set(0.0)
        self.grip_slider.pack(side='left', expand=True)

        # 发送按钮
        btn = tk.Button(self.root, text="发送当前位姿", command=self.send_pose, bg='#0078d7', fg='white', font=('Arial', 12, 'bold'))
        btn.pack(pady=20)

        # 窗口关闭事件绑定
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # --- 3. 启动 ROS 2 刷新定时器 ---
        # 让 Tkinter 帮我们每 50 毫秒跑一次 ROS 2 的事件循环
        self.ros_spin_timer()

    def ros_spin_timer(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self.ros_spin_timer)

    def send_pose(self):
        # 获取滑块数据
        arm_positions = [s.get() for s in self.arm_sliders]
        grip_position = self.grip_slider.get()

        # ==========================================
        # 组装并发布机械臂消息 (1-6轴)
        # ==========================================
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        arm_point = JointTrajectoryPoint()
        arm_point.positions = arm_positions
        arm_point.time_from_start.sec = 1
        arm_point.time_from_start.nanosec = 0 
        arm_msg.points.append(arm_point)
        self.arm_pub.publish(arm_msg)

        # ==========================================
        # 组装并发布夹爪消息 (7-8轴)
        # ==========================================
        grip_msg = JointTrajectory()
        grip_msg.joint_names = ['joint7', 'joint8']
        grip_point = JointTrajectoryPoint()
        grip_point.positions = [grip_position, -grip_position]
        grip_point.time_from_start.sec = 1
        grip_point.time_from_start.nanosec = 0
        grip_msg.points.append(grip_point)
        self.grip_pub.publish(grip_msg)

        # 终端打印确认
        self.get_logger().info(f"🚀 位姿已发送 -> 臂: {[round(p, 2) for p in arm_positions]} | 爪: {grip_position}")

    def on_close(self):
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