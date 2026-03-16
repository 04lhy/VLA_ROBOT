#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk, Scale, HORIZONTAL, Button, Label, Frame, Checkbutton, BooleanVar


class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_joint_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4',
                            'joint5', 'joint6', 'joint7', 'joint8']
        self.arm_sliders = []      # 前6个关节的滑块
        self.grip_slider = None    # 夹爪滑块
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.create_gui()

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Robot Control Panel")
        self.root.geometry("800x650")
        self.root.configure(bg='#1e1e1e')

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', background='#1e1e1e', foreground='#e0e0e0', font=('Arial', 10))
        style.configure('TButton', background='#0078d7', foreground='white', font=('Arial', 10))
        style.map('TButton', background=[('active', '#1a86d9')])
        style.configure('TFrame', background='#1e1e1e')
        style.configure('TCheckbutton', background='#1e1e1e', foreground='#e0e0e0')

        main_frame = Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)

        # 左侧：机械臂控制
        left_frame = Frame(main_frame, bg='#2d2d2d', relief='groove', bd=2, highlightbackground='#3a3a3a')
        left_frame.pack(side='left', fill='both', expand=True, padx=5)

        title_left = Label(left_frame, text="Arm Joint Control", bg='#2d2d2d', fg='#e0e0e0',
                           font=('Arial', 14, 'bold'))
        title_left.pack(pady=5)

        # 前6个关节滑块
        for i in range(6):
            name = self.joint_names[i]
            frame = Frame(left_frame, bg='#2d2d2d')
            frame.pack(pady=2, fill='x', padx=10)
            label = Label(frame, text=f"{name}:", width=10, anchor='w',
                          bg='#2d2d2d', fg='#e0e0e0')
            label.pack(side='left')
            slider = Scale(frame, from_=-3.14, to=3.14, resolution=0.01,
                           orient=HORIZONTAL, length=250,
                           bg='#3a3a3a', fg='#e0e0e0', troughcolor='#1e1e1e',
                           highlightbackground='#0078d7', activebackground='#1a86d9')
            slider.set(0.0)
            slider.pack(side='left', expand=True, fill='x')
            self.arm_sliders.append(slider)

        # Grip滑块（控制joint7和joint8）
        grip_frame = Frame(left_frame, bg='#2d2d2d')
        grip_frame.pack(pady=10, fill='x', padx=10)
        label_grip = Label(grip_frame, text="Grip (open/close):", width=12, anchor='w',
                           bg='#2d2d2d', fg='#e0e0e0')
        label_grip.pack(side='left')
        self.grip_slider = Scale(grip_frame, from_=0.0, to=0.05, resolution=0.001,
                                 orient=HORIZONTAL, length=250,
                                 bg='#3a3a3a', fg='#e0e0e0', troughcolor='#1e1e1e',
                                 highlightbackground='#0078d7', activebackground='#1a86d9')
        self.grip_slider.set(0.0)
        self.grip_slider.pack(side='left', expand=True, fill='x')

        # 发送按钮
        btn_send = Button(left_frame, text="Send Arm Angles", command=self.send_arm_angles,
                          bg='#0078d7', fg='white', activebackground='#1a86d9',
                          relief='flat', bd=0, padx=10, pady=5)
        btn_send.pack(pady=10)

        # 右侧：小车控制（保持不变）
        right_frame = Frame(main_frame, bg='#2d2d2d', relief='groove', bd=2, highlightbackground='#3a3a3a')
        right_frame.pack(side='right', fill='both', expand=True, padx=5)

        title_right = Label(right_frame, text="Mobile Base Control", bg='#2d2d2d', fg='#e0e0e0',
                            font=('Arial', 14, 'bold'))
        title_right.pack(pady=5)

        # 线速度
        frame_linear = Frame(right_frame, bg='#2d2d2d')
        frame_linear.pack(pady=10, fill='x', padx=20)
        label_linear = Label(frame_linear, text="Linear X (m/s):", bg='#2d2d2d', fg='#e0e0e0')
        label_linear.pack(anchor='w')
        self.linear_slider = Scale(frame_linear, from_=-0.5, to=0.5, resolution=0.01,
                                   orient=HORIZONTAL, length=300,
                                   bg='#3a3a3a', fg='#e0e0e0', troughcolor='#1e1e1e',
                                   highlightbackground='#0078d7', activebackground='#1a86d9')
        self.linear_slider.set(0.0)
        self.linear_slider.pack(fill='x')

        # 角速度
        frame_angular = Frame(right_frame, bg='#2d2d2d')
        frame_angular.pack(pady=10, fill='x', padx=20)
        label_angular = Label(frame_angular, text="Angular Z (rad/s):", bg='#2d2d2d', fg='#e0e0e0')
        label_angular.pack(anchor='w')
        self.angular_slider = Scale(frame_angular, from_=-1.0, to=1.0, resolution=0.01,
                                    orient=HORIZONTAL, length=300,
                                    bg='#3a3a3a', fg='#e0e0e0', troughcolor='#1e1e1e',
                                    highlightbackground='#0078d7', activebackground='#1a86d9')
        self.angular_slider.set(0.0)
        self.angular_slider.pack(fill='x')

        # 停止按钮
        btn_stop = Button(right_frame, text="STOP", command=self.stop_robot,
                          bg='#c42b1c', fg='white', activebackground='#d83a2b',
                          font=('Arial', 12, 'bold'), relief='flat', bd=0, padx=20, pady=8)
        btn_stop.pack(pady=20)

        # 自动发送选项
        self.auto_send_arm = BooleanVar(value=True)
        self.auto_send_base = BooleanVar(value=True)
        check_arm = Checkbutton(right_frame, text="Auto-send arm", variable=self.auto_send_arm,
                                bg='#2d2d2d', fg='#e0e0e0', selectcolor='#2d2d2d',
                                activebackground='#2d2d2d', activeforeground='#ffffff',
                                highlightbackground='#0078d7')
        check_arm.pack()
        check_base = Checkbutton(right_frame, text="Auto-send base", variable=self.auto_send_base,
                                 bg='#2d2d2d', fg='#e0e0e0', selectcolor='#2d2d2d',
                                 activebackground='#2d2d2d', activeforeground='#ffffff',
                                 highlightbackground='#0078d7')
        check_base.pack()

        self.after_id = self.root.after(100, self.auto_update)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def send_arm_angles(self):
        angles = [slider.get() for slider in self.arm_sliders]
        grip_val = self.grip_slider.get()
        angles.append(grip_val)      # joint7
        angles.append(-grip_val)     # joint8
        msg = Float64MultiArray()
        msg.data = angles
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Published arm angles: {angles}')

    def send_base_command(self):
        self.linear_x = self.linear_slider.get()
        self.angular_z = self.angular_slider.get()
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.linear_slider.set(0.0)
        self.angular_slider.set(0.0)
        self.send_base_command()

    def auto_update(self):
        if self.auto_send_arm.get():
            self.send_arm_angles()
        if self.auto_send_base.get():
            self.send_base_command()
        self.after_id = self.root.after(100, self.auto_update)

    def on_closing(self):
        if self.after_id:
            self.root.after_cancel(self.after_id)
        self.root.destroy()

    def run(self):
        while rclpy.ok():
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    gui = RobotControlGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()