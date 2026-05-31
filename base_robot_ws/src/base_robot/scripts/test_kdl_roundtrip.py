#!/usr/bin/env python3
"""KDL 正逆运动学往返验证脚本（ROS 版本）"""

import numpy as np
from transforms3d.axangles import axangle2mat
import time

import PyKDL
from kdl_parser_py.urdf import treeFromString

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    # 初始化 ROS
    rclpy.init()
    node = Node("test_kdl", allow_undeclared_parameters=True)
    
    # 从参数服务器获取 URDF
    urdf_string = node.get_parameter_or(
        'robot_description',
        rclpy.parameter.Parameter('robot_description', rclpy.Parameter.Type.STRING, '')
    ).value
    
    if not urdf_string:
        print("ERROR: robot_description 参数为空！请确保 robot_state_publisher 在运行。")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    print(f"URDF 长度: {len(urdf_string)}")
    
    # 构建 KDL 链
    _, tree = treeFromString(urdf_string)
    chain = tree.getChain("arm_base_link", "link6")
    print(f"Chain has {chain.getNrOfJoints()} joints")
    
    # 构建求解器
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver_pos = PyKDL.ChainIkSolverPos_NR(
        chain, fk_solver, ik_solver_vel, maxiter=500, eps=1e-5
    )
    
    # ============================================================
    # 测试 1: FK → IK 往返一致性
    # ============================================================
    print("\n[Test 1] FK → IK 往返一致性")
    print("-" * 40)
    
    test_angles = [0.0, -0.5, 0.0, -1.0, 0.0, 0.5]
    
    # FK
    kdl_arr = PyKDL.JntArray(6)
    for i, a in enumerate(test_angles):
        kdl_arr[i] = a
    end_frame = PyKDL.Frame()
    fk_solver.JntToCart(kdl_arr, end_frame)
    pos = np.array([end_frame.p.x(), end_frame.p.y(), end_frame.p.z()])
    rot = np.array([[end_frame.M[i,j] for j in range(3)] for i in range(3)])
    
    print(f"初始角度: {np.round(test_angles, 3)}")
    print(f"FK 位姿: pos={np.round(pos, 4)}")
    
    # IK
    target_frame = PyKDL.Frame(
        PyKDL.Rotation(*[rot[i,j] for i in range(3) for j in range(3)]),
        PyKDL.Vector(*pos)
    )
    result = PyKDL.JntArray(6)
    ik_ret = ik_solver_pos.CartToJnt(kdl_arr, target_frame, result)
    ik_angles = [result[i] for i in range(6)]
    
    angle_err = np.max(np.abs(np.array(test_angles) - np.array(ik_angles)))
    status = "✅ PASS" if ik_ret >= 0 and angle_err < 0.01 else "❌ FAIL"
    print(f"IK 返回码: {ik_ret}, 角度误差: {angle_err:.2e} rad  {status}")
    
    # ============================================================
    # 测试 2: 模拟 OpenVLA delta action
    # ============================================================
    print("\n[Test 2] 模拟 OpenVLA Action 叠加")
    print("-" * 40)
    
    action = np.array([0.1, 0.05, -0.05, 0.1, 0.1, 0.2, 0.5])
    position_scale = 0.5
    rotation_scale = 0.5
    
    delta_pos = action[0:3] * position_scale
    target_pos = pos + delta_pos
    
    delta_aa = action[3:6] * rotation_scale
    delta_norm = np.linalg.norm(delta_aa)
    if delta_norm > 1e-6:
        delta_rot = axangle2mat(delta_aa / delta_norm, delta_norm)
        target_rot = delta_rot @ rot
    else:
        target_rot = rot
    
    print(f"当前位姿: {np.round(pos, 4)}")
    print(f"目标位姿: {np.round(target_pos, 4)}")
    
    # IK
    tf = PyKDL.Frame(
        PyKDL.Rotation(*[target_rot[i,j] for i in range(3) for j in range(3)]),
        PyKDL.Vector(*target_pos)
    )
    res = PyKDL.JntArray(6)
    ik_ret = ik_solver_pos.CartToJnt(kdl_arr, tf, res)
    
    if ik_ret >= 0:
        target_joints = [res[i] for i in range(6)]
        print(f"✅ IK 成功: {np.round(target_joints, 3)}")
    else:
        print(f"❌ IK 失败: 返回码={ik_ret}")
    
    # ============================================================
    # 测试 3: 发布抬升指令到 /vla_instruction
    # ============================================================
    print("\n[Test 3] 发布抬升指令")
    print("-" * 40)

    pub = node.create_publisher(String, '/vla_instruction', 10)

    # 等待 publisher 就绪
    time.sleep(1.0)

    msg = String()
    msg.data = 'lift the arm up by 15 cm'
    pub.publish(msg)
    print(f"✅ 已发布指令: '{msg.data}'")

    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    print("\n验证完成！")

if __name__ == "__main__":
    main()