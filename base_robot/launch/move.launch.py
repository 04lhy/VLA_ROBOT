import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'base_robot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # 默认 xacro 文件路径（已修复内部 $(find) 问题）
    default_model_path = os.path.join(pkg_path, 'urdf', '4_robot', '4_robot.urdf.xacro')

    # 声明 model 参数
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )

    # 动态生成 robot_description（通过 xacro 命令）
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 差速模拟节点（替代 joint_state_publisher_gui）
    fake_diff_drive_node = Node(
    package='base_robot',
    executable='fake_diff_drive.py',
    name='fake_diff_drive',
    parameters=[{
        'wheel_separation': 0.2,
        'wheel_radius': 0.0325,
        'left_joint': 'left_wheel_joint',
        'right_joint': 'right_wheel_joint',
        # 机械臂关节名称（与URDF完全一致）
        'arm_joint_names': ['joint1', 'joint2', 'joint3', 'joint4',
                            'joint5', 'joint6', 'joint7', 'joint8'],
        # 对应的初始位置（弧度）
        'arm_joint_positions': [0.0, 0.5, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0]   # 示例：joint2 弯曲 0.5 rad
    }],
    output='screen'
    )
    arm_gui_node = Node(
    package='base_robot',
    executable='arm_control_gui.py',
    name='arm_control_gui',
    output='screen'
    )
    # RViz2（可指定配置文件，若无则启动空白）
    rviz_config_file = os.path.join(pkg_path, 'config', 'diff_drive.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    return launch.LaunchDescription([
        declare_model_arg,
        robot_state_publisher_node,
        fake_diff_drive_node,
        arm_gui_node,
        rviz_node
    ])
