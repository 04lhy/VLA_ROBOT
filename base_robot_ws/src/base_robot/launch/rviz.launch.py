import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name_in_model = 'base_robot'
    urdf_tutorial_path = get_package_share_directory('base_robot')
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', '4_robot', '4_robot.urdf.xacro')
    my_slam_params_file = os.path.join(urdf_tutorial_path, 'config', 'my_slam_params.yaml')
    
    # --- 1. 声明参数 ---
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF的绝对路径'
    )

    # --- 2. 机器人描述 (Xacro) ---
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # --- 3. 节点配置 ---

    # Robot State Publisher: 必须开启 use_sim_time 解决 TF 树时间戳旧的问题
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    # RViz2: 开启仿真时间以对齐坐标轴
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    # --- 4. Slam Toolbox 配置 (重点修改) ---
    
    # 获取你自己包下的配置文件路径
    # 请确保你已经把系统默认的 yaml 拷贝到了 base_robot/config/my_mapper_params.yaml

    slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            my_slam_params_file, # 加载你修改过雷达话题名称的自定义配置文件
            {'use_sim_time': True}
        ]
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        # 注意：仿真时建议去掉 joint_state_publisher，因为它会发布不带仿真时间戳的数据干扰 TF
        # joint_state_publisher_node, 
        rviz_node,
        slam_toolbox_node
    ])