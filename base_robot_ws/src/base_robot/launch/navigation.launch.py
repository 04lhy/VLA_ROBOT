import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    base_robot_dir = get_package_share_directory('base_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    rviz_config_dir = os.path.join(base_robot_dir, 'config', 'nav2_default_view.rviz')

    # 声明launch配置参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    # ========== 核心修改：地图路径调整 ==========
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map',
        default=os.path.join(base_robot_dir, 'maps', 'room1.yaml')
    )
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(base_robot_dir, 'config', 'my_nav_params.yaml')
    )

    # ========== AMCL 初始位姿发布 ==========
    # Nav2 Humble 的 AMCL 不再读取 yaml 里的 set_initial_pose / initial_pose 参数，
    # 需通过 /initialpose 话题手动发布。坐标需与 test.world 中 base_robot 模型的
    # <pose> 保持一致（x y yaw）。
    initial_pose_node = launch_ros.actions.Node(
        package='base_robot',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'x': 2.0279},
            {'y': 1.8948},
            {'yaw': 1.7867},
            {'delay': 15.0},
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        # 声明可配置的launch参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),

        # 包含Nav2的bringup启动文件
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            # 传递参数到Nav2启动文件
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items()
        ),

        # 发布 AMCL 初始位姿（延迟后执行，确保 AMCL 已启动）
        initial_pose_node,

        # 启动RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])