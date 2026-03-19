import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    base_robot_dir = get_package_share_directory('base_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ========== 核心修改：直接指定你的 RViz 配置文件绝对路径 ==========
    rviz_config_dir = '/home/lhy/Desktop/VLA_ROBOT/base_robot_ws/src/base_robot/config/nav2_default_view.rviz'

    # 声明launch配置参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    # ========== 核心修改：地图路径调整 ==========
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map',
        default=os.path.join(base_robot_dir, 'maps', 'room.yaml')
    )
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(base_robot_dir, 'config', 'my_nav_params.yaml')
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