import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 新增：用于处理节点启动顺序的事件处理器
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    robot_name_in_model = 'base_robot'
    urdf_tutorial_path = get_package_share_directory('base_robot')
    default_model_path = urdf_tutorial_path + '/urdf/4_robot/4_robot.urdf.xacro'
    # default_world_path = urdf_tutorial_path + '/world/room_demo.world'
    # 使用简化的房间场景
    default_world_path = urdf_tutorial_path + '/world/simple_room.world'

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name ='model',
        default_value=str(default_model_path),
        description='URDF的绝对路径'
    )
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # --- 1. robot_state_publisher ---
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True} 
        ]
    )

    # --- 2. Gazebo 启动 ---
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[
            ('world', default_world_path),
            ('verbose', 'true')
        ]
    )
    
    # --- 3. 生成实体 ---
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name_in_model],
    )

    # --- 新增 4. 加载 joint_state_broadcaster ---
    # 必须先加载状态发布器，否则控制器会报错
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # --- 新增 5. 加载 arm_controller ---
    # 这里的名字 'arm_controller' 必须和你 controller_position.yaml 里的配置一致
    load_arm_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # --- 新增 6. 加载 gripper_controller ---
    load_gripper_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # --- 7. 语音播报节点 ---
    tts_node = launch_ros.actions.Node(
        package='base_robot',
        executable='tts_node.py',
        parameters=[
            {'topic_name': '/robot/tts_text'},
            {'voice_rate': 180},
            {'voice_volume': 1.0},
        ]
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        tts_node,
        
        # === 恢复控制器加载 ===
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_gripper_controller],
            )
        ),
    ])