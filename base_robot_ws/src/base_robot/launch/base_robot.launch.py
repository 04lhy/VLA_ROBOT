import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    urdf_tutorial_path = get_package_share_directory('base_robot')
    default_model_path = urdf_tutorial_path + '/urdf/4_robot/4_robot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/demo_scene.world'

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
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
        ],
        output='screen'
    )

    # --- 2. Gazebo 启动 ---
    # 注意：demo_scene.world 内已包含 base_robot 模型（含 diff_drive / gazebo_ros2_control /
    # 传感器 / grasp_fix 等插件），因此无需再通过 spawn_entity 生成实体。
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[
            ('world', default_world_path),
            ('verbose', 'true')
        ]
    )

    # --- 3. 加载 joint_state_broadcaster ---
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # --- 4. 加载 arm_controller ---
    load_arm_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # --- 5. 加载 gripper_controller ---
    load_gripper_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # --- 6. 语音播报节点 ---
    tts_node = launch_ros.actions.Node(
        package='base_robot',
        executable='tts_node.py',
        parameters=[
            {'topic_name': '/robot/tts_text'},
            {'voice_rate': 180},
            {'voice_volume': 1.0},
        ]
    )

    # --- 7. 启动机械臂预置位姿（确保腕部相机第一帧就能看到杯子）---
    vla_preset_pose_node = launch_ros.actions.Node(
        package="base_robot",
        executable="vla_preset_pose.py",
        name="vla_preset_pose",
        output='screen'
    )

    # --- 8. 启动 vla_gazebo_bridge ---
    vla_gazebo_bridge_node = launch_ros.actions.Node(
        package="base_robot",
        executable="vla_gazebo_bridge.py",
        name="vla_gazebo_bridge",
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
            # test.world 里腕部摄像头的 sensor 名是 camera_sensor，故话题是 /camera_sensor/image_raw
            {'camera_topic': '/camera_sensor/image_raw'},
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        launch_gazebo,
        tts_node,
        # 等 Gazebo 加载完 world（含 gazebo_ros2_control 插件）后再加载控制器。
        # 用定时器替代原先的 spawn_entity 退出事件，避免 spawn_entity 因
        # "already exists" / 服务超时导致控制器事件链卡住。
        TimerAction(
            period=8.0,
            actions=[load_joint_state_broadcaster],
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_gripper_controller,
                on_exit=[vla_preset_pose_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=vla_preset_pose_node,
                on_exit=[vla_gazebo_bridge_node],
            )
        ),
    ])
