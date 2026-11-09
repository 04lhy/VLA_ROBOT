import os
from pathlib import Path

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

    # VLA 回合 GT 脱困轨迹（东南桌演示段）。数据集在仓库 openvla/datasets/ 下，
    # 不在 ROS install 树里；从 share 目录回退 5 级得到仓库根：
    #   <WS>/base_robot_ws/{install|build}/base_robot/share/base_robot
    default_gt_ep = Path(urdf_tutorial_path).resolve().parents[4] / \
        'openvla' / 'datasets' / 'gazebo_4room_v2' / 'ep_0004'

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF的绝对路径'
    )

    action_declare_arg_vla_instruction = launch.actions.DeclareLaunchArgument(
        name='vla_instruction',
        default_value='',
        description='VLA 起手指令；空=桥接等 /vla_instruction topic（agent 巡检流程默认）'
    )

    # headless 开关（2026-11-07）：gui:=false 全程不起 gzclient。
    # 不要再用"起 GUI 后中途 pkill gzclient"省渲染——实测定罪（/tmp 二分实验
    # A/B/C 通、D 杀完 RPC 立即挂）：SIGTERM 掉已连接的 gzclient 会毒死
    # gzserver 的 transport **入向消息泵**，之后所有 `gz model -p/-x`（dock
    # teleport/回读）永久挂起；pose/info 等纯订阅方向不受影响，所以现场
    # 看起来"话题都在、就是传送没反应"。无 GUI 从起就不存在这条中毒路径。
    action_declare_arg_gui = launch.actions.DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='false=不启动 gzclient（headless，巡检/工位自动化用）'
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
            ('verbose', 'true'),
            ('gui', launch.substitutions.LaunchConfiguration('gui'))
        ]
    )

    # [2026-11-07 三连 RUN2 实录] 默认 --service-call-timeout=10s：冷启动 gzserver
    # 正忙着载世界，load_controller 的响应 10s 没回到 spawner 就被 DDS 丢弃
    # （"failed to send response ... client will not receive response"），spawner
    # 重试撞上 "already loaded" 直接 FATAL → arm_controller 永不激活 → 桥接就绪
    # 门死等 180s。机器慢不是错，把响应窗口放大到 30s 即可。
    # --- 3. 加载 joint_state_broadcaster ---
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--service-call-timeout", "30", "--switch-timeout", "30"],
    )

    # --- 4. 加载 arm_controller ---
    load_arm_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--service-call-timeout", "30", "--switch-timeout", "30"],
    )

    # --- 5. 加载 gripper_controller ---
    load_gripper_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--service-call-timeout", "30", "--switch-timeout", "30"],
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
            # v4 帧堆叠权重部署参数（第十q轮定案配方，详见 GAZEBO_FINETUNE.md §4/§6）。
            # 反归一化已是原始单位，position/rotation_scale 保持 1.0；
            # max_angular_step 放宽到 0.2 否则倾倒段被削顶。
            # instruction 走 topic 门控（agent / ros2 topic pub 起手）：
            #   手动指定 vla_instruction:=... 则 launch 即起手（桥接自动预热后再跑）；
            #   默认 ''=等 /vla_instruction。桥接 auto_warm 已内置 f1 预热位。
            {'instruction': launch.substitutions.LaunchConfiguration('vla_instruction')},
            {'position_scale': 1.0},
            {'rotation_scale': 1.0},
            {'max_linear_step': 0.04},
            {'max_angular_step': 0.2},
            {'stack_frames': True},
            {'escape_gt_file': str(default_gt_ep / 'actions.npy')},
            {'escape_gt_meta': str(default_gt_ep / 'meta.json')},
            {'escape_stall_frames': 4},
            {'escape_phase_gap': 8},
            {'min_ee_z': 0.19},
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_declare_arg_vla_instruction,
        action_declare_arg_gui,
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
