import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 包路径
    pkg_share = get_package_share_directory('robot_simulation')

    # 默认 xacro 和 world 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf/fishrot/fishbot.xacro')
    default_world_path = os.path.join(pkg_share, 'world/warehouse_world.sdf') #字符串拼接路径

    # 运行时参数：模型路径、世界路径
    model_arg = DeclareLaunchArgument(
        'model', default_value=str(default_model_path),
        description='fishbot xacro 文件的绝对路径'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=str(default_world_path),
        description='Gazebo 世界 SDF 文件的绝对路径'
    )

    # WSL2 需要软件渲染，否则 Gazebo 黑屏/崩溃
    libgl_env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # 将 xacro 展开为 URDF，注入 /robot_description 参数
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 1. Robot State Publisher：广播 TF + 发布 /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2. Joint State Publisher：发布关节状态（驱动连续转动）
    #joint_state_publisher = Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #   parameters=[{
    #
    #   }],
    #    output='screen'
    #)


    # 3. 启动 Gazebo Fortress（使用 ros_gz_sim 的官方 launch）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    # 4. Spawn 机器人到 Gazebo（ros_gz_sim 内部自动 URDF→SDF 转换）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fishbot', #gz侧会基于这个名字生成话题名称，/model/fishbot/xxx（xxx是话题名称），只用于gz内部通信，除非用ros_gz_bridge桥接
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )

    # 5. Joint State Broadcaster spawner：等 spawn 完成后再加载
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_joint_state_broadcaster'],
        output='screen'
    )

    # 6. Effort Controller spawner：等 spawn 完成后再加载
    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_effort_controller'],
        output='screen'
    )

    # 7. Diff Drive Controller spawner：等 spawn 完成后再加载
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_diff_drive_controller'],
        output='screen'
    )

    jsb_spawn_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner, effort_spawner, diff_drive_spawner],
        )
    )

    # 8. 修复激光雷达 frame_id：GZ 生成的 frame_id 带斜杠不兼容 TF 树
    #     创建 fishbot/base_footprint/laser_sensor → base_footprint 的零偏移静态变换
    scan_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint',
                   'fishbot/base_footprint/laser_sensor'],
        output='screen'
    )

    # 5. ros_gz_bridge 桥接：把 Gazebo Transport 话题映射为 ROS 2 话题
    #     - /cmd_vel: ROS → Gazebo，导航栈下发速度指令给 DiffDrive 插件
    #     - /odom:    Gazebo → ROS，DiffDrive 插件发布的里程计
    #     - /tf:      Gazebo → ROS，DiffDrive 插件发布的 odom→base_footprint 变换
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ## Gazebo → ROS（仿真时钟）
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ## Gazebo → ROS（激光扫描数据）
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )

    return LaunchDescription([
        libgl_env,
        model_arg,
        world_arg,
        robot_state_publisher,
        #joint_state_publisher,
        gz_sim,
        spawn_robot,
        jsb_spawn_event,
        gz_bridge,
        scan_tf_fix,
    ])
