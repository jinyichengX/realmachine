"""启动 PX4 双向桥接: 上行 odom/TF + 下行 Offboard 控制.

用法:
    ros2 launch px4_bridge px4_bridge.launch.py
    ros2 launch px4_bridge px4_bridge.launch.py auto_arm:=true use_target_yaw:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_offboard_arg = DeclareLaunchArgument(
        "auto_offboard",
        default_value="true",
        description="收到 setpoint 流约 1s 后是否自动切换到 OFFBOARD 模式",
    )
    auto_arm_arg = DeclareLaunchArgument(
        "auto_arm",
        default_value="false",
        description="切换 OFFBOARD 后是否自动发送解锁指令 (默认关闭)",
    )
    use_target_yaw_arg = DeclareLaunchArgument(
        "use_target_yaw",
        default_value="false",
        description="是否使用目标位姿中的偏航角控制机头朝向",
    )
    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="odom 话题与 TF 的父坐标系",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="odom 话题与 TF 的子坐标系",
    )

    px4_odom_bridge = Node(
        package="px4_bridge",
        executable="px4_odom_bridge",
        name="px4_odom_bridge",
        output="screen",
        parameters=[
            {
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
            }
        ],
    )

    px4_offboard_control = Node(
        package="px4_bridge",
        executable="px4_offboard_control",
        name="px4_offboard_control",
        output="screen",
        parameters=[
            {
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "use_target_yaw": LaunchConfiguration("use_target_yaw"),
            }
        ],
    )

    return LaunchDescription(
        [
            auto_offboard_arg,
            auto_arm_arg,
            use_target_yaw_arg,
            odom_frame_arg,
            base_frame_arg,
            px4_odom_bridge,
            px4_offboard_control,
        ]
    )
