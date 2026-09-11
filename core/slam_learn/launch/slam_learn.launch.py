from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动手写 SLAM 节点。
    注意：本节点只做建图，需要先启动机器人仿真（gazebo），
    保证 /scan 话题和 TF 树（odom→base_footprint→laser）已就绪。
    """
    return LaunchDescription([
        Node(
            package='slam_learn',
            executable='simple_slam1',
            name='simple_slam1',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
