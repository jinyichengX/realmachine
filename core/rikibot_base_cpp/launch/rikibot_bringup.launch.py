#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot 底盘 bringup (C++ 节点版, 对应 py 包 rikibot_bringup 的 launch):
  - rikibot_base_node  (C++ 底盘驱动: cmd_vel/odom/TF/imu/battery)
  - base_link->laser_link / imu_link 静态 TF (x4 几何, 后续有 URDF 可移除)

雷达单独启动:  ros2 launch rikibot_lidar rikibot_lidar.launch.py
用法:
  ros2 launch rikibot_base_cpp rikibot_bringup.launch.py
  (串口默认自动选择: 依次尝试 /dev/rikibase 与 /dev/serial/by-id 稳定路径, 无需手动指定)
  (手动覆盖: serial_port:=/dev/ttyUSB1)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='',
                              description='底盘串口; 留空表示自动选择 (/dev/rikibase 或 by-id 路径)'),

        Node(
            package='rikibot_base_cpp',
            executable='base_node',
            name='rikibot_base_node',
            parameters=[{'serial_port': serial_port}],
            output='screen',
            emulate_tty=True,
        ),

        # x4 几何: base_link -> laser_link (0.0158, 0, 0.1706)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.0158', '0.0', '0.1706', '0', '0', '0',
                        'base_link', 'laser_link']),
        # base_link -> imu_link
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.038', '-0.008', '0.085', '0', '0', '0',
                        'base_link', 'imu_link']),
    ])
