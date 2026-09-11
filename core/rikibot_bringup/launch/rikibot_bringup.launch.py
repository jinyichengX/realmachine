#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot 底盘 bringup (只负责底盘, 不含雷达):
  - rikibot_base_node   (底盘驱动: cmd_vel/odom/TF/imu/battery)
  - base_link->laser_link / imu_link 静态 TF (x4 几何, 后续有 URDF 可移除)

雷达单独启动:  ros2 launch rikibot_lidar rikibot_lidar.launch.py
用法:
  ros2 launch rikibot_bringup rikibot_bringup.launch.py [serial_port:=/dev/ttyUSB0]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # auto = 底盘节点自己按 CH340(1a86:7523) 查找, 不依赖 ttyUSB 编号
    serial_port = LaunchConfiguration('serial_port', default='auto')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='底盘串口'),

        # Node(
        #     package='rikibot_bringup',
        #     executable='rikibot_base_node',
        #     name='rikibot_base_node',
        #     parameters=[{'serial_port': serial_port}],
        #     output='screen',
        #     emulate_tty=True,
        # ),

        # x4 几何: base_link -> laser_link (0.0158, 0, 0.1706)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.0158', '0.0', '0.1706', '0', '0', '0',
                        'base_link', 'laser_link']),
        # # base_link -> imu_link
        # Node(package='tf2_ros', executable='static_transform_publisher',
        #      arguments=['0.038', '-0.008', '0.085', '0', '0', '0',
        #                 'base_link', 'imu_link']),
    ])
