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
             arguments=['0.0158', '0.0', '0.21', '0', '0', '0',
                        'base_link', 'laser_link']),
        # # base_link -> imu_link
        # Node(package='tf2_ros', executable='static_transform_publisher',
        #      arguments=['0.038', '-0.008', '0.085', '0', '0', '0',
        #                 'base_link', 'imu_link']),

        # 相机几何(实测): base_link -> camera_link
        # x=前 0.12m, y=左 0.023m, z=上 0.17m; 相机平装, 无俯仰/偏航
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.12', '--y', '0.023', '--z', '0.17',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'camera_link']),
        # camera_link -> 镜头光心: 固定旋转, 平移为 0
        # 把"x前/y左/z上"转成相机的"z朝镜头外/x朝右/y朝下", 是常数, 不用测量
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                        '--frame-id', 'camera_link',
                        '--child-frame-id', 'camera_color_optical_frame']),
    ])
