#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot_camera_cpp 启动文件: 乐视 LeTMC-520 相机 ROS2 节点(发布 RGB 图像)

用法:
  ros2 launch rikibot_camera_cpp rikibot_camera.launch.py
可覆盖参数:
  color_topic:=/camera/color/image_raw   color_frame_id:=camera_color_optical_frame
  device_uri:=device/default             mirror:=false
说明: 相机 usb 权限先按包内 config/orbbec-usb.rules 在宿主机安装
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    color_frame_id = LaunchConfiguration('color_frame_id', default='camera_color_optical_frame')
    device_uri = LaunchConfiguration('device_uri', default='device/default')
    mirror = LaunchConfiguration('mirror', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('color_topic', default_value=color_topic,
                              description='RGB 彩色图像话题'),
        DeclareLaunchArgument('color_frame_id', default_value=color_frame_id,
                              description='彩色图像坐标系'),
        DeclareLaunchArgument('device_uri', default_value=device_uri,
                              description='Astra 设备 uri'),
        DeclareLaunchArgument('mirror', default_value=mirror,
                              description='是否镜像(默认 false=不镜像)'),

        Node(
            package='rikibot_camera_cpp',
            executable='rikibot_camera_node',
            name='rikibot_camera_node',
            parameters=[{
                'color_topic': color_topic,
                'color_frame_id': color_frame_id,
                'device_uri': device_uri,
                'mirror': mirror,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
