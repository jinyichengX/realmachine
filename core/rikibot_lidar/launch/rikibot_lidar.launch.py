#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot 激光雷达启动 (思岚 RPLidar A1)
- 串口: 自动识别 CP2102(10c4:ea60), 也可 udev 映射 /dev/rikilidar
- 波特率: 115200
- frame_id: laser_link
- 话题: /scan

用法:
    ros2 launch rikibot_lidar rikibot_lidar.launch.py
    可覆盖参数:
    ros2 launch rikibot_lidar rikibot_lidar.launch.py serial_port:=/dev/ttyUSB0
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_lidar_port():
    """按 CP2102(10c4:ea60) 找雷达串口, 不依赖 ttyUSB 编号"""
    if os.path.exists('/dev/rikilidar'):
        return '/dev/rikilidar'
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid == 0x10c4 and p.pid == 0xea60:
                return p.device
    except Exception:
        pass
    for node in glob.glob('/sys/class/tty/ttyUSB*'):
        d = os.path.realpath(node + '/device')
        while True:
            vf = os.path.join(d, 'idVendor')
            if os.path.exists(vf):
                try:
                    v = open(vf).read().strip()
                    p = open(os.path.join(d, 'idProduct')).read().strip()
                    if v == '10c4' and p == 'ea60':
                        return '/dev/' + os.path.basename(node)
                except Exception:
                    pass
                break
            par = os.path.dirname(d)
            if par == d:
                break
            d = par
    return '/dev/ttyUSB1'   # 找不到时兜底


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default=_find_lidar_port())
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_link')
    topic_name = LaunchConfiguration('topic_name', default='scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='激光雷达串口设备'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
                              description='串口波特率 (A1=115200)'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='激光数据坐标系'),
        DeclareLaunchArgument('topic_name', default_value=topic_name,
                              description='发布的激光话题名'),
        DeclareLaunchArgument('inverted', default_value=inverted,
                              description='是否反转扫描方向'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate,
                              description='角度补偿'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode,
                              description='扫描模式 (留空=标准模式; 本雷达支持 Standard/Express/Boost/Stability)'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'topic_name': topic_name,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
