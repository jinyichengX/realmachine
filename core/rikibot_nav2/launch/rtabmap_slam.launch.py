#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot 三维建图: RTAB-Map (RGB-D)

输入(全部由已有节点提供, 本启动文件不重复启动它们):
  彩色图    /camera/color/image_raw      <- rikibot_camera_cpp
  深度图    /camera/depth/image_raw      <- rikibot_camera_cpp, 已对齐到彩色, 16UC1 单位毫米
  相机内参  /camera/color/camera_info    <- rikibot_camera_cpp
  里程计    TF: odom -> base_link        <- rikibot_base_cpp

TF 树(每条边只有一个发布者):
  map               <- rtabmap 发布
   └─ odom          <- 底盘节点发布 (轮式里程计)
       └─ base_link
           └─ camera_link
               └─ camera_color_optical_frame

已实测确认的事实(排查时别再重复验证):
  深度尺度    中心像素读数 1m->1001mm, 1.8m->1798mm, 3.58m->3579mm, 误差 0.1% 以内
  轮式里程计  直线 1.000m -> 0.992m (1.1%); 原地转 90 度 -> 88~92 度
  深度对齐    D2C 已开启, 彩色与深度都是 640x480
  相机 TF     位置与光学坐标系旋转都已用矩阵验证
  尚存问题    同一物体从不同位置被观测时, 在地图里会错开十几到几十厘米(重影/重复)
              已知嫌疑: camera_info 里的内参没标定过 —— 畸变 D 全填 0, 主点 cx/cy 用图像中心假设

关于 rtabmap 自带的视觉里程计(不要再打开):
  曾用 guess_frame_id 的方式试过, 实测 rtabmap 会认为机器人全程没动, 拒绝添加节点。
  原因未彻底定位, 所以本文件不再提供该选项。

注意:
  不能和 slam_toolbox / amcl 同时运行 —— 它们都会发布 map->odom 一类的关系, 会打架。

用法:
  ros2 launch rikibot_nav2 rtabmap_slam.launch.py
  # 每次重新建图前清空地图数据库
  ros2 launch rikibot_nav2 rtabmap_slam.launch.py rtabmap_args:=-d
  # 排查问题时把 rtabmap 内部日志打出来(能看到位姿和节点增长情况)
  ros2 launch rikibot_nav2 rtabmap_slam.launch.py rtabmap_args:="-d --uinfo"
  # 本机有图形界面时, 直接弹出 rtabmap 的三维视图
  ros2 launch rikibot_nav2 rtabmap_slam.launch.py rtabmap_viz:=true

输出:
  /rtabmap/cloud_map   三维点云地图 (rviz2 里添加 PointCloud2 显示)
  /rtabmap/grid_map    由深度生成的二维栅格图
  /rtabmap/mapGraph    位姿图
  TF: map -> odom      定位修正量
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')

    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/color/camera_info')
    frame_id = LaunchConfiguration('frame_id', default='base_link')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    database_path = LaunchConfiguration(
        'database_path', default=os.path.expanduser('~/.ros/rtabmap.db'))
    rtabmap_viz = LaunchConfiguration('rtabmap_viz', default='false')
    rtabmap_args = LaunchConfiguration('rtabmap_args', default='')

    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value=rgb_topic,
                              description='彩色图话题'),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic,
                              description='深度图话题(需已对齐到彩色)'),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic,
                              description='相机内参话题'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='机器人基座坐标系'),
        DeclareLaunchArgument('odom_frame_id', default_value=odom_frame_id,
                              description='里程计坐标系, 设置后从 TF 读位姿而不是话题'),
        DeclareLaunchArgument('database_path', default_value=database_path,
                              description='地图数据库保存路径'),
        DeclareLaunchArgument('rtabmap_viz', default_value=rtabmap_viz,
                              description='是否弹出 rtabmap 三维视图(需要图形界面)'),
        DeclareLaunchArgument('rtabmap_args', default_value=rtabmap_args,
                              description='额外参数, 例如 -d 表示启动时清空地图'),

        # 复用 rtabmap 官方启动文件, 只把我们实际情况的参数传进去
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'frame_id': frame_id,
                'odom_frame_id': odom_frame_id,
                'database_path': database_path,
                'rtabmap_viz': rtabmap_viz,
                'rtabmap_args': rtabmap_args,

                # 位姿完全来自底盘轮式里程计(经 TF odom->base_link 读取)
                'visual_odometry': 'false',
                'icp_odometry': 'false',

                'approx_sync': 'true',

                # 暂时不融合雷达, 先把 RGB-D 这条链路跑通
                'subscribe_scan': 'false',

                'use_sim_time': 'false',
            }.items(),
        ),
    ])
