import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

# 已建好的地图 (slam_toolbox -> map_saver 保存, 位于工作区根目录与 room.pgm 同处)
# 换地图/换位置时直接覆盖: map:=/path/to/xxx.yaml
DEFAULT_MAP = '/root/auto_navigation_realmachine/room.yaml'


def generate_launch_description():
    pkg_dir = get_package_share_directory('rikibot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=DEFAULT_MAP)
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真实机器人必须为 false'),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP,
                              description='已保存地图 yaml 全路径'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Nav2 参数 yaml 全路径'),

        # 启动整套 Nav2 server (map_server/amcl/planner/controller/...)
        # 真实底盘直接消费 /cmd_vel, 无需仿真里的 topic relay
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch',
                             'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': params_file,
            }.items(),
        ),
    ])
