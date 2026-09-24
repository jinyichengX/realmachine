import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

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

    # 是否把深度点云加入代价地图(补雷达盲区)。
    # false 时本文件不启动转换节点, nav2_params.yaml 里那个 depth_cloud 障碍源
    # 因为没有数据而变成"哑的", 代价地图行为退化成纯雷达 —— 用于做对照实验,
    # 不需要改任何文件。
    use_depth_scan = LaunchConfiguration('use_depth_scan', default='true')

    # 深度图 -> 三维点云, 作为代价地图的第二个障碍源。
    #
    # 为什么需要它: 雷达装在 0.21m, 扫描面以下的障碍(矮台阶 / 桌子下方的横梁 /
    #   趴在地上的东西)它看不见, 而深度相机看得到。
    #
    # 为什么是"点云"而不是"虚拟激光"(这条是被实机打回来才搞清楚的):
    #   虚拟激光(depthimage_to_laserscan)把三维压成"每方位一个距离", 高度信息只能
    #   靠 scan_height(图像行带)表达, 而行带的几何有硬约束 ——
    #     要让 12cm 的障碍一直可见(直到相机极限 0.6m): 带半宽 >= 570*(0.17-0.12)/0.6 = 47.5 行
    #     要不让地面混进来(3m 以内):                    带半宽 <= 570*0.17/3        = 32.3 行
    #   47.5 > 32.3, 两个条件互相矛盾, 无解。
    #   后果: 障碍靠近到约 0.95m 就掉出行带 -> 虚拟激光改报它后面的墙 -> 射线从 0.6m 清到墙
    #   -> 障碍的标记被清掉 -> 规划器随即规划出穿过它的更近路径 -> 撞上去。
    #   点云保留每个点的真实三维坐标, 高度过滤变成"按真实 z 过滤", 不受距离影响,
    #   障碍一直能标到相机物理极限 0.6m, 上面那个窗口就没了。
    #
    # 为什么不用 pcl_ros 的 passthrough_filter_node 做高度滤波:
    #   这台机器上 pcl_ros 只装了 pcd_to_pointcloud, 没有那两个滤波节点, 跑不起来。
    #   高度过滤改由代价地图的体素网格完成(见 nav2_params.yaml 里的 origin_z/z_voxels),
    #   不需要任何额外依赖。
    #
    # 参数含义:
    #   decimation  隔几个像素取一个点。640x480 全量约 30 万点, 取 8 之后约 4800 点。
    #               代价地图还要对每个点做一次射线清空(全局图 1Hz), 这个量级 4 核 ARM 能扛。
    #               嫌点太稀改小到 4; 卡就改大到 12。
    #   min_depth   相机最近只能测 0.6 米
    #   max_depth   超过 3 米不生成点 —— 结构光远距离不可靠
    #   voxel_size  体素降采样, 再去掉一批重复点
    #   noise_filter_* 故意不开: 半径搜索是这里最贵的一项, 噪点交给代价地图的体素网格处理
    #
    # 话题名已用 ros2 node info 实机核对: 订阅 depth/image + depth/camera_info, 发布 cloud
    # (上一版用 depthimage_to_laserscan 时, 它实际订阅的是 /depth 和 /depth_camera_info,
    #  和文档/惯例都不一致 —— 所以这次没有再靠猜)
    depth_to_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        condition=IfCondition(use_depth_scan),
        parameters=[{
            'decimation': 8,
            'min_depth': 0.6,
            'max_depth': 3.0,
            'voxel_size': 0.03,
            'approx_sync': True,
            'queue_size': 5,
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', '/camera/depth/points'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真实机器人必须为 false'),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP,
                              description='已保存地图 yaml 全路径'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Nav2 参数 yaml 全路径'),
        DeclareLaunchArgument(
            'use_depth_scan', default_value=use_depth_scan,
            description='是否用深度点云补雷达盲区; '
                        'false = 纯雷达基线(对照实验用), 不需要改文件'),

        # 默认启动深度->点云转换(见 use_depth_scan 参数)。
        # 注意相机节点本身要另外启动, 否则这个节点收不到深度图, /camera/depth/points
        # 不会有数据, 代价地图就只剩雷达一个源 —— 和加三维避障之前一样。
        depth_to_cloud,

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
