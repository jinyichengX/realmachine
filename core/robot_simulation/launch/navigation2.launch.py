import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 包路径
    fishbot_navigation2_dir = get_package_share_directory('robot_simulation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    # 创建launch配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', 
        default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return LaunchDescription([
        # 声明新的 launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            description='Use simulation (Gazebo) clock if true',
            default_value='true'),

        launch.actions.DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file to load',
            default_value=map_yaml_path),

        launch.actions.DeclareLaunchArgument(
            'params_file',
            description='Full path to nav2 parameters file',
            default_value=nav2_param_path),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items()
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 将 Nav2 的 /cmd_vel (Twist) 桥接到 fishbot_diff_drive_controller 的 /cmd_vel (Twist)
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel', '/fishbot_diff_drive_controller/cmd_vel_unstamped'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
