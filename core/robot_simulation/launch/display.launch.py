import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取当前robot_simulation包的安装路径
    urdf_tutorial_path = get_package_share_directory('robot_simulation')
    # 默认xacro文件路径
    default_model_path = urdf_tutorial_path + '/urdf/first_robot.xacro'

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='xacro 文件的绝对路径'
    )

    # 用 xacro 命令展开 xacro → URDF
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # 1. 机器人状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # 3. RViz可视化节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
