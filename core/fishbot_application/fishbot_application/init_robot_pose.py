from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init(args=None)
    
    navigator = BasicNavigator()
    init_pose = PoseStamped()

    init_pose.header.frame_id = "map"
    init_pose.header.stamp = navigator.get_clock().now().to_msg()

    init_pose.pose.position.x = 4.0
    init_pose.pose.position.y = 2.0
    init_pose.pose.position.z = 0.0
    init_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(init_pose)
    navigator.waitUntilNav2Active() # 等待导航器激活

    rclpy.spin(navigator)
    rclpy.shutdown()
