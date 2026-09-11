from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init(args=None)
    
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active() # 等待导航器激活

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f}m')
        #超时自动取消
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
            navigator.cancelTask()

    #判断结果
    result = navigator.getResult()
    navigator.get_logger().info(str(result))

    rclpy.spin(navigator)
    rclpy.shutdown()
