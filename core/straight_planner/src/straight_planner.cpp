#include "straight_planner/straight_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace straight_planner
{

void StraightPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  RCLCPP_INFO(logger_, "StraightPlanner configured: %s", name.c_str());
}

void StraightPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "StraightPlanner cleanup");
}

void StraightPlanner::activate()
{
  RCLCPP_INFO(logger_, "StraightPlanner activated");
}

void StraightPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "StraightPlanner deactivated");
}

nav_msgs::msg::Path StraightPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  
  if (!makePlan(start, goal, plan)) {
    RCLCPP_WARN(logger_, "Failed to create straight line plan. Path crosses obstacle or is invalid.");
  }
  
  return plan;
}

bool StraightPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan)
{
  RCLCPP_INFO(logger_,
    "makePlan start: frame_id=%s, stamp=(%d.%u), position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
    start.header.frame_id.c_str(),
    start.header.stamp.sec, start.header.stamp.nanosec,
    start.pose.position.x, start.pose.position.y, start.pose.position.z,
    start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);

  RCLCPP_INFO(logger_,
    "makePlan goal: frame_id=%s, stamp=(%d.%u), position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
    goal.header.frame_id.c_str(),
    goal.header.stamp.sec, goal.header.stamp.nanosec,
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

  // Check if the line is valid (no collision)
  if (!isLineValid(start, goal)) {
    return false;
  }

  // Generate the straight line path
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = costmap_ros_->get_clock()->now();

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  if (distance < 0.01) {
    // Start and goal are the same, just return the start pose
    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header.stamp = plan.header.stamp;
    plan.poses.push_back(start_pose);
    return true;
  }

  double step_size = costmap_->getResolution(); // 5cm default
  int steps = static_cast<int>(distance / step_size);

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = plan.header.frame_id;
    pose.header.stamp = plan.header.stamp;
    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = start.pose.position.z;
    
    // Interpolate orientation (slerp would be better, but simple linear for now)
    pose.pose.orientation = start.pose.orientation;
    
    plan.poses.push_back(pose);
  }
  
  // Ensure the goal is exactly at the end, with its timestamp normalized
  // to the plan time (RViz may stamp the goal with wall-clock time, which
  // breaks controller_server's transform lookups in simulation time)
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = plan.header.stamp;
  plan.poses.push_back(goal_pose);

  // Normalize timestamps of all poses to the plan time
  for (auto & pose : plan.poses) {
    pose.header.stamp = plan.header.stamp;
  }

  return true;
}

bool StraightPlanner::isLineValid(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < 0.01) {
    return true;
  }

  double step_size = costmap_->getResolution();
  int steps = static_cast<int>(distance / step_size);

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double x = start.pose.position.x + t * dx;
    double y = start.pose.position.y + t * dy;
    
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      // Point is outside the map
      RCLCPP_WARN(logger_, "Point (%.2f, %.2f) is outside the map boundaries!", x, y);
      return false;
    }
    
    unsigned char cost = costmap_->getCost(mx, my);
    
    // Only treat real obstacles (253/254) as invalid; unknown space (255) is traversable
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
      cost < nav2_costmap_2d::NO_INFORMATION) {
      RCLCPP_WARN(logger_, "Line crosses obstacle at (%.2f, %.2f) with cost %d", x, y, cost);
      return false;
    }
  }
  
  return true;
}

}  // namespace straight_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(straight_planner::StraightPlanner, nav2_core::GlobalPlanner)
