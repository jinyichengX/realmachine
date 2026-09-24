#include "rrt_planner/rrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(nav2_costmap_2d::Costmap2D * costmap, double step_size, int iter_max)
: rd_(), gen_(rd_()), costmap_(costmap),
  map_min_(0.0, 0.0), map_max_(0.0, 0.0),
  step_size_(step_size), iter_max_(iter_max)
{
}

Vec2 RRTPlanner::GenRandomPoint()
{
  std::uniform_real_distribution<double> dis_x(map_min_.x, map_max_.x);
  std::uniform_real_distribution<double> dis_y(map_min_.y, map_max_.y);
  return Vec2(dis_x(gen_), dis_y(gen_));
}

bool RRTPlanner::SegmentFree(const Vec2 & p1, const Vec2 & p2, bool escape_only_lethal) const
{
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  const double distance = std::sqrt(dx * dx + dy * dy);

  // 采样间隔取半个栅格，保证不会"跳过"任何一个格子
  const double sample = costmap_->getResolution() * 0.5;
  const int steps = std::max(1, static_cast<int>(distance / sample));

  const unsigned char blocked_from = escape_only_lethal ?
    nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

  for (int i = 1; i <= steps; ++i) {   // 从 i=1 开始：起点所在格不检查，否则机器人站在膨胀区里永远走不出去
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    unsigned int mx = 0, my = 0;
    if (!costmap_->worldToMap(p1.x + t * dx, p1.y + t * dy, mx, my)) {
      return false;             // 越出地图边界，视为不可通行
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= blocked_from && cost < nav2_costmap_2d::NO_INFORMATION) {
      return false;             // 253/254 是障碍；255 是未知区域，当作可通行
    }
  }
  return true;
}

int RRTPlanner::SearchNearestNode(const std::vector<PointNode> & node_array, const Vec2 & p) const
{
  if (node_array.empty()) {
    return -1;
  }

  int nearest_idx = 0;
  double dx0 = node_array[0].point.x - p.x;
  double dy0 = node_array[0].point.y - p.y;
  double min_dist_pow = dx0 * dx0 + dy0 * dy0;

  for (size_t i = 1; i < node_array.size(); i++) {
    double dx = node_array[i].point.x - p.x;
    double dy = node_array[i].point.y - p.y;
    double dist_pow = dx * dx + dy * dy;
    if (dist_pow < min_dist_pow) {
      min_dist_pow = dist_pow;
      nearest_idx = static_cast<int>(i);
    }
  }
  return nearest_idx;
}

Vec2 RRTPlanner::Normalize(const Vec2 & v)
{
  double mod = std::sqrt(v.x * v.x + v.y * v.y);
  if (mod < 1e-9) {
    return Vec2(0.0, 0.0);
  }
  return Vec2(v.x / mod, v.y / mod);
}

void RRTPlanner::AddNewPoint(const Vec2 & p, int par_idx)
{
  PointNode new_pn(p);
  new_pn.parent_idx = par_idx;   // 存索引，不存指针（vector 扩容会使指针失效）
  node_array_.push_back(new_pn);
}

bool RRTPlanner::Plan(const Vec2 & start, const Vec2 & goal, std::vector<Vec2> & path)
{
  path.clear();
  path_.clear();
  node_array_.clear();

  // 地图边界直接取自代价地图（原点 + 尺寸），不再手工传参
  map_min_ = Vec2(costmap_->getOriginX(), costmap_->getOriginY());
  map_max_ = Vec2(
    costmap_->getOriginX() + costmap_->getSizeInMetersX(),
    costmap_->getOriginY() + costmap_->getSizeInMetersY());

  // 起点作为根节点
  node_array_.push_back(PointNode(start));

  for (int iter = 0; iter < iter_max_; ++iter) {
    Vec2 random = GenRandomPoint();

    int nearest_idx = SearchNearestNode(node_array_, random);
    if (nearest_idx == -1) {
      return false;
    }

    // 标准化向量并乘以步长
    const Vec2 & nearest = node_array_[nearest_idx].point;
    Vec2 dir = Normalize(Vec2(random.x - nearest.x, random.y - nearest.y));
    if (dir.x == 0.0 && dir.y == 0.0) {
      continue;
    }
    Vec2 new_point(nearest.x + dir.x * step_size_, nearest.y + dir.y * step_size_);

    // 检查新边是否碰撞，碰撞则丢弃该点
    const bool from_root = (nearest_idx == 0);
    if (!SegmentFree(nearest, new_point, from_root)) {
      continue;
    }

    AddNewPoint(new_point, nearest_idx);

    const double dx = new_point.x - goal.x;
    const double dy = new_point.y - goal.y;
    if (std::sqrt(dx * dx + dy * dy) <= step_size_ && SegmentFree(new_point, goal, false)) {
      // 添加终点，终点接在 new_point 后面
      const int new_idx = static_cast<int>(node_array_.size()) - 1;
      AddNewPoint(goal, new_idx);

      // 回溯路径
      for (int idx = static_cast<int>(node_array_.size()) - 1; idx != -1;
        idx = node_array_[idx].parent_idx)
      {
        path_.push_back(node_array_[idx].point);
      }
      std::reverse(path_.begin(), path_.end());

      path = path_;
      return true;
    }
  }
  return false;
}

// ---------------- Nav2 插件壳 ----------------

void RRTPlannerPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("RRTPlannerPlugin: failed to lock the parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".iter_max", rclcpp::ParameterValue(5000));
  node->get_parameter(name_ + ".step_size", step_size_);
  node->get_parameter(name_ + ".iter_max", iter_max_);

  RCLCPP_INFO(
    logger_, "RRTPlanner configured: %s (step_size=%.2f m, iter_max=%d)",
    name_.c_str(), step_size_, iter_max_);
}

void RRTPlannerPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "RRTPlanner cleanup");
}

void RRTPlannerPlugin::activate()
{
  RCLCPP_INFO(logger_, "RRTPlanner activated");
}

void RRTPlannerPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "RRTPlanner deactivated");
}

nav_msgs::msg::Path RRTPlannerPlugin::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = costmap_ros_->get_clock()->now();

  RRTPlanner rrt(costmap_, step_size_, iter_max_);

  std::vector<Vec2> rrt_path;
  const bool found = rrt.Plan(
    Vec2(start.pose.position.x, start.pose.position.y),
    Vec2(goal.pose.position.x, goal.pose.position.y),
    rrt_path);

  if (!found) {
    RCLCPP_WARN(
      logger_, "RRT failed: no path found in %d iterations (step_size=%.2f m)",
      iter_max_, step_size_);
    return plan;   // 空路径，交给 Nav2 走恢复行为
  }

  RCLCPP_INFO(
    logger_, "RRT plan: %zu points, %zu tree nodes",
    rrt_path.size(), rrt.GetNodeArray().size());

  plan.poses.reserve(rrt_path.size());
  for (size_t i = 0; i < rrt_path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = plan.header.frame_id;
    pose.header.stamp = plan.header.stamp;
    pose.pose.position.x = rrt_path[i].x;
    pose.pose.position.y = rrt_path[i].y;
    pose.pose.position.z = 0.0;

    // 朝向取该点到下一点的方向（最后一点沿用前一段），控制器顺着路径走更稳
    double yaw = 0.0;
    if (i + 1 < rrt_path.size()) {
      yaw = std::atan2(rrt_path[i + 1].y - rrt_path[i].y, rrt_path[i + 1].x - rrt_path[i].x);
    } else if (rrt_path.size() > 1) {
      yaw = std::atan2(
        rrt_path[i].y - rrt_path[i - 1].y, rrt_path[i].x - rrt_path[i - 1].x);
    }
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    plan.poses.push_back(pose);
  }

  // 终点用目标位姿的原始朝向，并把所有点的时间戳统一成规划时刻
  plan.poses.back().pose.orientation = goal.pose.orientation;
  for (auto & pose : plan.poses) {
    pose.header.stamp = plan.header.stamp;
  }

  return plan;
}

}  // namespace rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlannerPlugin, nav2_core::GlobalPlanner)
