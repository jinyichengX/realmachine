#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace rrt_planner
{

// ---------------- 基础几何 ----------------
struct Vec2
{
  double x, y;
  Vec2(double x_, double y_): x(x_), y(y_) {}
};

class PointNode
{
public:
  Vec2 point;
  int parent_idx = -1;              // 父节点索引，-1 表示无父（根节点）
  explicit PointNode(const Vec2 p): point(p) {}
};

// ---------------- RRT 算法本体 ----------------
// 和原来那份 RRT 的流程完全一致（采样 -> 找最近节点 -> 按步长生长 -> 回溯），
// 只把"撞圆障碍物"换成了"查代价地图栅格"，地图边界也不再手填，直接从代价地图取。
class RRTPlanner
{
public:
  RRTPlanner(nav2_costmap_2d::Costmap2D * costmap, double step_size = 0.5, int iter_max = 5000);
  ~RRTPlanner() = default;

  // 规划成功返回 true，path 按「起点 -> 终点」顺序写入；失败返回 false，path 为空
  bool Plan(const Vec2 & start, const Vec2 & goal, std::vector<Vec2> & path);

  // 获取最终生成的回溯路径（起点 -> 终点）
  const std::vector<Vec2> & GetPath() const {return path_;}

  // 获取随机树上的全部节点（可用于可视化调试）
  const std::vector<PointNode> & GetNodeArray() const {return node_array_;}

private:
  // 地图边界内均匀生成随机点
  Vec2 GenRandomPoint();

  // 检查线段 p1->p2 是否穿过障碍。判定与直线规划器保持一致：
  // 253/254 视为障碍，255（未知区域）视为可通行。
  // escape_only_lethal 为 true 时只把 254 当障碍：机器人有可能正停在膨胀区
  // （比如挤在门口），此时必须允许它先"走出来"，否则永远规划不出路径。
  bool SegmentFree(const Vec2 & p1, const Vec2 & p2, bool escape_only_lethal) const;

  // 返回 node_array 中距离 p 最近的节点索引；数组为空返回 -1
  int SearchNearestNode(const std::vector<PointNode> & node_array, const Vec2 & p) const;

  static Vec2 Normalize(const Vec2 & v);

  void AddNewPoint(const Vec2 & p, int par_idx);

  std::random_device rd_;
  std::mt19937 gen_;

  nav2_costmap_2d::Costmap2D * costmap_;

  Vec2 map_min_, map_max_;          // 地图边界（世界坐标）
  double step_size_;                // 单次路径步长
  int iter_max_;                    // 最大迭代次数

  std::vector<PointNode> node_array_;
  std::vector<Vec2> path_;          // 最终生成的回溯路径
};

// ---------------- Nav2 插件壳 ----------------
// Nav2 只认 nav2_core::GlobalPlanner 这个接口，所以用这层壳把上面的算法包起来
class RRTPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
  RRTPlannerPlugin() = default;
  ~RRTPlannerPlugin() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("rrt_planner")};

  double step_size_{0.5};
  int iter_max_{5000};
};

}  // namespace rrt_planner

#endif  // RRT_PLANNER_HPP
