/**
 * file: src/fishbot_dlite_planner/include/fishbot_dlite_planner/dlite_planner.hpp
 */
#ifndef FISHBOT_DLITE_PLANNER__DLITE_PLANNER_HPP_
#define FISHBOT_DLITE_PLANNER__DLITE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <set>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace fishbot_dlite_planner
{

struct Node {
  int x, y;
  int index;
  double g = std::numeric_limits<double>::infinity();
  double rhs = std::numeric_limits<double>::infinity();
  std::pair<double, double> key;
  unsigned char cached_cost = 0; 
  bool is_in_open = false; 

  // [修改] 新增指针，指向通往目标的下一个节点 (用于计算搜索阶段的转向角)
  Node* next_node = nullptr; 
};

struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const {
    if (a->key != b->key) {
      return a->key < b->key;
    }
    return a->index < b->index; 
  }
};

class DLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DLitePlanner();
  ~DLitePlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // D* Lite Core
  std::pair<double, double> calculateKey(const Node* s);
  
  // [修复]在此处添加 getExtraCost 的声明
  double getExtraCost(const Node* n);
  
  double getCost(const Node* a, const Node* b);
  void updateVertex(Node* u);
  void computeShortestPath();
  
  // Helpers
  inline Node* getNode(int x, int y);
  inline Node* getNode(int index);
  
  // 地图变化扫描
  bool scanMapChanges();
  
  void computeStaticDistanceMap();

  bool findNearestFreePoint(int& mx, int& my);
  
  bool worldToMap(double wx, double wy, int& mx, int& my);
  void mapToWorld(int mx, int my, double& wx, double& wy);
  
  // [新增] 浮点坐标转换函数声明
  void mapToWorld(double mx, double my, double& wx, double& wy);

  double heuristic(const Node* a, const Node* b);

  // [新增] Field D* 插值路径提取函数声明
  nav_msgs::msg::Path extractPathFieldDStar(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal);

  // Variables
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  
  double interpolation_resolution_;
  double tolerance_;
  double obstacle_weight_; 
  
  bool allow_unknown_; 
  
  // [参数] 算法特性开关
  bool use_potential_field_;    // 是否启用势场
  bool use_bspline_smoothing_;  // 是否启用B样条平滑
  
  // 转向惩罚参数
  bool use_turning_penalty_;      
  double turning_penalty_weight_; 

  // [新增] Field D* 插值开关
  bool use_field_dstar_interpolation_;

  std::vector<Node> graph_grid_; 
  int size_x_, size_y_;
  
  std::set<Node*, NodeCompare> open_list_;
  
  Node* start_node_ptr_ = nullptr;
  Node* goal_node_ptr_ = nullptr;
  Node* last_start_node_ptr_ = nullptr;
  
  double km_; 
  
  std::vector<float> static_distance_map_;
  
  // 标记静态地图是否已经计算完成
  bool is_static_map_ready_ = false;

  // 智能重规划缓存
  nav_msgs::msg::Path last_plan_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_plan_ = false;
};

}  // namespace fishbot_dlite_planner

#endif