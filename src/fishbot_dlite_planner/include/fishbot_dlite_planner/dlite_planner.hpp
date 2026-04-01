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

  // 指向通往目标的下一个节点 (用于计算搜索阶段的转向角)
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
  void mapToWorld(double mx, double my, double& wx, double& wy);

  double heuristic(const Node* a, const Node* b);

  // ====== 路径后处理 (达到图2效果的核心) ======
  // 智能抽稀：只保留关键拐点和间隔一定距离的点，作为平滑的控制点
  std::vector<geometry_msgs::msg::PoseStamped> smartDownsample(
      const std::vector<geometry_msgs::msg::PoseStamped>& path, double min_dist);
      
  // 全局 B 样条平滑 (改写为更丝滑的版本)
  std::vector<geometry_msgs::msg::PoseStamped> applyBSplineSmoothing(
      const std::vector<geometry_msgs::msg::PoseStamped>& path, int degree = 3, int resolution = 10);

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
  bool use_potential_field_;    // 开启势场才能让路径远离障碍
  bool use_turning_penalty_;      
  double turning_penalty_weight_; 

  // [新增] 平滑控制参数
  double downsample_distance_;    // 抽稀距离 (决定平滑的程度，值越大越平滑)
  int bspline_degree_;            // B样条阶数 (通常为3，代表三次曲线)
  int bspline_resolution_;        // 两个控制点之间的插值数

  std::vector<Node> graph_grid_; 
  int size_x_, size_y_;
  
  std::set<Node*, NodeCompare> open_list_;
  
  Node* start_node_ptr_ = nullptr;
  Node* goal_node_ptr_ = nullptr;
  Node* last_start_node_ptr_ = nullptr;
  
  double km_; 
  
  std::vector<float> static_distance_map_;
  bool is_static_map_ready_ = false;

  nav_msgs::msg::Path last_plan_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_plan_ = false;
};

}  // namespace fishbot_dlite_planner

#endif