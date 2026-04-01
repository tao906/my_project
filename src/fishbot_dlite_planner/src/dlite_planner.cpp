/**
 * file: src/fishbot_dlite_planner/src/dlite_planner.cpp
 */
#include "fishbot_dlite_planner/dlite_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include <queue> 
#include <cmath> 
#include <vector>
#include <algorithm>
#include <chrono> 

PLUGINLIB_EXPORT_CLASS(fishbot_dlite_planner::DLitePlanner, nav2_core::GlobalPlanner)

namespace fishbot_dlite_planner
{

const int DX_LIST[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int DY_LIST[8] = {0, 1, 1, 1, 0, -1, -1, -1};

DLitePlanner::DLitePlanner() : costmap_(nullptr), km_(0.0) {}

DLitePlanner::~DLitePlanner() {}

void DLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  node_ = parent; 

  auto node = parent.lock();
  
  nav2_util::declare_parameter_if_not_declared(node, name + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);
  
  nav2_util::declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  
  // 势力场权重：如果想远离障碍，稍微调大，比如 2.0 - 5.0
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_weight", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".obstacle_weight", obstacle_weight_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_potential_field", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_potential_field", use_potential_field_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_turning_penalty", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_turning_penalty", use_turning_penalty_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".turning_penalty_weight", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".turning_penalty_weight", turning_penalty_weight_);

  // [新增] 达到流线型平滑的关键参数
  // 抽稀距离：决定平滑的程度。0.3m~0.5m 通常能得到非常好的曲线。太大可能切内角撞墙。
  nav2_util::declare_parameter_if_not_declared(node, name + ".downsample_distance", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".downsample_distance", downsample_distance_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".bspline_degree", rclcpp::ParameterValue(3));
  node->get_parameter(name + ".bspline_degree", bspline_degree_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".bspline_resolution", rclcpp::ParameterValue(5));
  node->get_parameter(name + ".bspline_resolution", bspline_resolution_);

  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();
  int total_cells = size_x_ * size_y_;
  
  graph_grid_.resize(total_cells);
  unsigned char* char_map = costmap_->getCharMap();
  for(int y=0; y<size_y_; ++y) {
    for(int x=0; x<size_x_; ++x) {
      int idx = y * size_x_ + x;
      graph_grid_[idx].x = x;
      graph_grid_[idx].y = y;
      graph_grid_[idx].index = idx;
      graph_grid_[idx].cached_cost = char_map[idx];
      graph_grid_[idx].g = std::numeric_limits<double>::infinity();
      graph_grid_[idx].rhs = std::numeric_limits<double>::infinity();
      graph_grid_[idx].is_in_open = false;
      graph_grid_[idx].next_node = nullptr; 
    }
  }
  
  is_static_map_ready_ = false;
  has_last_plan_ = false; 
  
  RCLCPP_INFO(node->get_logger(), 
    "DLitePlanner (Fluid Ver): Potential: %s, TurnPenalty: %s, Downsample Dist: %.2f", 
    use_potential_field_ ? "ON" : "OFF", 
    use_turning_penalty_ ? "ON" : "OFF",
    downsample_distance_);
}

void DLitePlanner::computeStaticDistanceMap() {
  int total_cells = size_x_ * size_y_;
  static_distance_map_.assign(total_cells, -1.0f); 

  std::queue<int> q;
  unsigned char* char_map = costmap_->getCharMap();
  int obstacle_count = 0;

  for (int i = 0; i < total_cells; ++i) {
    if (char_map[i] == nav2_costmap_2d::LETHAL_OBSTACLE || 
        char_map[i] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      static_distance_map_[i] = 0.0f;
      q.push(i);
      obstacle_count++;
    }
  }

  if (obstacle_count < 10) { 
      RCLCPP_WARN(rclcpp::get_logger("DLitePlanner"), "Static map empty or few obstacles. Skipping distance field.");
      is_static_map_ready_ = false;
      return;
  }

  while (!q.empty()) {
    int idx = q.front();
    q.pop();
    
    float current_dist = static_distance_map_[idx];
    int cx = idx % size_x_;
    int cy = idx / size_x_;

    const int dx[] = {0, 0, 1, -1};
    const int dy[] = {1, -1, 0, 0};

    for (int i = 0; i < 4; ++i) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_) {
        int n_idx = ny * size_x_ + nx;
        if (static_distance_map_[n_idx] < 0) { 
          static_distance_map_[n_idx] = current_dist + 1.0f;
          q.push(n_idx);
        }
      }
    }
  }
  is_static_map_ready_ = true;
}

void DLitePlanner::cleanup() {
  open_list_.clear();
  static_distance_map_.clear();
  is_static_map_ready_ = false;
  has_last_plan_ = false;
}

void DLitePlanner::activate() {
  RCLCPP_INFO(rclcpp::get_logger("DLitePlanner"), "Activating");
}

void DLitePlanner::deactivate() {
  RCLCPP_INFO(rclcpp::get_logger("DLitePlanner"), "Deactivating");
}

inline Node* DLitePlanner::getNode(int x, int y) {
  if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) return nullptr;
  return &graph_grid_[y * size_x_ + x];
}

inline Node* DLitePlanner::getNode(int index) {
  if (index < 0 || index >= (int)graph_grid_.size()) return nullptr;
  return &graph_grid_[index];
}

double DLitePlanner::heuristic(const Node* a, const Node* b) {
  return std::hypot(a->x - b->x, a->y - b->y); 
}

std::pair<double, double> DLitePlanner::calculateKey(const Node* s) {
  double min_val = std::min(s->g, s->rhs);
  return {min_val + heuristic(start_node_ptr_, s) + km_, min_val};
}

double DLitePlanner::getExtraCost(const Node* n) {
    if (!n) return std::numeric_limits<double>::infinity();
    
    unsigned char cost = costmap_->getCost(n->x, n->y);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return std::numeric_limits<double>::infinity();

    if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_)
      return std::numeric_limits<double>::infinity();

    double extra = 0.0;

    // 势力场代价，让路径远离墙壁
    if (use_potential_field_) {
        if (is_static_map_ready_ && !static_distance_map_.empty()) {
            float dist_to_wall = static_distance_map_[n->index];
            if (dist_to_wall >= 0) {
                // 修改了势场衰减函数，使其更平缓，有助于生成流线型路径
                extra += obstacle_weight_ * std::exp(-0.2 * dist_to_wall);
            }
        }
    } 
    
    // 叠加地图膨胀代价
    if (cost > 0 && cost < 253) {
        extra += (static_cast<double>(cost) / 252.0) * obstacle_weight_;
    }

    return extra;
}

double DLitePlanner::getCost(const Node* a, const Node* b) {
  double extra = getExtraCost(b);
  if (extra == std::numeric_limits<double>::infinity()) return extra;
  double traverse_dist = (a->x != b->x && a->y != b->y) ? 1.4142 : 1.0;
  return traverse_dist + extra;
}

void DLitePlanner::updateVertex(Node* u) {
  if (u->is_in_open) {
    auto it = open_list_.find(u);
    if (it != open_list_.end()) {
      open_list_.erase(it);
    }
    u->is_in_open = false;
  }

  if (u != goal_node_ptr_) {
    double min_rhs = std::numeric_limits<double>::infinity();
    Node* best_succ = nullptr; 

    for (int k = 0; k < 8; ++k) {
        Node* succ = getNode(u->x + DX_LIST[k], u->y + DY_LIST[k]);
        if (succ) {
            double c = getCost(u, succ);
            double turn_cost = 0.0;
            
            // 转向惩罚，惩罚剧烈转弯，有利于生成平滑路线
            if (use_turning_penalty_ && succ->next_node) {
                 int vec1_x = succ->x - u->x;
                 int vec1_y = succ->y - u->y;
                 int vec2_x = succ->next_node->x - succ->x;
                 int vec2_y = succ->next_node->y - succ->y;
                 double dot = vec1_x * vec2_x + vec1_y * vec2_y;
                 double norm1 = std::hypot(vec1_x, vec1_y);
                 double norm2 = std::hypot(vec2_x, vec2_y);
                 if (norm1 > 1e-3 && norm2 > 1e-3) {
                    double cos_theta = dot / (norm1 * norm2);
                    turn_cost = turning_penalty_weight_ * (1.0 - cos_theta);
                 }
            }

            if (c != std::numeric_limits<double>::infinity()) {
                double new_rhs = succ->g + c + turn_cost;
                if (new_rhs < min_rhs) {
                    min_rhs = new_rhs;
                    best_succ = succ;
                }
            }
        }
    }

    u->rhs = min_rhs;
    u->next_node = best_succ; 
  }

  if (u->rhs != u->g) {
    u->key = calculateKey(u);
    open_list_.insert(u);
    u->is_in_open = true;
  }
}

void DLitePlanner::computeShortestPath() {
  int k = 0;
  int max_k = 4000000; 

  while (!open_list_.empty() && k++ < max_k) {
    Node* u = *open_list_.begin();
    std::pair<double, double> u_old_key = u->key;
    std::pair<double, double> start_key = calculateKey(start_node_ptr_);

    if (u->key >= start_key && start_node_ptr_->rhs == start_node_ptr_->g) {
      break;
    }

    open_list_.erase(open_list_.begin());
    u->is_in_open = false;

    std::pair<double, double> u_new_key = calculateKey(u);

    if (u_old_key < u_new_key) {
      u->key = u_new_key;
      open_list_.insert(u);
      u->is_in_open = true;
    } 
    else if (u->g > u->rhs) {
      u->g = u->rhs;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          Node* pred = getNode(u->x + dx, u->y + dy);
          if (pred) updateVertex(pred);
        }
      }
    } 
    else {
      u->g = std::numeric_limits<double>::infinity();
      updateVertex(u);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          Node* pred = getNode(u->x + dx, u->y + dy);
          if (pred) updateVertex(pred);
        }
      }
    }
  }
}

bool DLitePlanner::scanMapChanges() {
  std::vector<Node*> nodes_to_update;
  nodes_to_update.reserve(1000); 

  unsigned char* char_map = costmap_->getCharMap();
  bool has_changes = false;

  for (auto& node : graph_grid_) {
    unsigned char current_cost = char_map[node.index];
    if (node.cached_cost != current_cost) {
      node.cached_cost = current_cost;
      nodes_to_update.push_back(&node);
      has_changes = true; 
    }
  }

  if (!nodes_to_update.empty()) {
    for (Node* u : nodes_to_update) {
      updateVertex(u);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          Node* neighbor = getNode(u->x + dx, u->y + dy);
          if (neighbor) updateVertex(neighbor);
        }
      }
    }
  }
  return has_changes;
}

bool DLitePlanner::findNearestFreePoint(int& mx, int& my) {
  unsigned char* char_map = costmap_->getCharMap();
  int index = my * size_x_ + mx;
  
  if (char_map[index] != nav2_costmap_2d::LETHAL_OBSTACLE && 
      char_map[index] != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (allow_unknown_ || char_map[index] != nav2_costmap_2d::NO_INFORMATION) {
          return true;
      }
  }

  int search_radius = static_cast<int>(tolerance_ / costmap_->getResolution());
  
  for (int r = 1; r <= search_radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        int nx = mx + dx;
        int ny = my + dy;
        
        if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_) {
          int n_idx = ny * size_x_ + nx;
          unsigned char c = char_map[n_idx];
          bool is_lethal = (c == nav2_costmap_2d::LETHAL_OBSTACLE || c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
          if (!is_lethal) {
              if (allow_unknown_ || c != nav2_costmap_2d::NO_INFORMATION) {
                  mx = nx; my = ny; return true;
              }
          }
        }
      }
    }
  }
  return false;
}

// ============================================================================
// [路径后处理优化] 流线型 B 样条平滑 (图2效果的核心)
// ============================================================================

std::vector<geometry_msgs::msg::PoseStamped> DLitePlanner::smartDownsample(
    const std::vector<geometry_msgs::msg::PoseStamped>& path, double min_dist)
{
    if (path.size() < 3) return path;

    std::vector<geometry_msgs::msg::PoseStamped> downsampled;
    downsampled.push_back(path.front());

    double accumulated_dist = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        double dx = path[i].pose.position.x - path[i - 1].pose.position.x;
        double dy = path[i].pose.position.y - path[i - 1].pose.position.y;
        accumulated_dist += std::hypot(dx, dy);

        // 每隔一段距离，或者当路径发生较大转折时保留点
        if (accumulated_dist >= min_dist) {
            downsampled.push_back(path[i]);
            accumulated_dist = 0.0;
        }
    }
    
    // 确保终点总是被包含
    double dx = path.back().pose.position.x - downsampled.back().pose.position.x;
    double dy = path.back().pose.position.y - downsampled.back().pose.position.y;
    if (std::hypot(dx, dy) > 0.05) {
        downsampled.push_back(path.back());
    } else {
        downsampled.back() = path.back(); 
    }

    return downsampled;
}

std::vector<geometry_msgs::msg::PoseStamped> DLitePlanner::applyBSplineSmoothing(
    const std::vector<geometry_msgs::msg::PoseStamped>& path, int degree, int resolution)
{
    // 如果点数太少，直接返回（B样条要求 控制点数 > 阶数）
    if (path.size() <= static_cast<size_t>(degree)) return path;

    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
    smoothed_path.push_back(path.front());

    int n = path.size() - 1; // 控制点最高索引
    int m = n + degree + 1;  // 节点向量大小

    // 生成均匀节点向量 (Clamped B-Spline, 保证首尾端点相连)
    std::vector<double> knots(m + 1, 0.0);
    for (int j = 0; j <= m; ++j) {
        if (j <= degree) {
            knots[j] = 0.0;
        } else if (j >= m - degree) {
            knots[j] = 1.0;
        } else {
            knots[j] = static_cast<double>(j - degree) / (n - degree + 1);
        }
    }

    // 在曲线参数 t 上进行采样
    int total_samples = path.size() * resolution;
    for (int s = 1; s <= total_samples; ++s) {
        double t = static_cast<double>(s) / total_samples;
        
        // 保证 t 不会超出边界 (数值精度问题)
        if (t > 1.0) t = 1.0;
        if (t == 1.0 && s < total_samples) continue; // 处理最后一个点避免重复

        double smoothed_x = 0.0;
        double smoothed_y = 0.0;

        // 计算基函数 N_{i, degree}(t) 并累加控制点
        for (int i = 0; i <= n; ++i) {
            // De Boor-Cox 递归计算基函数
            std::vector<double> N(degree + 1, 0.0);
            for (int j = 0; j <= degree; ++j) {
                if (t >= knots[i + j] && t < knots[i + j + 1]) {
                    N[j] = 1.0;
                } else if (t == 1.0 && knots[i + j + 1] == 1.0 && knots[i + j] != 1.0) { // 处理右边界
                     N[j] = 1.0;
                }
            }

            for (int d = 1; d <= degree; ++d) {
                for (int j = 0; j <= degree - d; ++j) {
                    double left_term = 0.0, right_term = 0.0;
                    double denom1 = knots[i + j + d] - knots[i + j];
                    double denom2 = knots[i + j + d + 1] - knots[i + j + 1];

                    if (denom1 != 0.0) {
                        left_term = ((t - knots[i + j]) / denom1) * N[j];
                    }
                    if (denom2 != 0.0) {
                        right_term = ((knots[i + j + d + 1] - t) / denom2) * N[j + 1];
                    }
                    N[j] = left_term + right_term;
                }
            }
            
            double basis_val = N[0];
            smoothed_x += path[i].pose.position.x * basis_val;
            smoothed_y += path[i].pose.position.y * basis_val;
        }

        geometry_msgs::msg::PoseStamped new_pose = path.front(); // 拷贝 header
        new_pose.pose.position.x = smoothed_x;
        new_pose.pose.position.y = smoothed_y;
        
        // 计算航向角
        const auto& last_pose = smoothed_path.back().pose;
        double dx = smoothed_x - last_pose.position.x;
        double dy = smoothed_y - last_pose.position.y;
        if (std::hypot(dx, dy) > 1e-4) {
            double yaw = std::atan2(dy, dx);
            new_pose.pose.orientation.z = std::sin(yaw / 2.0);
            new_pose.pose.orientation.w = std::cos(yaw / 2.0);
        } else {
            new_pose.pose.orientation = last_pose.orientation;
        }

        smoothed_path.push_back(new_pose);
    }
    
    // 强制修正最后一个点的姿态与目标一致
    smoothed_path.back().pose.orientation = path.back().pose.orientation;

    return smoothed_path;
}

// ============================================================================
// [主流水线装配]
// ============================================================================

nav_msgs::msg::Path DLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_.lock()->now(); 
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  unsigned int cx = costmap_->getSizeInCellsX();
  unsigned int cy = costmap_->getSizeInCellsY();
  
  if (cx != (unsigned int)size_x_ || cy != (unsigned int)size_y_) {
    size_x_ = cx; size_y_ = cy;
    graph_grid_.clear();
    graph_grid_.resize(size_x_ * size_y_);
    unsigned char* char_map = costmap_->getCharMap();
    for(int i=0; i < size_x_ * size_y_; ++i) {
        graph_grid_[i].x = i % size_x_;
        graph_grid_[i].y = i / size_x_;
        graph_grid_[i].index = i;
        graph_grid_[i].cached_cost = char_map[i];
        graph_grid_[i].g = std::numeric_limits<double>::infinity();
        graph_grid_[i].rhs = std::numeric_limits<double>::infinity();
        graph_grid_[i].next_node = nullptr; 
    }
    open_list_.clear();
    km_ = 0;
    start_node_ptr_ = nullptr;
    goal_node_ptr_ = nullptr;
    is_static_map_ready_ = false;
    has_last_plan_ = false; 
  }
  
  if (use_potential_field_ && !is_static_map_ready_) {
      computeStaticDistanceMap();
  }

  bool map_changed = scanMapChanges();

  int mx_start, my_start, mx_goal, my_goal;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    return path;
  }

  if (!findNearestFreePoint(mx_start, my_start) || !findNearestFreePoint(mx_goal, my_goal)) {
    RCLCPP_WARN(rclcpp::get_logger("DLitePlanner"), "Points blocked.");
    return path;
  }

  bool goal_changed = false;
  if (goal_node_ptr_ == nullptr || mx_goal != goal_node_ptr_->x || my_goal != goal_node_ptr_->y) {
    goal_changed = true;
  }

  if (goal_changed) {
    open_list_.clear();
    km_ = 0;
    for (auto& n : graph_grid_) {
        n.g = std::numeric_limits<double>::infinity();
        n.rhs = std::numeric_limits<double>::infinity();
        n.is_in_open = false;
        n.next_node = nullptr; 
    }
    start_node_ptr_ = getNode(mx_start, my_start);
    goal_node_ptr_ = getNode(mx_goal, my_goal);
    last_start_node_ptr_ = start_node_ptr_;
    
    goal_node_ptr_->rhs = 0;
    goal_node_ptr_->key = calculateKey(goal_node_ptr_);
    open_list_.insert(goal_node_ptr_);
    goal_node_ptr_->is_in_open = true;
  }
  else {
    start_node_ptr_ = getNode(mx_start, my_start);
    double dist = heuristic(last_start_node_ptr_, start_node_ptr_);
    km_ += dist;
    last_start_node_ptr_ = start_node_ptr_;
  }

  // 1. 运行核心 D* Lite 搜索 (此时带势力场，会自动远离墙壁)
  computeShortestPath();

  if (start_node_ptr_->g == std::numeric_limits<double>::infinity()) {
    open_list_.clear();
    km_ = 0;
    for (auto& n : graph_grid_) {
        n.g = std::numeric_limits<double>::infinity();
        n.rhs = std::numeric_limits<double>::infinity();
        n.is_in_open = false;
        n.next_node = nullptr; 
    }
    goal_node_ptr_->rhs = 0;
    goal_node_ptr_->key = calculateKey(goal_node_ptr_);
    open_list_.insert(goal_node_ptr_);
    goal_node_ptr_->is_in_open = true;
    computeShortestPath();
  }

  if (start_node_ptr_->g == std::numeric_limits<double>::infinity()) {
    return path; 
  }

  // 2. 提取基础离散路径 (由于有势场，这条折线已经在走廊中央了)
  std::vector<geometry_msgs::msg::PoseStamped> discrete_path;
  Node* current = start_node_ptr_;
  geometry_msgs::msg::PoseStamped start_pose = start;
  start_pose.header = path.header;
  mapToWorld(mx_start, my_start, start_pose.pose.position.x, start_pose.pose.position.y);
  discrete_path.push_back(start_pose);
  
  int max_steps = 20000;
  
  while (current != goal_node_ptr_ && max_steps-- > 0) {
    Node* best_next = nullptr;
    double min_score = std::numeric_limits<double>::infinity();

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        Node* neighbor = getNode(current->x + dx, current->y + dy);
        if (!neighbor) continue;

        double edge_cost = getCost(current, neighbor);
        if (edge_cost == std::numeric_limits<double>::infinity()) continue;

        double current_score = edge_cost + neighbor->g;

        if (current_score < min_score) {
          min_score = current_score;
          best_next = neighbor;
        }
      }
    }
    if (!best_next || min_score == std::numeric_limits<double>::infinity()) break; 

    current = best_next;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    mapToWorld(current->x, current->y, pose.pose.position.x, pose.pose.position.y);
    discrete_path.push_back(pose);
  }
  
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  mapToWorld(mx_goal, my_goal, goal_pose.pose.position.x, goal_pose.pose.position.y);
  discrete_path.push_back(goal_pose);

  // 3. 核心流线型处理：均匀抽稀 -> B样条拟合
  // 不再使用生硬的 LOS 拉直线，而是将势场推开的折线转化为圆滑的曲线
  std::vector<geometry_msgs::msg::PoseStamped> downsampled_path = smartDownsample(discrete_path, downsample_distance_);
  std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = applyBSplineSmoothing(downsampled_path, bspline_degree_, bspline_resolution_);

  path.poses = smoothed_path;

  if (!path.poses.empty()) {
      last_plan_ = path;
      last_goal_ = goal;
      has_last_plan_ = true;
  }

  return path;
}

bool DLitePlanner::worldToMap(double wx, double wy, int& mx, int& my) {
  unsigned int umx, umy;
  if (costmap_->worldToMap(wx, wy, umx, umy)) {
    mx = static_cast<int>(umx);
    my = static_cast<int>(umy);
    return true;
  }
  return false;
}

void DLitePlanner::mapToWorld(int mx, int my, double& wx, double& wy) {
  costmap_->mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
}

void DLitePlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    double res = costmap_->getResolution();
    wx = origin_x + (mx + 0.5) * res;
    wy = origin_y + (my + 0.5) * res;
}

}  // namespace fishbot_dlite_planner