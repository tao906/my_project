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

// ============================================================================
// [全局常量]
// ============================================================================
const int DX_LIST[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int DY_LIST[8] = {0, 1, 1, 1, 0, -1, -1, -1};
const int PAIRS[8][2] = {
    {0, 1}, {2, 1}, {2, 3}, {4, 3},
    {4, 5}, {6, 5}, {6, 7}, {0, 7}
};

// ============================================================================
// [辅助函数]
// ============================================================================

double get_dist(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2) {
    return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

nav_msgs::msg::Path downsamplePath(const nav_msgs::msg::Path& input_path, double min_dist = 0.5) {
    if (input_path.poses.size() < 3) return input_path;
    nav_msgs::msg::Path sparse_path;
    sparse_path.header = input_path.header;
    sparse_path.poses.push_back(input_path.poses.front());
    for (size_t i = 1; i < input_path.poses.size() - 1; ++i) {
        const auto& last_p = sparse_path.poses.back();
        const auto& curr_p = input_path.poses[i];
        if (get_dist(last_p, curr_p) >= min_dist) {
            sparse_path.poses.push_back(curr_p);
        }
    }
    sparse_path.poses.push_back(input_path.poses.back());
    return sparse_path;
}

nav_msgs::msg::Path smoothPathBSpline(const nav_msgs::msg::Path& input_path, double resolution = 0.05) {
    if (input_path.poses.size() < 4) return input_path; 
    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = input_path.header;
    
    const auto& points = input_path.poses;
    smoothed_path.poses.push_back(points.front());

    for (size_t i = 0; i < points.size() - 3; ++i) {
        auto p0 = points[i];
        auto p1 = points[i+1];
        auto p2 = points[i+2];
        auto p3 = points[i+3];
        
        double seg_dist = get_dist(p1, p2);
        if (seg_dist < 0.001) continue; 
        
        int num_samples = std::max(2, static_cast<int>(seg_dist / resolution));
        
        for (int j = 0; j < num_samples; ++j) {
            double t = (double)j / num_samples;
            double t2 = t * t;
            double t3 = t2 * t;
            
            double b0 = (1.0 - 3.0*t + 3.0*t2 - t3) / 6.0;
            double b1 = (4.0 - 6.0*t2 + 3.0*t3) / 6.0;
            double b2 = (1.0 + 3.0*t + 3.0*t2 - 3.0*t3) / 6.0;
            double b3 = t3 / 6.0;
            
            geometry_msgs::msg::PoseStamped p_new;
            p_new.header = input_path.header;
            
            p_new.pose.position.x = b0*p0.pose.position.x + b1*p1.pose.position.x + b2*p2.pose.position.x + b3*p3.pose.position.x;
            p_new.pose.position.y = b0*p0.pose.position.y + b1*p1.pose.position.y + b2*p2.pose.position.y + b3*p3.pose.position.y;
            p_new.pose.position.z = 0.0;
            
            const auto& last_pose = smoothed_path.poses.back().pose;
            double dx = p_new.pose.position.x - last_pose.position.x;
            double dy = p_new.pose.position.y - last_pose.position.y;
            
            if (std::hypot(dx, dy) > 1e-4) {
                double yaw = std::atan2(dy, dx);
                p_new.pose.orientation.z = std::sin(yaw / 2.0);
                p_new.pose.orientation.w = std::cos(yaw / 2.0);
            } else {
                p_new.pose.orientation = last_pose.orientation;
            }

            smoothed_path.poses.push_back(p_new);
        }
    }
    
    if (points.size() >= 2) {
         auto p_penultimate = points[points.size()-2];
         const auto& last_smoothed = smoothed_path.poses.back().pose;
         double dx = p_penultimate.pose.position.x - last_smoothed.position.x;
         double dy = p_penultimate.pose.position.y - last_smoothed.position.y;
         
         if(std::hypot(dx, dy) > 1e-4) {
             double yaw = std::atan2(dy, dx);
             p_penultimate.pose.orientation.z = std::sin(yaw / 2.0);
             p_penultimate.pose.orientation.w = std::cos(yaw / 2.0);
         } else {
             p_penultimate.pose.orientation = last_smoothed.orientation;
         }
         smoothed_path.poses.push_back(p_penultimate);
         
         smoothed_path.poses.push_back(points.back());
    }
    
    return smoothed_path;
}

// ============================================================================
// [DLitePlanner 类实现]
// ============================================================================

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
  
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".obstacle_weight", obstacle_weight_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_potential_field", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_potential_field", use_potential_field_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_bspline_smoothing", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_bspline_smoothing", use_bspline_smoothing_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_turning_penalty", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_turning_penalty", use_turning_penalty_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".turning_penalty_weight", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".turning_penalty_weight", turning_penalty_weight_);

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_field_dstar_interpolation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_field_dstar_interpolation", use_field_dstar_interpolation_);

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
    "DLitePlanner: Configured. Potential: %s, Smooth: %s, TurnPenalty: %s, FieldInterp: %s", 
    use_potential_field_ ? "ON" : "OFF", 
    use_bspline_smoothing_ ? "ON" : "OFF",
    use_turning_penalty_ ? "ON" : "OFF",
    use_field_dstar_interpolation_ ? "ON" : "OFF");
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

// ============================================================================
// [核心修改] getExtraCost: 现在同时考虑 势场 和 地图膨胀数值
// ============================================================================
double DLitePlanner::getExtraCost(const Node* n) {
    if (!n) return std::numeric_limits<double>::infinity();
    
    unsigned char cost = costmap_->getCost(n->x, n->y);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return std::numeric_limits<double>::infinity();

    if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_)
      return std::numeric_limits<double>::infinity();

    double extra = 0.0;

    // 1. 势场代价 (如果开启且准备好)
    if (use_potential_field_) {
        if (is_static_map_ready_ && !static_distance_map_.empty()) {
            float dist_to_wall = static_distance_map_[n->index];
            if (dist_to_wall >= 0) {
                // 距离越近，代价越高 (反比关系)
                extra += obstacle_weight_ / (dist_to_wall * 1.0 + 1.0);
            }
        }
    } 
    
    // 2. 叠加地图膨胀代价 (响应用户需求："也考虑"膨胀数值)
    // 无论势场是否开启，如果 Costmap 处有非致命的膨胀值 (1-252)，都应该计入代价。
    // 这能让机器人在势场失效或未覆盖区域（如新出现的动态障碍物）依然保持避障能力。
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

    if (use_field_dstar_interpolation_) {
        for (int k = 0; k < 8; ++k) {
            int idx_c = PAIRS[k][0]; 
            int idx_d = PAIRS[k][1]; 

            Node* n_c = getNode(u->x + DX_LIST[idx_c], u->y + DY_LIST[idx_c]);
            Node* n_d = getNode(u->x + DX_LIST[idx_d], u->y + DY_LIST[idx_d]);

            if (!n_c || !n_d) continue;

            double extra_c = getExtraCost(n_c);
            double extra_d = getExtraCost(n_d);

            bool c_blocked = (extra_c == std::numeric_limits<double>::infinity());
            bool d_blocked = (extra_d == std::numeric_limits<double>::infinity());
            
            if (c_blocked && d_blocked) continue;

            double g_c = c_blocked ? std::numeric_limits<double>::infinity() : n_c->g;
            double g_d = d_blocked ? std::numeric_limits<double>::infinity() : n_d->g;

            if (g_c == std::numeric_limits<double>::infinity() && 
                g_d == std::numeric_limits<double>::infinity()) continue;

            double dist_c = 1.0; 
            double y_opt = 0.0; 
            
            if (g_d == std::numeric_limits<double>::infinity()) {
                y_opt = 0.0; 
            } else if (g_c == std::numeric_limits<double>::infinity()) {
                y_opt = 1.0; 
            } else {
                double k_diff = g_c - g_d;
                if (dist_c > std::abs(k_diff)) {
                    y_opt = k_diff / std::sqrt(dist_c*dist_c - k_diff*k_diff);
                } else {
                    y_opt = (k_diff > 0) ? 1.0 : 0.0; 
                }
            }
            y_opt = std::max(0.0, std::min(1.0, y_opt));

            double interp_g;
            if (g_d == std::numeric_limits<double>::infinity()) interp_g = g_c;
            else if (g_c == std::numeric_limits<double>::infinity()) interp_g = g_d;
            else interp_g = (1.0 - y_opt) * g_c + y_opt * g_d;

            double traverse_dist = std::sqrt(1.0 + y_opt * y_opt); 
            
            double current_extra;
            if (d_blocked) current_extra = extra_c;
            else if (c_blocked) current_extra = extra_d;
            else current_extra = std::max(extra_c, extra_d); 

            double basic_cost = interp_g + traverse_dist + current_extra;

            double turn_cost = 0.0;
            if (use_turning_penalty_) {
                double rel_cx = DX_LIST[idx_c], rel_cy = DY_LIST[idx_c];
                double rel_dx = DX_LIST[idx_d], rel_dy = DY_LIST[idx_d];
                double vec1_x = (1.0 - y_opt) * rel_cx + y_opt * rel_dx;
                double vec1_y = (1.0 - y_opt) * rel_cy + y_opt * rel_dy;

                Node* next_step_node = (y_opt > 0.5) ? n_d : n_c;
                if (next_step_node && next_step_node->next_node) {
                    double vec2_x = next_step_node->next_node->x - next_step_node->x;
                    double vec2_y = next_step_node->next_node->y - next_step_node->y;

                    double dot = vec1_x * vec2_x + vec1_y * vec2_y;
                    double norm1 = std::hypot(vec1_x, vec1_y);
                    double norm2 = std::hypot(vec2_x, vec2_y);

                    if (norm1 > 1e-3 && norm2 > 1e-3) {
                        double cos_theta = dot / (norm1 * norm2);
                        turn_cost = turning_penalty_weight_ * (1.0 - cos_theta);
                    }
                }
            }

            double total_rhs = basic_cost + turn_cost;
            if (total_rhs < min_rhs) {
                min_rhs = total_rhs;
                best_succ = (y_opt > 0.5) ? n_d : n_c;
            }
        }
    } 
    else {
        for (int k = 0; k < 8; ++k) {
            Node* succ = getNode(u->x + DX_LIST[k], u->y + DY_LIST[k]);
            if (succ) {
                double c = getCost(u, succ);
                double turn_cost = 0.0;
                
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
  
  if (k >= max_k) {
    RCLCPP_WARN(rclcpp::get_logger("DLitePlanner"), "Steps limit reached.");
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

nav_msgs::msg::Path DLitePlanner::extractPathFieldDStar(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = node_.lock()->now();
    
    auto start_time = std::chrono::steady_clock::now();
    double max_duration_ms = 200.0; 

    int mx_s, my_s, mx_g, my_g;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, mx_s, my_s) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, mx_g, my_g)) {
        return path;
    }
    
    double curr_x = static_cast<double>(mx_s);
    double curr_y = static_cast<double>(my_s);
    
    geometry_msgs::msg::PoseStamped start_p = start;
    start_p.header = path.header; 
    path.poses.push_back(start_p);

    double qx = start.pose.orientation.x;
    double qy = start.pose.orientation.y;
    double qz = start.pose.orientation.z;
    double qw = start.pose.orientation.w;
    double last_theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    int max_steps = 5000;
    int step = 0;
    
    while (step++ < max_steps) {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(current_time - start_time).count();
        if (elapsed_ms > max_duration_ms) {
            RCLCPP_WARN(node_.lock()->get_logger(), "Field D* extraction timed out! Returning partial path.");
            break; 
        }

        double dist_to_goal = std::hypot(curr_x - mx_g, curr_y - my_g);
        if (dist_to_goal < 1.0) break;
        
        int cx = static_cast<int>(std::round(curr_x));
        int cy = static_cast<int>(std::round(curr_y));
        
        double vals[8];
        bool valid_mask[8];
        int neighbor_indices[8]; 
        
        for (int k = 0; k < 8; ++k) {
            int nx = cx + DX_LIST[k];
            int ny = cy + DY_LIST[k];
            
            vals[k] = std::numeric_limits<double>::infinity();
            valid_mask[k] = false;
            neighbor_indices[k] = -1;
            
            if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_) {
                int n_idx = ny * size_x_ + nx;
                Node* n_ptr = &graph_grid_[n_idx];
                
                if (n_ptr->g != std::numeric_limits<double>::infinity()) {
                    vals[k] = n_ptr->g;
                    valid_mask[k] = true;
                    neighbor_indices[k] = n_idx;
                }
            }
        }
        
        double best_cost = std::numeric_limits<double>::infinity();
        double next_x = curr_x;
        double next_y = curr_y;
        bool found_gradient = false;
        double best_step_theta = last_theta; 

        for (int k = 0; k < 8; ++k) {
            int idx_c = PAIRS[k][0]; 
            int idx_d = PAIRS[k][1]; 
            
            if (!valid_mask[idx_c]) continue; 
            double g_c = vals[idx_c];
            double g_d = vals[idx_d]; 
            
            if (g_c == std::numeric_limits<double>::infinity() && 
                g_d == std::numeric_limits<double>::infinity()) continue;
                
            double dist_c = 1.0; 
            double y_opt = 0.0;
            double cost_interp = std::numeric_limits<double>::infinity();
            
            if (g_d == std::numeric_limits<double>::infinity()) {
                y_opt = 0.0;
                cost_interp = dist_c + g_c;
            } else if (g_c == std::numeric_limits<double>::infinity()) {
                y_opt = 1.0;
                cost_interp = std::sqrt(2.0) * dist_c + g_d;
            } else {
                double k_diff = g_c - g_d;
                if (dist_c > std::abs(k_diff)) {
                    y_opt = k_diff / std::sqrt(dist_c*dist_c - k_diff*k_diff);
                } else {
                    y_opt = (k_diff > 0) ? 1.0 : 0.0; 
                }
                
                if (y_opt < 0.0) y_opt = 0.0;
                if (y_opt > 1.0) y_opt = 1.0;
                
                cost_interp = std::sqrt(1.0 + y_opt*y_opt) * dist_c + (1.0 - y_opt)*g_c + y_opt*g_d;
            }
            
            int n_c_idx = neighbor_indices[idx_c];
            int n_d_idx = neighbor_indices[idx_d];
            double r_c = static_cast<double>(graph_grid_[n_c_idx].y); 
            double c_c = static_cast<double>(graph_grid_[n_c_idx].x);
            double r_d = r_c; 
            double c_d = c_c;
            if (n_d_idx != -1) {
                r_d = static_cast<double>(graph_grid_[n_d_idx].y);
                c_d = static_cast<double>(graph_grid_[n_d_idx].x);
            } else if (y_opt > 0.001) {
                y_opt = 0.0;
                cost_interp = dist_c + g_c;
            }

            double cand_x = (1.0 - y_opt) * c_c + y_opt * c_d;
            double cand_y = (1.0 - y_opt) * r_c + y_opt * r_d;
            
            if (cost_interp < best_cost) {
                best_cost = cost_interp;
                next_x = cand_x;
                next_y = cand_y;
                found_gradient = true;
                if (std::hypot(next_x - curr_x, next_y - curr_y) > 1e-3) {
                    best_step_theta = std::atan2(next_y - curr_y, next_x - curr_x);
                }
            }
        }
        
        if (found_gradient) {
            double move_dist = std::hypot(next_x - curr_x, next_y - curr_y);
            if (move_dist < 0.05) { 
                found_gradient = false; 
            } else {
                curr_x = next_x;
                curr_y = next_y;
                last_theta = best_step_theta;
            }
        }
        
        if (!found_gradient) {
            double min_g = std::numeric_limits<double>::infinity();
            int best_nbr_idx = -1;
            for (int k=0; k<8; ++k) {
                if (valid_mask[k] && vals[k] < min_g) {
                    min_g = vals[k];
                    best_nbr_idx = neighbor_indices[k];
                }
            }
            if (best_nbr_idx != -1) {
                curr_x = static_cast<double>(graph_grid_[best_nbr_idx].x);
                curr_y = static_cast<double>(graph_grid_[best_nbr_idx].y);
            } else {
                break; 
            }
        }
        
        geometry_msgs::msg::PoseStamped p_next;
        p_next.header = path.header; 
        double wx, wy;
        mapToWorld(curr_x, curr_y, wx, wy);
        p_next.pose.position.x = wx;
        p_next.pose.position.y = wy;
        p_next.pose.position.z = 0.0;
        
        if (!path.poses.empty()) {
            const auto& last_pose = path.poses.back().pose;
            double dx = wx - last_pose.position.x;
            double dy = wy - last_pose.position.y;
            
            if (std::hypot(dx, dy) > 1e-3) {
                double yaw = std::atan2(dy, dx);
                p_next.pose.orientation.z = std::sin(yaw / 2.0);
                p_next.pose.orientation.w = std::cos(yaw / 2.0);
            } else {
                p_next.pose.orientation = last_pose.orientation;
            }
        } else {
             p_next.pose.orientation.w = 1.0; 
        }

        path.poses.push_back(p_next);
    }
    
    geometry_msgs::msg::PoseStamped goal_p = goal;
    goal_p.header = path.header;
    path.poses.push_back(goal_p);
    
    return path;
}


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

  nav_msgs::msg::Path final_path;
  
  if (use_field_dstar_interpolation_) {
      path = extractPathFieldDStar(start, goal);
  } else {
      Node* current = start_node_ptr_;
      geometry_msgs::msg::PoseStamped start_pose = start;
      start_pose.header = path.header;
      mapToWorld(mx_start, my_start, start_pose.pose.position.x, start_pose.pose.position.y);
      path.poses.push_back(start_pose);
      
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
        
        if (!path.poses.empty()) {
             const auto& last_pose = path.poses.back().pose;
             double dx = pose.pose.position.x - last_pose.position.x;
             double dy = pose.pose.position.y - last_pose.position.y;
             if (std::hypot(dx, dy) > 1e-3) {
                 double yaw = std::atan2(dy, dx);
                 pose.pose.orientation.z = std::sin(yaw / 2.0);
                 pose.pose.orientation.w = std::cos(yaw / 2.0);
             } else {
                 pose.pose.orientation = last_pose.orientation;
             }
        } else {
            pose.pose.orientation.w = 1.0;
        }

        path.poses.push_back(pose);
      }
      geometry_msgs::msg::PoseStamped goal_pose = goal;
      goal_pose.header = path.header;
      mapToWorld(mx_goal, my_goal, goal_pose.pose.position.x, goal_pose.pose.position.y);
      path.poses.push_back(goal_pose);
  }

  final_path = path;
  if (use_bspline_smoothing_ && path.poses.size() > 5) {
      nav_msgs::msg::Path sparse_path = downsamplePath(path, 0.5); 
      final_path = smoothPathBSpline(sparse_path, 0.05);
  }

  if (!final_path.poses.empty()) {
      last_plan_ = final_path;
      last_goal_ = goal;
      has_last_plan_ = true;
  }

  return final_path;
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