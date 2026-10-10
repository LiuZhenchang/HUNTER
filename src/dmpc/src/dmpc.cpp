#include "dmpc/dmpc.h"
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include "dmpc/role_reward_policy.h"

namespace EA {
std::mutex mtx_rand;
}

std::vector<std::vector<double>> parseNoFlyZonesString(const std::string &zones_str) {
  std::vector<std::vector<double>> result;
  if (zones_str.empty()) {
    return result;
  }

  std::vector<double> current_zone;
  std::string num_str;

  auto processNumber = [&]() {
    if (!num_str.empty()) {
      try {
        current_zone.push_back(std::stod(num_str));
      } catch (...) {
      }
      num_str.clear();
    }
  };

  for (char c : zones_str) {
    if (c == ';') {
      processNumber();
      if (!current_zone.empty()) {
        result.push_back(current_zone);
        current_zone.clear();
      }
    } else if (c == ',') {
      processNumber();
    } else if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
      num_str += c;
    }
  }
  processNumber();
  if (!current_zone.empty()) {
    result.push_back(current_zone);
  }

  return result;
}

DMPC::DMPC() {
  current_realtime_position_ = Eigen::Vector3d::Zero();
  current_realtime_yaw_ = 0.0;
}

DMPC::~DMPC() {}

rclcpp::Logger DMPC::getLogger() const {
  if (node_) {
    return node_->get_logger();
  }
  return rclcpp::get_logger("dmpc");
}

void DMPC::initDmpc(rclcpp::Node::SharedPtr node) {
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("dmpc"), "DMPC::initDmpc: Node pointer is null");
    return;
  }
  node_ = node;

  // GA parameters must be read during the initialization path used by SearchManager.
  // swarm.launch.py forwards these values through single.launch.py as dmpc/* parameters.
  if (!node_->has_parameter("dmpc/population_size")) {
    node_->declare_parameter<int>("dmpc/population_size", 60);
  }
  if (!node_->has_parameter("dmpc/generation_max")) {
    node_->declare_parameter<int>("dmpc/generation_max", 20);
  }
  if (!node_->has_parameter("dmpc/crossover_fraction")) {
    node_->declare_parameter<double>("dmpc/crossover_fraction", 0.8);
  }
  if (!node_->has_parameter("dmpc/mutation_rate")) {
    node_->declare_parameter<double>("dmpc/mutation_rate", 0.2);
  }
  if (!node_->has_parameter("dmpc/best_stall_max")) {
    node_->declare_parameter<int>("dmpc/best_stall_max", 5);
  }
  if (!node_->has_parameter("dmpc/elite_count")) {
    node_->declare_parameter<int>("dmpc/elite_count", 2);
  }
  if (!node_->has_parameter("dmpc/max_steps")) {
    node_->declare_parameter<int>("dmpc/max_steps", 9);
  }
  ga_params_.population_size = node_->get_parameter("dmpc/population_size").as_int();
  ga_params_.generation_max = node_->get_parameter("dmpc/generation_max").as_int();
  ga_params_.crossover_fraction = node_->get_parameter("dmpc/crossover_fraction").as_double();
  ga_params_.mutation_rate = node_->get_parameter("dmpc/mutation_rate").as_double();
  ga_params_.best_stall_max = node_->get_parameter("dmpc/best_stall_max").as_int();
  ga_params_.elite_count = node_->get_parameter("dmpc/elite_count").as_int();
  ga_params_.max_steps = node_->get_parameter("dmpc/max_steps").as_int();
  if (ga_params_.population_size <= 0 || ga_params_.generation_max <= 0 || ga_params_.best_stall_max <= 0 ||
      ga_params_.elite_count < 0 || ga_params_.max_steps <= 0 || ga_params_.crossover_fraction < 0.0 ||
      ga_params_.crossover_fraction > 1.0 || ga_params_.mutation_rate < 0.0 || ga_params_.mutation_rate > 1.0) {
    throw std::invalid_argument("Invalid GA parameters");
  }
  RCLCPP_INFO(node_->get_logger(),
              "DMPC GA parameters: population=%d, generations=%d, crossover=%.3f, mutation=%.3f, stall=%d, elite=%d, "
              "max_steps=%d",
              ga_params_.population_size, ga_params_.generation_max, ga_params_.crossover_fraction,
              ga_params_.mutation_rate, ga_params_.best_stall_max, ga_params_.elite_count, ga_params_.max_steps);

  node_->declare_parameter<double>("dmpc/safe_uav", 1.0);
  node_->declare_parameter<double>("dmpc/safe_obs", 1.0);
  node_->declare_parameter<double>("dmpc/apf_distance", 1.0);
  node_->declare_parameter<double>("dmpc/apf_param1", 1.0);
  node_->declare_parameter<double>("dmpc/apf_param2", 1.0);
  node_->declare_parameter<int>("dmpc/predict_horizon", 1);
  node_->declare_parameter<double>("dmpc/w_jA1", 1.0);
  node_->declare_parameter<double>("dmpc/w_jA2", 1.0);
  node_->declare_parameter<double>("dmpc/w_jB", 1.0);
  node_->declare_parameter<double>("dmpc/w_jC", 1.0);
  node_->declare_parameter<double>("dmpc/w_jD", 1.0);
  node_->declare_parameter<double>("dmpc/k_wJA1", 1.0);
  node_->declare_parameter<double>("dmpc/k_wJA2", 1.0);
  node_->declare_parameter<double>("dmpc/k_wJB", 0.0);
  node_->declare_parameter<double>("dmpc/k_wJC", 1.0);
  node_->declare_parameter<double>("dmpc/k_wJD", 1.0);
  node_->declare_parameter<double>("dmpc/safe_margin_x", 0.0);
  node_->declare_parameter<double>("dmpc/safe_margin_y", 0.0);
  node_->declare_parameter<bool>("dmpc/use_expert_system", true);
  node_->declare_parameter<double>("dmpc/dis_tar_warnning", 5.0);
  node_->declare_parameter<double>("dmpc/max_dt", 2.0);
  node_->declare_parameter<double>("dmpc/velocity_threshold", 2.0);
  node_->declare_parameter<double>("dmpc/search_jump", 1.0);
  node_->declare_parameter<double>("dmpc/target_switch_distance", 80.0);
  node_->declare_parameter<double>("dmpc/target_arrival_distance", 20.0);
  node_->declare_parameter<int>("task_param/path_points_per_execution", 3);
  node_->declare_parameter<std::vector<double>>("initial_uav_position", std::vector<double>());
  node_->declare_parameter<bool>("dmpc/use_initial_task_assignment", false);

  agent_param_->safe_uav_ = node_->get_parameter("dmpc/safe_uav").as_double();
  agent_param_->safe_obs_ = node_->get_parameter("dmpc/safe_obs").as_double();
  agent_param_->apf_distance_ = node_->get_parameter("dmpc/apf_distance").as_double();
  agent_param_->apf_param1_ = node_->get_parameter("dmpc/apf_param1").as_double();
  agent_param_->apf_param2_ = node_->get_parameter("dmpc/apf_param2").as_double();
  agent_param_->predict_horizon_ = node_->get_parameter("dmpc/predict_horizon").as_int();
  agent_param_->w_jA1 = node_->get_parameter("dmpc/w_jA1").as_double();
  agent_param_->w_jA2 = node_->get_parameter("dmpc/w_jA2").as_double();
  agent_param_->w_jB = node_->get_parameter("dmpc/w_jB").as_double();
  agent_param_->w_jC = node_->get_parameter("dmpc/w_jC").as_double();
  agent_param_->w_jD = node_->get_parameter("dmpc/w_jD").as_double();
  agent_param_->k_wJA1 = node_->get_parameter("dmpc/k_wJA1").as_double();
  agent_param_->k_wJA2 = node_->get_parameter("dmpc/k_wJA2").as_double();
  agent_param_->k_wJB = node_->get_parameter("dmpc/k_wJB").as_double();
  agent_param_->k_wJC = node_->get_parameter("dmpc/k_wJC").as_double();
  agent_param_->k_wJD = node_->get_parameter("dmpc/k_wJD").as_double();
  agent_param_->safe_margin_x_ = node_->get_parameter("dmpc/safe_margin_x").as_double();
  agent_param_->safe_margin_y_ = node_->get_parameter("dmpc/safe_margin_y").as_double();
  agent_param_->use_expert_system_ = node_->get_parameter("dmpc/use_expert_system").as_bool();
  agent_param_->dis_tar_warnning_ = node_->get_parameter("dmpc/dis_tar_warnning").as_double();
  agent_param_->max_dt = node_->get_parameter("dmpc/max_dt").as_double();
  agent_param_->velocity_threshold = node_->get_parameter("dmpc/velocity_threshold").as_double();
  agent_param_->search_jump = node_->get_parameter("dmpc/search_jump").as_double();
  target_switch_distance_ = node_->get_parameter("dmpc/target_switch_distance").as_double();
  target_arrival_distance_ = node_->get_parameter("dmpc/target_arrival_distance").as_double();
  path_points = node_->get_parameter("task_param/path_points_per_execution").as_int();
  use_initial_task_assignment = node_->get_parameter("dmpc/use_initial_task_assignment").as_bool();

  std::string no_fly_zones_str = node_->get_parameter("no_fly_zones").as_string();
  enable_no_fly_zone_ = node_->get_parameter("dmpc/enable_no_fly_zone").as_bool();

  v_n = node_->get_parameter("v_n").as_double();
  map_resolution = node_->get_parameter("map/resolution").as_double();
  agent_num = node_->get_parameter("agent_num").as_int();
  target_num = node_->get_parameter("target_num").as_int();
  std::vector<double> initial_agents = node_->get_parameter("initial_uav_position").as_double_array();

  std::vector<double> region_vertices_raw;
  if (!node_->has_parameter("region_vertices")) {
    region_vertices_raw = {-1000.0, -1000.0, -1000.0, 1000.0, 1000.0, 1000.0, 1000.0, -1000.0};
    node_->declare_parameter<std::vector<double>>("region_vertices", region_vertices_raw);
  }
  region_vertices_raw = node_->get_parameter("region_vertices").as_double_array();
  region_vertices_ = transformRegionVertices(region_vertices_raw);
  RCLCPP_INFO(getLogger(), "DMPC: Task region vertices loaded with %zu vertices", region_vertices_.size());

  if (enable_no_fly_zone_) {
    no_fly_zones_.clear();
    int valid_zone_count = 0;
    std::vector<std::vector<double>> no_fly_zones_raw = parseNoFlyZonesString(no_fly_zones_str);

    for (size_t i = 0; i < no_fly_zones_raw.size(); ++i) {
      auto zone_vertices = transformRegionVertices(no_fly_zones_raw[i]);
      if (!zone_vertices.empty()) {
        no_fly_zones_.push_back(zone_vertices);
        valid_zone_count++;
        RCLCPP_INFO(getLogger(), "DMPC: Loaded no-fly zone %zu with %zu vertices", i, zone_vertices.size());
      } else {
        RCLCPP_WARN(getLogger(), "DMPC: no-fly zone %zu has invalid vertices, skipped", i);
      }
    }
    if (!no_fly_zones_.empty()) {
      RCLCPP_INFO(getLogger(), "DMPC: Loaded %d valid no-fly zones", valid_zone_count);
    } else {
      RCLCPP_INFO(getLogger(), "DMPC: no-fly zones enabled but no valid zones provided, constraint inactive");
    }
  } else {
    no_fly_zones_.clear();
    RCLCPP_INFO(getLogger(), "DMPC: no-fly zones disabled by parameter dmpc/enable_no_fly_zone");
  }

  // GetinitialUAVposition
  initial_agent_positions_.clear();
  for (size_t i = 0; i + 2 < initial_agents.size(); i += 3) {
    int id = static_cast<int>(initial_agents[i]);
    double x = initial_agents[i + 1];
    double y = initial_agents[i + 2];
    initial_agent_positions_[id] = Eigen::Vector2d(x, y);
  }

  // Initializepredict_state_
  int predict_horizon = agent_param_->predict_horizon_;
  if (predict_horizon <= 0) {
    RCLCPP_ERROR(getLogger(), "Invalid prediction horizon: %d", predict_horizon);
    return;
  }
  predict_state_.clear();
  predict_state_.resize(predict_horizon);
  for (int i = 0; i < predict_horizon; ++i) {
    predict_state_[i].position = Eigen::Vector3d::Zero();
    predict_state_[i].yaw = 0.0;
  }

  // Initialize the CVT system
  initCVTSystem();
  RCLCPP_INFO(getLogger(), "DMPC initialized with prediction horizon = %d", predict_horizon);
}

void DMPC::initCVTSystem() {
  const std::vector<Target> targets_info = getLatestTargetsInfo();
  std::vector<Point> target_points;
  for (size_t i = 0; i < targets_info.size(); i++) {
    Point p;
    p.x = targets_info[i].position.x();
    p.y = targets_info[i].position.y();
    target_points.push_back(p);
  }
  target_num = targets_info.size();
  if (agent_num >= target_num) {
    explore_uav_num = agent_num - target_num;
  } else {
    explore_uav_num = agent_num;
  }

  std::vector<int> explorer_ids;
  for (int i = 0; i < agent_num; ++i) {
    bool is_tracker = isTargetTracker(i, agent_num, target_num);
    RCLCPP_INFO(getLogger(), "DMPC:init - Agent %d isTargetTracker=%s (agent_num=%d,target_num=%d)", i,
                is_tracker ? "true" : "false", agent_num, target_num);
    if (!is_tracker) {
      explorer_ids.push_back(i);
      RCLCPP_INFO(getLogger(), "DMPC:init - Added explorer id %d", i);
    }
  }
  explore_uav_num = static_cast<int>(explorer_ids.size());
  if (explore_uav_num == 0) {
    explore_uav_num = std::max(0, agent_num - target_num);
  }

  if (explorer_ids.empty()) {
    RCLCPP_WARN(getLogger(), "No explorer UAVs, skip CVT initialization");
    cvt_assigned_ = true;
    cvt_assignment_.assign(agent_num, -1);
    return;
  }

  // BuildinitialUAVposition
  std::vector<Point> initial_drones;
  initial_drones.reserve(explore_uav_num);
  for (int id : explorer_ids) {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    auto it = initial_agent_positions_.find(id);
    if (it != initial_agent_positions_.end()) {
      pos = it->second;
    }
    initial_drones.emplace_back(pos.x(), pos.y());
    RCLCPP_INFO(getLogger(), "initCVTsystem: id:%d, pos.x:%f, pos.y:%f", id, pos.x(), pos.y());
  }

  const double mask_radius = 2.0 * v_n;
  cvt_system_ = new WeightedCVTPolygon(target_points, explore_uav_num, map_resolution, mask_radius, region_vertices_);
  if (!initial_drones.empty()) {
    cvt_system_->setInitialDrones(initial_drones);
  }
  RCLCPP_INFO(getLogger(), "initCVTsystem: explore_uav_num:%d", explore_uav_num);
  cvt_system_->computeCVT(20);
}

static double action_to_yaw(int action, bool use_large_angles = false) {
  if (use_large_angles) {
    static const double yaw_options_large[3] = {-90.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0};
    if (action < 0)
      action = 0;
    if (action > 2)
      action = 2;
    return yaw_options_large[action];
  } else {
    static const double yaw_options[3] = {-30.0 * M_PI / 180.0, 0.0, 30.0 * M_PI / 180.0};
    if (action < 0)
      action = 0;
    if (action > 2)
      action = 2;
    return yaw_options[action];
  }
}

std::vector<PredictState> DMPC::RH_GA() {
  RCLCPP_INFO(getLogger(), "RH_GA start!");
  if (node_) {
    init(node_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("dmpc"), "RH_GA: Node pointer is null. Please call initDmpc() first.");
  }
  std::vector<PredictState> best_state = solve();
  return best_state;
}

// Compute the role direction
Eigen::Vector2d DMPC::calculateRoleDirection(int agent_id, int agent_num) {
  std::vector<Target> target_info = getLatestTargetsInfo();
  target_num = target_info.size();

  if (target_info.empty()) {
    target_num = 0;
  } else {
    target_num = static_cast<int>(target_info.size());
  }

  Eigen::Vector2d current_position;
  const std::deque<AgentState> &state_history = (*agents_state_history_)[agent_id];
  current_position = state_history.back().position.head<2>();

  if (target_num > 0 && isTargetTracker(agent_id, agent_num, target_num)) {
    int target_index = getAssignedTargetIndex(agent_id, agent_num, target_num);
    if (target_index >= 0 && target_index < target_num) {
      Eigen::Vector2d target_position = target_info[target_index].position.head<2>();
      return (target_position - current_position).normalized();
    }

  } else {
    if (target_num >= 0) {
      return calculateExplorationDirection(agent_id, agent_num, target_num, current_position, target_info);
    }
  }

  return Eigen::Vector2d::Zero();
}

bool DMPC::isTargetTracker(int agent_id, int agent_num, int target_num) {
  if (target_num == 0) {
    return false;
  }

  std::vector<std::pair<int, int>> assignment = computeDynamicTargetAssignment(agent_num, target_num);

  for (const auto &pair : assignment) {
    if (pair.first == agent_id) {
      return true;
    }
  }
  return false;
}

double DMPC::computeAssignmentCost(const std::vector<std::pair<int, int>> &assignment,
                                   std::vector<Eigen::Vector2d> agent_pos, std::vector<Target> target_pos) {
  if (assignment.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;
  for (const auto &pair : assignment) {
    int agent = pair.first;
    int target = pair.second;
    if (agent >= 0 && agent < agent_pos.size() && target >= 0 && target < target_pos.size()) {
      Eigen::Vector2d agent_position = agent_pos[agent];
      Eigen::Vector2d target_position(target_pos[target].position.x(), target_pos[target].position.y());
      cost += (agent_position - target_position).norm();
    } else {
      cost += std::numeric_limits<double>::infinity();
    }
  }
  return cost;
}

std::vector<std::pair<int, int>> DMPC::computeDynamicTargetAssignment(int agent_num, int target_num) {
  std::vector<std::pair<int, int>> assignment;
  std::vector<Target> target_info = getLatestTargetsInfo();
  if (target_num == 0) {
    return assignment;
  }

  std::vector<Eigen::Vector2d> agent_positions(agent_num, Eigen::Vector2d::Zero());
  std::vector<bool> has_position(agent_num, false);

  for (int i = 0; i < agent_num; ++i) {
    if (!has_position[i]) {
      auto it = initial_agent_positions_.find(i);
      if (it != initial_agent_positions_.end()) {
        agent_positions[i] = it->second;
        has_position[i] = true;
        RCLCPP_DEBUG(node_->get_logger(), "Using initial position for agent %d: (%.2f, %.2f)", i,
                     agent_positions[i].x(), agent_positions[i].y());
      }
    }
  }

  std::vector<bool> agent_assigned(agent_num, false);
  for (int t = 0; t < target_num; t++) {
    Eigen::Vector2d target_pos(target_info[t].position.x(), target_info[t].position.y());
    int available_agents = 0;
    for (int a = 0; a < agent_num; a++) {
      if (!agent_assigned[a] && has_position[a])
        available_agents++;
    }

    double min_dist = 1e10;
    int best_agent = -1;
    for (int a = 0; a < agent_num; a++) {
      if (agent_assigned[a] || !has_position[a]) {
        continue;
      }
      double dist = (agent_positions[a] - target_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_agent = a;
      }
    }

    if (best_agent != -1) {
      assignment.push_back(std::make_pair(best_agent, t));
      agent_assigned[best_agent] = true;
    }
  }

  double new_cost = computeAssignmentCost(assignment, agent_positions, target_info);
  double last_cost = computeAssignmentCost(lastTargetAssignment_, agent_positions, target_info);

  rclcpp::Time now = node_->now();

  bool allow_time_update = is_first_assignment_ || (now - last_update_time_).seconds() > update_period_;
  bool allow_cost_update = is_first_assignment_ || new_cost < hysteresis_ratio_ * last_cost;

  if (allow_time_update && allow_cost_update) {
    lastTargetAssignment_ = assignment;
    last_update_time_ = now;
    is_first_assignment_ = false;
  }

  return lastTargetAssignment_;
}

int DMPC::getAssignedTargetIndex(int agent_id, int agent_num, int target_num) {
  if (target_num == 0) {
    return -1;
  }
  std::vector<std::pair<int, int>> target_assignment = computeDynamicTargetAssignment(agent_num, target_num);

  for (const auto &pair : target_assignment) {
    if (pair.first == agent_id) {
      return pair.second;  // Returntargetindex
    }
  }
  return -1;
}

// Get the UAV position
Eigen::Vector2d DMPC::getAgentPosition(int agent_id) const {
  if (agents_state_history_ && agent_id >= 0 && static_cast<size_t>(agent_id) < agents_state_history_->size()) {
    const auto &hist = (*agents_state_history_)[agent_id];
    if (!hist.empty()) {
      return Eigen::Vector2d(hist.back().position.x(), hist.back().position.y());
    }
  }

  auto it = initial_agent_positions_.find(agent_id);
  if (it != initial_agent_positions_.end())
    return it->second;
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d DMPC::calculateExplorationDirection(int agent_id, int agent_num, int target_num,
                                                    const Eigen::Vector2d &current_pos,
                                                    const std::vector<Target> &targets) {
  if (!cvt_assigned_) {
    explorer_positions.clear();
    explorer_ids_.clear();

    for (int i = 0; i < agent_num; ++i) {
      bool is_tracker = isTargetTracker(i, agent_num, target_num);
      if (!is_tracker) {
        explorer_positions.push_back(getAgentPosition(i));
        explorer_ids_.push_back(i);
      }
    }

    cvt_assignment_.assign(agent_num, -1);
    std::vector<bool> cvt_assigned_local(cvt_system_->drones_.size(), false);

    for (size_t i = 0; i < explorer_ids_.size(); ++i) {
      double min_dist = std::numeric_limits<double>::max();
      int best_cvt_idx = -1;

      for (size_t cvt_idx = 0; cvt_idx < cvt_system_->drones_.size(); ++cvt_idx) {
        if (cvt_assigned_local[cvt_idx])
          continue;

        const Eigen::Vector2d &agent_pos = explorer_positions[i];
        const Point &cvt_point = cvt_system_->drones_[cvt_idx];

        double dist = (agent_pos - Eigen::Vector2d(cvt_point.x, cvt_point.y)).norm();
        std::cout << "  UAV " << explorer_ids_[i] << " -> CVT " << cvt_idx << " (distance: " << dist << ")"
                  << std::endl;
        if (dist < min_dist) {
          min_dist = dist;
          best_cvt_idx = cvt_idx;
        }
      }

      if (best_cvt_idx != -1) {
        int actual_agent_id = explorer_ids_[i];
        cvt_assignment_[actual_agent_id] = best_cvt_idx;
        cvt_assigned_local[best_cvt_idx] = true;
      }
    }

    for (int id : explorer_ids_) {
      int cvt = cvt_assignment_[id];
      std::cout << "UAV " << id << " -> CVTindex: " << cvt;
      if (cvt >= 0 && cvt < cvt_system_->drones_.size()) {
        const Point &p = cvt_system_->drones_[cvt];
        std::cout << " (position: " << p.x << ", " << p.y << ")";
      }
      std::cout << std::endl;
    }
    cvt_assigned_ = true;
  }

  int cvt_index = cvt_assignment_[agent_id];
  std::cout << "currentUAV agent_id:" << agent_id << " Get cvt_index:" << cvt_index << std::endl;
  if (cvt_index >= 0 && cvt_index < cvt_system_->drones_.size()) {
    const Point &p = cvt_system_->drones_[cvt_index];
    Eigen::Vector2d target_point(p.x, p.y);
    return (target_point - current_pos).normalized();
  }
  return Eigen::Vector2d::Zero();
}

double DMPC::computeRoleCost(const Eigen::Vector2d &position, const Eigen::Vector2d &current_position,
                             const Eigen::Vector2d &role_direction, double wr) {
  double projection = (position - current_position).dot(role_direction);
  return wr * projection;
}

double DMPC::jD(const std::vector<PredictState> &predict_state, const Eigen::Vector2d &current_position,
                double wr_base) {
  if (predict_state.empty() || shouldDisableCompletedTargetReward(target_task_completed_)) {
    return 0.0;
  }

  // Compute the role direction
  if (!role_direction_initialized_) {
    bool is_tracker = isTargetTracker(agent_param_->agent_id_, agent_param_->agent_num_, target_num);
    if (!is_tracker) {
      role_direction = calculateExplorationDirection(agent_param_->agent_id_, agent_param_->agent_num_, target_num,
                                                     current_position, getLatestTargetsInfo());
      role_target_initialized_ = false;
    } else {
      role_target_initialized_ = false;
      const std::vector<Target> &target_info = getLatestTargetsInfo();
      int target_index = getAssignedTargetIndex(agent_param_->agent_id_, agent_param_->agent_num_, target_num);
      if (target_index >= 0 && target_index < static_cast<int>(target_info.size())) {
        role_target_position_ = target_info[target_index].position.head<2>();
        Eigen::Vector2d target_delta = role_target_position_ - current_position;
        if (target_delta.norm() > 1e-6) {
          role_direction = target_delta.normalized();
          role_target_initialized_ = true;
        } else {
          role_direction = Eigen::Vector2d::Zero();
        }
      } else {
        role_direction = calculateRoleDirection(agent_param_->agent_id_, agent_param_->agent_num_);
      }
    }
    role_direction_initialized_ = true;
  }

  double sum_cost = 0.0;
  for (size_t i = 0; i < predict_state.size(); ++i) {
    const auto &s = predict_state[i];
    Eigen::Vector2d pos(s.position.x(), s.position.y());

    constexpr double kRoleDecay = 0.15;
    double decay = std::exp(-kRoleDecay * static_cast<double>(i));
    double wr = wr_base * decay;

    double projection = (pos - current_position).dot(role_direction);
    sum_cost += wr * projection;
  }

  double avg_cost = sum_cost / static_cast<double>(predict_state.size());
  return avg_cost;
}

std::pair<double, double> DMPC::jA(const std::vector<PredictState> &predict_states) {
  if (!env_) {
    RCLCPP_WARN(getLogger(), "DMPC: Environment pointer is null.");
    return std::make_pair(0.0, 0.0);
  }

  std::string current_hash = generateTrajectoryHash(predict_states);
  {
    std::lock_guard<std::mutex> lock(grid_cache_mutex_);
    if (grid_cache_.find(current_hash) != grid_cache_.end()) {
      return grid_cache_[current_hash].benefit;
    }
  }
  std::vector<int> covered_grids = env_->getGridInPredictState(predict_states);

  std::pair<double, double> benefit = env_->calculateSearchBenefit(covered_grids);
  {
    std::lock_guard<std::mutex> lock(grid_cache_mutex_);
    GridCache cache_entry;
    cache_entry.covered_grids = covered_grids;
    cache_entry.benefit = benefit;
    cache_entry.valid = true;
    grid_cache_[current_hash] = cache_entry;
  }

  return benefit;
}

std::string DMPC::generateTrajectoryHash(const std::vector<PredictState> &trajectory) {
  std::ostringstream oss;
  for (const auto &state : trajectory) {
    oss << std::fixed << std::setprecision(2) << state.position.x() << "," << state.position.y() << "," << state.yaw
        << ";";
  }
  return oss.str();
}

std::pair<double, double> DMPC::jB(const std::vector<PredictState> &predictStates, double initial_yaw) {
  int m = predictStates.size();

  if (m == 0) {
    return {0.0, 0.0};
  }

  double sum_maneuver = std::abs(predictStates.front().yaw - initial_yaw);

  for (int i = 0; i < m - 1; ++i) {
    sum_maneuver += std::abs(predictStates[i + 1].yaw - predictStates[i].yaw);
  }

  const double maneuver_cost = sum_maneuver / static_cast<double>(m);

  double explored_count = 0.0;
  for (const auto &state : predictStates) {
    Eigen::Vector3i pos_id;
    env_->posToIndex(state.position, pos_id);
    int index = env_->indexToAddress(pos_id);
    if (index >= 0 && index < env_->grid_data_.size()) {
      const auto &grid = env_->grid_data_[index];
      if (grid.detect_num_ != -1 && grid.covered_) {
        explored_count += 1.0;
      }
    }
  }

  return {maneuver_cost, explored_count};
}

double DMPC::jC(const std::vector<PredictState> &predict_states) {
  const double d_max = agent_param_->apf_distance_;
  const double miu = agent_param_->apf_param1_;
  const double k = agent_param_->apf_param2_;

  const int pt_num = predict_states.size();
  if (pt_num == 0 || agents_state_history_ == nullptr)
    return 0.0;

  const int uav_num = agents_state_history_->size();
  const int uav_id = agent_param_->agent_id_;
  int valid_teammates = 0;
  for (int j = 0; j < uav_num; ++j) {
    if (j == uav_id)
      continue;
    if (!(*agents_state_history_)[j].empty())
      ++valid_teammates;
  }
  if (valid_teammates == 0 || k <= 0.0)
    return 0.0;

  double collision_cost = 0.0;

  for (int i = 0; i < pt_num; ++i) {
    const Eigen::Vector3d &pred_pos = predict_states[i].position;
    Eigen::Vector2d field_sum(0.0, 0.0);

    for (int j = 0; j < uav_num; ++j) {
      const std::deque<AgentState> &history = (*agents_state_history_)[j];
      if (history.empty())
        continue;

      const Eigen::Vector3d &teammate_pos = history.back().position;
      Eigen::Vector2d diff = (pred_pos - teammate_pos).head<2>();
      double dist = diff.norm();

      if (dist < d_max && dist > 1e-6) {
        field_sum += k * std::exp(-miu * dist) * (diff / dist);
      }
    }

    collision_cost += field_sum.norm();
  }

  return collision_cost / (static_cast<double>(pt_num) * static_cast<double>(valid_teammates));
}

void DMPC::init(rclcpp::Node::SharedPtr node) {
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("dmpc"), "DMPC::init: Node pointer is null");
    return;
  }
  node_ = node;

  if (!node_->has_parameter("dmpc/population_size")) {
    node_->declare_parameter<int>("dmpc/population_size", 60);
  }
  if (!node_->has_parameter("dmpc/generation_max")) {
    node_->declare_parameter<int>("dmpc/generation_max", 20);
  }
  if (!node_->has_parameter("dmpc/crossover_fraction")) {
    node_->declare_parameter<double>("dmpc/crossover_fraction", 0.8);
  }
  if (!node_->has_parameter("dmpc/mutation_rate")) {
    node_->declare_parameter<double>("dmpc/mutation_rate", 0.4);
  }
  if (!node_->has_parameter("dmpc/best_stall_max")) {
    node_->declare_parameter<int>("dmpc/best_stall_max", 10);
  }
  if (!node_->has_parameter("dmpc/elite_count")) {
    node_->declare_parameter<int>("dmpc/elite_count", 2);
  }
  if (!node_->has_parameter("dmpc/max_steps")) {
    node_->declare_parameter<int>("dmpc/max_steps", 9);
  }

  ga_params_.population_size = node_->get_parameter("dmpc/population_size").as_int();
  ga_params_.generation_max = node_->get_parameter("dmpc/generation_max").as_int();
  ga_params_.crossover_fraction = node_->get_parameter("dmpc/crossover_fraction").as_double();
  ga_params_.mutation_rate = node_->get_parameter("dmpc/mutation_rate").as_double();
  ga_params_.best_stall_max = node_->get_parameter("dmpc/best_stall_max").as_int();
  ga_params_.elite_count = node_->get_parameter("dmpc/elite_count").as_int();
  ga_params_.max_steps = node_->get_parameter("dmpc/max_steps").as_int();

  if (ga_params_.population_size <= 0)
    throw std::invalid_argument("Population size must be positive");
  if (ga_params_.generation_max <= 0)
    throw std::invalid_argument("Max generations must be positive");
  if (ga_params_.crossover_fraction < 0.0 || ga_params_.crossover_fraction > 1.0)
    throw std::invalid_argument("Crossover fraction must be between 0 and 1");
  if (ga_params_.mutation_rate < 0.0 || ga_params_.mutation_rate > 1.0)
    throw std::invalid_argument("Mutation rate must be between 0 and 1");
  if (ga_params_.best_stall_max <= 0)
    throw std::invalid_argument("Best stall max must be positive");
  if (ga_params_.elite_count < 0)
    throw std::invalid_argument("Elite count cannot be negative");
}

std::vector<PredictState> DMPC::getCurrentPositionTrajectory() {
  std::vector<PredictState> current_trajectory;

  if (!agents_state_history_ || agents_state_history_->size() <= agent_param_->agent_id_) {
    RCLCPP_ERROR(getLogger(), "Cannot get current position for agent %d", agent_param_->agent_id_);
    return current_trajectory;
  }

  PredictState current_state;

  {
    std::lock_guard<std::mutex> lock(realtime_position_mutex_);
    current_state.position = current_realtime_position_;
    current_state.yaw = current_realtime_yaw_;
  }

  bool realtime_position_valid = !current_state.position.isZero() && !std::isnan(current_state.position.x()) &&
                                 !std::isnan(current_state.position.y()) && !std::isnan(current_state.position.z()) &&
                                 !std::isnan(current_state.yaw);

  if (!realtime_position_valid) {
    const std::deque<AgentState> &state_history = (*agents_state_history_)[agent_param_->agent_id_];
    if (state_history.empty()) {
      RCLCPP_ERROR(getLogger(), "Agent %d has empty state history and invalid realtime position",
                   agent_param_->agent_id_);
      return current_trajectory;
    }

    RCLCPP_WARN(getLogger(), "Using historical position for agent %d due to invalid realtime position",
                agent_param_->agent_id_);
    current_state.position = state_history.back().position;
    current_state.yaw = state_history.back().yaw;
  }

  // Generate the predicted trajectory
  for (int i = 0; i < agent_param_->predict_horizon_; ++i) {
    current_trajectory.push_back(current_state);
  }

  if (realtime_position_valid) {
    RCLCPP_INFO(getLogger(), "Agent %d maintaining realtime position (%.2f, %.2f, %.2f)", agent_param_->agent_id_,
                current_state.position.x(), current_state.position.y(), current_state.position.z());
  } else {
    RCLCPP_INFO(getLogger(), "Agent %d maintaining historical position (%.2f, %.2f, %.2f)", agent_param_->agent_id_,
                current_state.position.x(), current_state.position.y(), current_state.position.z());
  }

  return current_trajectory;
}

void DMPC::setCurrentRealtimePosition(const Eigen::Vector3d &position, double yaw) {
  std::lock_guard<std::mutex> lock(realtime_position_mutex_);
  current_realtime_position_ = position;
  current_realtime_yaw_ = yaw;
}

bool DMPC::getActiveTargetPosition(Eigen::Vector2d &target_position) const {
  if (target_task_completed_ || !role_target_initialized_) {
    return false;
  }
  target_position = role_target_position_;
  return true;
}

void DMPC::markTargetTaskCompleted() {
  target_task_completed_ = true;
}

std::vector<PredictState> DMPC::solve() {
  if (agent_param_->use_expert_system_ && !Expert_System()) {
    RCLCPP_WARN(getLogger(), "Expert System failed, using default parameters");
  }
  int horizon = agent_param_->predict_horizon_;
  if (horizon <= 0) {
    RCLCPP_ERROR(getLogger(), "Invalid prediction horizon: %d", horizon);
    return {};
  }

  bool decay_role_reward = true;
  if (target_num > 0 && isTargetTracker(agent_param_->agent_id_, agent_param_->agent_num_, target_num)) {
    int target_index = getAssignedTargetIndex(agent_param_->agent_id_, agent_param_->agent_num_, target_num);
    const std::vector<Target> &target_info = getLatestTargetsInfo();
    if (target_index >= 0 && target_index < static_cast<int>(target_info.size())) {
      Eigen::Vector2d current_position = getAgentPosition(agent_param_->agent_id_);
      double target_distance = (target_info[target_index].position.head<2>() - current_position).norm();
      decay_role_reward = shouldDecayRoleReward(true, target_distance, 40.0);
    }
  }

  if (!agent_param_->use_expert_system_ && decay_role_reward) {
    agent_param_->k_wJD = agent_param_->k_wJD * std::exp(-0.2 * 5);
  }

  double max_dt = agent_param_->max_dt;
  std::vector<std::pair<double, bool>> param_combinations = {
      {max_dt, false}, {max_dt * 0.8, false}, {max_dt / 2, false}, {max_dt / 4, false}, {max_dt / 6, false},
      // {max_dt/8, false},
      // {max_dt/16, false},
      // {max_dt/4, true},
      // {max_dt/8, true},
  };

  for (const auto &params : param_combinations) {
    double dt = params.first;
    bool use_large_angles = params.second;

    RCLCPP_INFO(getLogger(), "Trying with dt=%.1f, large_angles=%s", dt, use_large_angles ? "true" : "false");

    GA_Type ga_obj;
    ga_obj.problem_mode = EA::GA_MODE::SOGA;

    ga_obj.population = ga_params_.population_size;
    ga_obj.generation_max = ga_params_.generation_max;
    ga_obj.crossover_fraction = ga_params_.crossover_fraction;
    ga_obj.mutation_rate = ga_params_.mutation_rate;
    ga_obj.best_stall_max = ga_params_.best_stall_max;
    ga_obj.elite_count = ga_params_.elite_count;

    ga_obj.init_genes = [this](MySolution &p, const std::function<double(void)> &rnd01) { this->init_genes(p, rnd01); };
    ga_obj.eval_solution = [this, dt, use_large_angles](const MySolution &p, MyMiddleCost &c) {
      return this->evaluateFitness(p, c, dt, use_large_angles);
    };

    ga_obj.mutate = [this](const MySolution &X_base, const std::function<double(void)> &rnd01, double shrink_scale) {
      return this->mutate(X_base, rnd01, shrink_scale);
    };
    ga_obj.crossover = [this](const MySolution &p1, const MySolution &p2, const std::function<double(void)> &rnd01) {
      return this->crossover(p1, p2, rnd01);
    };
    ga_obj.calculate_SO_total_fitness = [this](const GA_Type::thisChromosomeType &X) {
      return this->calculate_SO_total_fitness(X);
    };
    ga_obj.SO_report_generation = [this](int generation_number, const Generation_type &last_generation,
                                         const MySolution &best_genes) {
      return this->SO_report_generation(generation_number, last_generation, best_genes);
    };

    ga_obj.solve();

    auto &best_chromosome = ga_obj.last_generation.chromosomes[ga_obj.last_generation.best_chromosome_index];
    double best_reward = best_chromosome.middle_costs.total_reward;

    if (best_reward > -900) {
      RCLCPP_INFO(getLogger(), "Found feasible solution with dt=%.1f, large_angles=%s", dt,
                  use_large_angles ? "true" : "false");

      const auto &best_solution = best_chromosome.middle_costs.predict_states;
      const auto &best_actions = best_chromosome.genes.actions;

      std::ostringstream oss;
      oss << "Final best actions: ";
      for (auto a : best_actions)
        oss << a << " ";
      oss << "| Final best total_reward: " << best_reward;
      RCLCPP_INFO(getLogger(), oss.str().c_str());

      std::vector<PredictState> best_yaws;
      best_yaws = best_solution;
      if (best_yaws.size() > 1) {
        best_yaws.resize(path_points);
      };
      return best_yaws;
    } else {
      RCLCPP_ERROR(getLogger(), "No feasible solution found with dt=%.1f, large_angles=%s, trying next combination", dt,
                   use_large_angles ? "true" : "false");
    }
  }

  RCLCPP_ERROR(getLogger(), "No feasible solution found with any parameter combination, returning default");

  std::vector<PredictState> default_trajectory;
  PredictState default_state;
  int uav_id = agent_param_->agent_id_;
  const std::deque<AgentState> &state_history = (*agents_state_history_)[uav_id];
  default_state.position = state_history.back().position;
  default_state.yaw = state_history.back().yaw;
  default_trajectory.push_back(default_state);

  RCLCPP_INFO(getLogger(), "Returning default trajectory with 1 point");
  return default_trajectory;
}

void DMPC::init_genes(MySolution &p, const std::function<double(void)> &rnd01) {
  p.actions.resize(ga_params_.max_steps);
  for (int &a : p.actions) {
    a = static_cast<int>(rnd01() * 3);
  }
}

std::vector<PredictState> DMPC::generatePredictState(std::vector<double> yaw_seq, double dt, bool use_next_target,
                                                     const PredictState *next_target) {
  std::vector<PredictState> predict_states;
  if (!agent_param_ || !agents_state_history_ || agents_state_history_->size() <= agent_param_->agent_id_) {
    RCLCPP_WARN(getLogger(), "DMPC: Invalid agent parameters or state history.");
    return predict_states;
  }

  int uav_id = agent_param_->agent_id_;
  const std::deque<AgentState> &state_history = (*agents_state_history_)[uav_id];

  if (state_history.empty()) {
    RCLCPP_WARN(getLogger(), "DMPC: Agent %d has empty state history.", uav_id);
    return predict_states;
  }

  Eigen::Vector3d pos;
  double yaw;

  if (use_next_target && next_target != nullptr) {
    pos = next_target->position;
    yaw = next_target->yaw;
  } else {
    const AgentState &current = state_history.back();
    pos = current.position;
    yaw = current.yaw;
  }

  double velocity_threshold = agent_param_->velocity_threshold;
  double search_jump = agent_param_->search_jump;

  for (double delta_yaw : yaw_seq) {
    Eigen::Vector3d delta;
    delta << search_jump * velocity_threshold * dt * cos(yaw), search_jump * velocity_threshold * dt * sin(yaw), 0.0;

    pos += delta;

    PredictState ps;
    ps.position = pos;
    ps.yaw = yaw;

    predict_states.push_back(ps);
  }

  return predict_states;
}

const std::vector<Target> &DMPC::getLatestTargetsInfo() const {
  if (!env_) {
    static std::vector<Target> empty_targets;
    return empty_targets;
  }
  return env_->getTargetsInfoRef();
}
bool DMPC::Expert_System() {
  const std::vector<Target> &targets_info = getLatestTargetsInfo();
  RCLCPP_INFO(getLogger(), "targets_info: %d", targets_info.size());
  int target_num = targets_info.size();

  if (target_num == 0) {
    RCLCPP_WARN(getLogger(), "Expert_System: No targets info available");
    return false;
  }

  double dis_min = 1e8;
  double dis_warn = agent_param_->dis_tar_warnning_;
  int uav_id = agent_param_->agent_id_;
  const std::deque<AgentState> &state_history = (*agents_state_history_)[uav_id];

  if (state_history.empty()) {
    RCLCPP_WARN(getLogger(), "Expert_System: Empty state history for UAV %d", uav_id);
    return false;
  }
  Eigen::Vector3d current_pos = state_history.back().position;

  for (int i = 0; i < target_num; i++) {
    double dis = sqrt(pow(current_pos.x() - targets_info[i].position.x(), 2) +
                      pow(current_pos.y() - targets_info[i].position.y(), 2));
    if (dis < dis_min) {
      dis_min = dis;
    }
  }
  double E1 = dis_min / dis_warn;

  double E2 = 0.0;
  if (env_ && !env_->getGridData().empty()) {
    int tar_find = 0;
    const std::vector<GridData> &grid_data = env_->getGridData();
    for (int i = 0; i < grid_data.size(); i++) {
      if (grid_data[i].probability_ > 0.8) {
        tar_find++;
      }
    }
    E2 = static_cast<double>(tar_find) / target_num;
  }

  bool decay_role_reward = true;
  if (isTargetTracker(uav_id, agent_param_->agent_num_, target_num)) {
    int target_index = getAssignedTargetIndex(uav_id, agent_param_->agent_num_, target_num);
    if (target_index >= 0 && target_index < static_cast<int>(targets_info.size())) {
      double target_distance = (targets_info[target_index].position.head<2>() - current_pos.head<2>()).norm();
      decay_role_reward = shouldDecayRoleReward(true, target_distance, 40.0);
    }
  }
  if (decay_role_reward) {
    agent_param_->k_wJD = agent_param_->k_wJD * std::exp(-0.2 * 5);
  }

  if (E1 < 1.0) {
  } else if (E1 >= 1.0 && E1 < 2.0) {
    agent_param_->search_jump = 0.6;
    agent_param_->k_wJC = 1.0;
  } else {
    agent_param_->search_jump = 1.0;
    agent_param_->k_wJC = 0.8;
  }

  if (E2 < 0.75) {
    agent_param_->k_wJA1 = 1.0;
    agent_param_->k_wJA2 = 1.0;
    // ga_params_.max_steps = 8;
  } else if (E2 >= 0.75 && E2 < 1.0) {
    agent_param_->k_wJA1 = 0.8;
    agent_param_->k_wJA2 = 1.2;
    // ga_params_.max_steps = 10;
  } else {
    agent_param_->k_wJA1 = 0.4;
    agent_param_->k_wJA2 = 1.6;
    // ga_params_.max_steps = 12;
  }

  RCLCPP_INFO(getLogger(), "Expert System: E1=%.3f, E2=%.3f, k_wJC=%.2f, k_wJA1=%.2f, k_wJA2=%.2f, max_steps=%d", E1,
              E2, agent_param_->k_wJC, agent_param_->k_wJA1, agent_param_->k_wJA2, ga_params_.max_steps);

  return true;
}

bool DMPC::evaluateFitness(const MySolution &p, MyMiddleCost &c, double dt, bool use_large_angles) {
  std::shared_lock<std::shared_mutex> read_lock(shared_mutex_);

  c.total_reward = 0.0;
  c.reward_breakdown_valid = false;
  std::vector<PredictState> predict_states;

  std::vector<double> yaw_seq;
  for (auto a : p.actions)
    yaw_seq.push_back(action_to_yaw(a, use_large_angles));

  // Generate the predicted trajectory
  predict_states = generatePredictState(yaw_seq, dt, use_next_target_, next_target_);
  c.predict_states = predict_states;
  predict_state_ = predict_states;

  int uav_id = agent_param_->agent_id_;
  const std::deque<AgentState> &state_history = (*agents_state_history_)[uav_id];
  Eigen::Vector2d current_position(state_history.back().position.x(), state_history.back().position.y());

  double x_min = env_->map_param_.map_min_boundary_.x();
  double x_max = env_->map_param_.map_max_boundary_.x();
  double y_min = env_->map_param_.map_min_boundary_.y();
  double y_max = env_->map_param_.map_max_boundary_.y();

  double x_margin = agent_param_->safe_margin_x_;
  double y_margin = agent_param_->safe_margin_y_;
  double safe_obs = agent_param_->safe_obs_;
  double safe_uav = agent_param_->safe_uav_;

  const std::vector<Obstacle> &obstacles = env_->obs;
  bool flag = false;

  Eigen::Vector2d prev_position = current_position;
  for (size_t i = 0; i < predict_states.size(); i++) {
    const auto &state = predict_states[i];
    Eigen::Vector2d position(state.position.x(), state.position.y());

    if (!isPointInConvexPolygon(position, region_vertices_)) {
      flag = true;
      break;
    }
    if (enable_no_fly_zone_ && !no_fly_zones_.empty()) {
      bool in_any_nofly_zone = false;
      for (const auto &zone : no_fly_zones_) {
        if (isPointInConvexPolygon(position, zone)) {
          in_any_nofly_zone = true;
          break;
        }
      }
      if (in_any_nofly_zone) {
        flag = true;
        c.total_reward = -1000;
        break;
      }

      int num_samples = 10;
      Eigen::Vector2d diff = position - prev_position;
      if (diff.norm() > 1e-3) {
        for (int k = 1; k <= num_samples; ++k) {
          double alpha = static_cast<double>(k) / static_cast<double>(num_samples + 1);
          Eigen::Vector2d mid = (1.0 - alpha) * prev_position + alpha * position;
          bool in_any_zone = false;
          for (const auto &zone : no_fly_zones_) {
            if (isPointInConvexPolygon(mid, zone)) {
              in_any_zone = true;
              break;
            }
          }
          if (in_any_zone) {
            flag = true;
            c.total_reward = -1000;
            break;
          }
        }
        if (flag) {
          break;
        }
      }
    }
    prev_position = position;
  }

  if (flag) {
    c.total_reward = -1000.0;
    return false;
  }

  {
    {
      double temp_w_jA1, temp_w_jA2, temp_w_jB, temp_w_jC, temp_w_jD;
      if (agent_param_->use_expert_system_) {
        temp_w_jA1 = agent_param_->w_jA1 * agent_param_->k_wJA1;
        temp_w_jA2 = agent_param_->w_jA2 * agent_param_->k_wJA2;
        temp_w_jB = agent_param_->w_jB;
        temp_w_jC = agent_param_->w_jC * agent_param_->k_wJC;
        temp_w_jD = agent_param_->w_jD * agent_param_->k_wJD;
      } else {
        temp_w_jA1 = agent_param_->w_jA1;
        temp_w_jA2 = agent_param_->w_jA2;
        temp_w_jB = agent_param_->w_jB;
        temp_w_jC = agent_param_->w_jC;
        temp_w_jD = agent_param_->w_jD * agent_param_->k_wJD;
      }
      auto jA_result = jA(predict_states);
      const double jA1_raw = jA_result.first;
      const double jA2_raw = jA_result.second;
      const double initial_yaw = use_next_target_ && next_target_ != nullptr ? next_target_->yaw
                                                                             : state_history.back().yaw;
      const auto jB_result = jB(predict_states, initial_yaw);
      const double jB_turn_raw = jB_result.first;
      const double jB_explored_raw = jB_result.second;
      double jC_raw = jC(predict_states);
      double jD_raw = 0.0;
      if (use_initial_task_assignment) {
        const Eigen::Vector2d role_cost_origin = use_next_target_ && next_target_ != nullptr
                                                     ? next_target_->position.head<2>()
                                                     : current_position;
        jD_raw = jD(predict_states, role_cost_origin, 1.0);
      }

      int horizon = static_cast<int>(predict_states.size());
      const int max_grids_per_step = env_->getMaxGridsPerFOVStep();
      const double max_jA = static_cast<double>(horizon) * static_cast<double>(std::max(0, max_grids_per_step));
      constexpr double lambda_explored = 0.5;
      const double yaw_limit = use_large_angles ? M_PI / 2.0 : M_PI / 6.0;
      const double max_jB = yaw_limit + lambda_explored * static_cast<double>(horizon);
      const double max_jC = std::max(0.0, agent_param_->apf_param2_);
      double max_jD = 0.0;
      constexpr double role_decay = 0.15;
      const double step_length = agent_param_->search_jump * agent_param_->velocity_threshold * dt;
      for (int i = 0; i < horizon; ++i) {
        double decay = std::exp(-role_decay * static_cast<double>(i));
        max_jD += decay * static_cast<double>(i + 1) * step_length;
      }
      max_jD = (horizon > 0) ? std::abs(max_jD / static_cast<double>(horizon)) : 0.0;

      double jA1_norm = (max_jA > 0) ? std::min(jA1_raw / max_jA, 1.0) : 0.0;
      double jA2_norm = (max_jA > 0) ? std::min(jA2_raw / max_jA, 1.0) : 0.0;
      const double jB_raw = jB_turn_raw + lambda_explored * jB_explored_raw;
      double jB_norm = (max_jB > 0) ? std::min(jB_raw / max_jB, 1.0) : 0.0;
      double jC_norm = (max_jC > 0) ? std::max(0.0, std::min(jC_raw / max_jC, 1.0)) : 0.0;
      double jD_norm = (max_jD > 0) ? std::max(-1.0, std::min(jD_raw / max_jD, 1.0)) : 0.0;

      c.reward_breakdown_valid = true;
      c.raw_jA1 = jA1_raw;
      c.raw_jA2 = jA2_raw;
      c.raw_jB_turn = jB_turn_raw;
      c.raw_jB_explored = jB_explored_raw;
      c.raw_jC = jC_raw;
      c.raw_jD = jD_raw;
      c.norm_jA1 = jA1_norm;
      c.norm_jA2 = jA2_norm;
      c.norm_jB = jB_norm;
      c.norm_jC = jC_norm;
      c.norm_jD = jD_norm;
      c.contribution_jA1 = temp_w_jA1 * jA1_norm;
      c.contribution_jA2 = temp_w_jA2 * jA2_norm;
      c.contribution_jB = -temp_w_jB * jB_norm;
      c.contribution_jC = -temp_w_jC * jC_norm;
      c.contribution_jD = temp_w_jD * jD_norm;

      c.total_reward = temp_w_jA1 * jA1_norm + temp_w_jA2 * jA2_norm - temp_w_jB * jB_norm - temp_w_jC * jC_norm +
                       temp_w_jD * jD_norm;

      RCLCPP_DEBUG(getLogger(), "[norm] jA1=%.3f jA2=%.3f jB=%.3f jC=%.3f jD=%.3f | reward=%.4f", jA1_norm, jA2_norm,
                   jB_norm, jC_norm, jD_norm, c.total_reward);
    }
  }
  return true;
}

bool DMPC::isPathSafe(const Eigen::Vector2d &start, const Eigen::Vector2d &end,
                      const std::vector<Obstacle> &obstacles) {
  // Check whether the path is safe
  const int num_samples = 20;
  for (int i = 0; i < num_samples; i++) {
    double t = static_cast<double>(i) / num_samples;
    Eigen::Vector2d sample_point = start + t * (end - start);

    for (const auto &obs : obstacles) {
      double dx = sample_point.x() - obs.position.x();
      double dy = sample_point.y() - obs.position.y();
      double d = std::sqrt(dx * dx + dy * dy);

      if (d <= (obs.radius + 0.3)) {
        return false;
      }
    }
  }
  return true;
}

DMPC::MySolution DMPC::mutate(const MySolution &X_base, const std::function<double(void)> &rnd01, double shrink_scale) {
  MySolution new_sol = X_base;
  const double mu = 0.2 * shrink_scale;
  for (int &a : new_sol.actions) {
    if (rnd01() < mu) {
      a = static_cast<int>(rnd01() * 3);
    }
  }
  return new_sol;
}

DMPC::MySolution DMPC::crossover(const MySolution &p1, const MySolution &p2, const std::function<double(void)> &rnd01) {
  MySolution new_sol;
  const int cut_point = static_cast<int>(rnd01() * ga_params_.max_steps);
  for (int i = 0; i < ga_params_.max_steps; ++i) {
    new_sol.actions.push_back(i < cut_point ? p1.actions[i] : p2.actions[i]);
  }
  return new_sol;
}

double DMPC::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X) {
  return -X.middle_costs.total_reward;
}

void DMPC::SO_report_generation(int generation_number, const Generation_type &last_generation,
                                const MySolution &best_genes) {
  const auto &best_chromosome = last_generation.chromosomes[last_generation.best_chromosome_index];
  double best_reward = best_chromosome.middle_costs.total_reward;

  const auto &reward = best_chromosome.middle_costs;
  if (reward.reward_breakdown_valid) {
    RCLCPP_INFO(getLogger(),
                "[DMPC][REWARD] gen=%d total=%.6f "
                "raw=(A1 %.6f,A2 %.6f,Bturn %.6f,Bexplored %.6f,C %.6f,D %.6f) "
                "norm=(A1 %.6f,A2 %.6f,B %.6f,C %.6f,D %.6f) "
                "contrib=(A1 %.6f,A2 %.6f,B %.6f,C %.6f,D %.6f)",
                generation_number, best_reward, reward.raw_jA1, reward.raw_jA2, reward.raw_jB_turn,
                reward.raw_jB_explored, reward.raw_jC, reward.raw_jD, reward.norm_jA1, reward.norm_jA2, reward.norm_jB,
                reward.norm_jC, reward.norm_jD, reward.contribution_jA1, reward.contribution_jA2,
                reward.contribution_jB, reward.contribution_jC, reward.contribution_jD);
  }
}
