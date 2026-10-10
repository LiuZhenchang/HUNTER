#include "search_manager.h"
#include <tf2/utils.h>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include "builtin_interfaces/msg/time.hpp"
#include "custom_msgs/msg/communication_data.hpp"
#include "dmpc/dmpc.h"
#include "rclcpp/rclcpp.hpp"
#include "search_env/search_environment.h"

namespace {
constexpr int64_t kNanoSecondsPerSec = 1000000000LL;
inline builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time &time) {
  builtin_interfaces::msg::Time ros_time;
  const int64_t nanos = time.nanoseconds();
  ros_time.sec = static_cast<int32_t>(nanos / kNanoSecondsPerSec);
  ros_time.nanosec = static_cast<uint32_t>(nanos % kNanoSecondsPerSec);
  return ros_time;
}
}  // namespace

SearchManager::SearchManager() {}
SearchManager::~SearchManager() {
  communication_running_.store(false);
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }
}

std::vector<std::vector<double>> SearchManager::parseNoFlyZonesString(const std::string &zones_str) {
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

void SearchManager::init(rclcpp::Node::SharedPtr &node) {
  node_ = node;

  last_odom_msg_time = node_->now();
  node_->declare_parameter("agent_id", agent_param_.agent_id_);
  node_->declare_parameter("agent_num", agent_param_.agent_num_);
  node_->declare_parameter("agent_task", agent_param_.agent_task_);
  node_->declare_parameter("history_size", agent_param_.history_size_);
  node_->declare_parameter("init_x_", init_x_);
  node_->declare_parameter("init_y_", init_y_);
  node_->declare_parameter("init_z_", init_z_);
  node_->declare_parameter("use_init_as_takeoff_position", use_init_as_takeoff_position);
  bool use_real_uav = false;
  std::string uav_id = "uav1";
  if (!node_->has_parameter("use_real_uav")) {
    node_->declare_parameter<bool>("use_real_uav", use_real_uav);
  }
  if (!node_->has_parameter("uav_id")) {
    node_->declare_parameter<std::string>("uav_id", uav_id);
  }
  if (!node_->has_parameter("target_num")) {
    node_->declare_parameter<int>("target_num", 0);
  }
  // agent_param_.agent_id_ = agent_param_.agent_id_ -1;

  node_->get_parameter("agent_id", agent_param_.agent_id_);
  node_->get_parameter("agent_num", agent_param_.agent_num_);
  node_->get_parameter("agent_task", agent_param_.agent_task_);
  node_->get_parameter("history_size", agent_param_.history_size_);
  node_->get_parameter("init_x_", init_x_);
  node_->get_parameter("init_y_", init_y_);
  node_->get_parameter("init_z_", init_z_);
  node_->get_parameter("use_init_as_takeoff_position", use_init_as_takeoff_position);
  node_->get_parameter("use_real_uav", use_real_uav);
  node_->get_parameter("uav_id", uav_id);

  node_->declare_parameter<int>("event_trigger/window_size", 3);
  node_->declare_parameter<int>("event_trigger/execution_stride", 3);
  node_->declare_parameter<double>("event_trigger/probability_weight", 0.4);
  node_->declare_parameter<double>("event_trigger/uncertainty_weight", 0.3);
  node_->declare_parameter<double>("event_trigger/probability_delta_weight", 0.2);
  node_->declare_parameter<double>("event_trigger/uncertainty_delta_weight", 0.1);
  node_->declare_parameter<double>("event_trigger/threshold", 0.25);
  node_->declare_parameter<std::string>("event_trigger/log_path", "");
  event_trigger_window_size_ = std::max(1,
                                        static_cast<int>(node_->get_parameter("event_trigger/window_size").as_int()));
  event_trigger_execution_stride_ = std::max(1, static_cast<int>(
                                                    node_->get_parameter("event_trigger/execution_stride").as_int()));
  event_trigger_ = std::make_unique<EventTrigger>(
      EventTriggerConfig{node_->get_parameter("event_trigger/probability_weight").as_double(),
                         node_->get_parameter("event_trigger/uncertainty_weight").as_double(),
                         node_->get_parameter("event_trigger/probability_delta_weight").as_double(),
                         node_->get_parameter("event_trigger/uncertainty_delta_weight").as_double(),
                         node_->get_parameter("event_trigger/threshold").as_double()});

  node_->declare_parameter<double>("target_completion/arrival_distance", 20.0);
  node_->declare_parameter<int>("target_completion/consecutive_samples", 3);
  target_arrival_distance_ = std::max(0.0, node_->get_parameter("target_completion/arrival_distance").as_double());
  target_required_consecutive_samples_ =
      std::max(1, static_cast<int>(node_->get_parameter("target_completion/consecutive_samples").as_int()));
  target_consecutive_samples_ = 0;

  std::string event_log_path = node_->get_parameter("event_trigger/log_path").as_string();
  if (event_log_path.empty()) {
    const char *home = std::getenv("HOME");
    event_log_path = std::string(home ? home : ".") + "/event_trigger_metrics_uav" +
                     std::to_string(agent_param_.agent_id_) + ".csv";
  }
  event_metrics_file_.open(event_log_path, std::ios::out | std::ios::trunc);
  if (event_metrics_file_.is_open()) {
    event_metrics_file_ << "timestamp_sec,distance_m,event_triggered,max_probability,mean_uncertainty,"
                           "probability_delta,uncertainty_delta,trigger_metric,ga_call_count,"
                           "first_target_detection_sec,reason\n";
    event_metrics_file_.flush();
    RCLCPP_INFO(node_->get_logger(), "Event-trigger metrics: %s", event_log_path.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Cannot open event-trigger metrics file: %s", event_log_path.c_str());
  }

  // no-fly zoneparameters
  node_->declare_parameter<std::string>("no_fly_zones", "[]");
  node_->declare_parameter<bool>("dmpc/enable_no_fly_zone", true);
  std::string no_fly_zones_str = node_->get_parameter("no_fly_zones").as_string();
  bool enable_no_fly_zone = node_->get_parameter("dmpc/enable_no_fly_zone").as_bool();

  int target_num = 0;
  node_->get_parameter("target_num", target_num);
  RCLCPP_INFO(node_->get_logger(), "Param target_num (node) = %d", target_num);

  std::vector<Eigen::Vector3d> positions;
  if (target_num > 0) {
    std::vector<double> target_x(target_num, -1.0), target_y(target_num, -1.0);
    positions.resize(target_num);
    for (int i = 0; i < target_num; ++i) {
      std::string target_x_param = "target" + std::to_string(i) + "_x";
      std::string target_y_param = "target" + std::to_string(i) + "_y";
      if (!node_->has_parameter(target_x_param))
        node_->declare_parameter(target_x_param, -1.0);
      if (!node_->has_parameter(target_y_param))
        node_->declare_parameter(target_y_param, -1.0);
      node_->get_parameter(target_x_param, target_x[i]);
      node_->get_parameter(target_y_param, target_y[i]);
      positions[i] = {target_x[i], target_y[i], 0};
      RCLCPP_INFO(node_->get_logger(), "target%d: pos(%.2f, %.2f)", i, target_x[i], target_y[i]);
    }
  } else {
    targets_info_.clear();
    RCLCPP_INFO(node_->get_logger(), "target_num == 0 -> no prior targets configured");
  }

  RCLCPP_INFO(node_->get_logger(), "agent_id: %d", agent_param_.agent_id_);
  RCLCPP_INFO(node_->get_logger(), "agent_num: %d", agent_param_.agent_num_);
  RCLCPP_INFO(node_->get_logger(), "agent_task: %d", agent_param_.agent_task_);
  RCLCPP_INFO(node_->get_logger(), "history_size: %d", agent_param_.history_size_);

  auto comm_qos = rclcpp::SensorDataQoS();
  auto odom_qos = rclcpp::SensorDataQoS();

  agent_msg_pub_ = node_->create_publisher<custom_msgs::msg::CommunicationData>(
      "/drone_" + std::to_string(agent_param_.agent_id_) + "_communication_data", comm_qos);
  for (int i = 0; i < agent_param_.agent_num_; ++i) {
    if (i != agent_param_.agent_id_) {
      std::string topic_name = "/drone_" + std::to_string(i) + "_communication_data";
      RCLCPP_INFO(node_->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
      agent_msg_subs_.emplace_back(
          node_->create_subscription<custom_msgs::msg::CommunicationData>(topic_name, comm_qos,
                                                                          std::bind(&SearchManager::agentMsgCallback,
                                                                                    this, std::placeholders::_1)));
    }
  }

  if (!use_real_uav) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("odom", 100,
                                                                    std::bind(&SearchManager::agentOdomCallback, this,
                                                                              std::placeholders::_1));
  } else {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/" + uav_id + "/local_position/odom", odom_qos,
                                                                    std::bind(&SearchManager::agentOdomCallback, this,
                                                                              std::placeholders::_1));
  }
  // target_sub_ =
  // node_->create_subscription<custom_msgs::msg::ObjectComputation>("/inner/information/object_computation", qos,
  // std::bind(&SearchManager::targetMsgCallback, this, std::placeholders::_1));
  env = std::make_shared<SearchEnvironment>();
  env->setAgentsStateHistory(&agents_state_history_);
  env->setTargetsInfo(&targets_info_);
  env->setAgentParam(&agent_param_);
  if (enable_no_fly_zone && !no_fly_zones_str.empty() && no_fly_zones_str != "[]") {
    std::vector<std::vector<double>> no_fly_zones_raw = parseNoFlyZonesString(no_fly_zones_str);
    std::vector<std::vector<Eigen::Vector2d>> no_fly_zones;
    int valid_count = 0;
    for (size_t i = 0; i < no_fly_zones_raw.size(); ++i) {
      auto zone_vertices = transformRegionVertices(no_fly_zones_raw[i]);
      if (!zone_vertices.empty()) {
        no_fly_zones.push_back(zone_vertices);
        valid_count++;
        RCLCPP_INFO(node_->get_logger(), "SearchManager: Parsed no-fly zone %zu with %zu vertices", i,
                    zone_vertices.size());
      }
    }
    env->setNoFlyZones(no_fly_zones);
    RCLCPP_INFO(node_->get_logger(), "SearchManager: Set %d valid no-fly zones", valid_count);
  }
  env->initEnv(node_);
  dmpc = std::make_unique<DMPC>();
  dmpc->setAgentsStateHistory(&agents_state_history_);
  dmpc->setAgentParam(&agent_param_);
  dmpc->setEnvironment(env);
  for (const auto &pos : positions) {
    Target target;
    target.position = pos;
    env->posToIndex(pos, target.grid);
    target.time_stamp = node_->now();
    targets_info_.push_back(target);
  }
  dmpc->initDmpc(node_);
  agents_state_history_.clear();
  agents_state_history_.resize(agent_param_.agent_num_);

  history_index_.assign(agent_param_.agent_num_, 0);
  comm_history_index_.assign(agent_param_.agent_num_, 0);

  // Keep communication publication at 10 Hz even while the planning loop runs GA.
  communication_running_.store(true);
  communication_thread_ = std::thread([this]() {
    auto next_publish = std::chrono::steady_clock::now();
    while (communication_running_.load()) {
      next_publish += std::chrono::milliseconds(100);
      agentMsgPublish();
      std::this_thread::sleep_until(next_publish);
    }
  });
}

bool SearchManager::isTriggerDmpc(int index) {
  std::vector<PredictState> predict_traj = latestPredictStateGet();
  if (predict_traj.empty()) {
    return false;
  }

  if (index < 0 || index >= static_cast<int>(predict_traj.size())) {
    RCLCPP_WARN(node_->get_logger(), "isTriggerDmpc: invalid index %d (trajectory size %zu)", index,
                predict_traj.size());
    return false;
  }
  const int window_length = std::min(event_trigger_window_size_, static_cast<int>(predict_traj.size()) - index);
  std::vector<PredictState> prefix_traj(predict_traj.begin() + index, predict_traj.begin() + index + window_length);

  std::vector<int> covered_grid_ids = env->getGridInPredictState(prefix_traj);
  if (covered_grid_ids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "checkTriggerDmpc: no covered grids for prefix trajectory, skip trigger check.");
    writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "event_check");
    return false;
  }

  const std::vector<GridData> &grid_data = env->getGridData();
  double max_probability = 0.0;
  double uncertainty_sum = 0.0;
  int valid_grid_count = 0;

  for (int id : covered_grid_ids) {
    if (id < 0 || id >= static_cast<int>(grid_data.size())) {
      continue;
    }
    max_probability = std::max(max_probability, grid_data[id].probability_);
    uncertainty_sum += grid_data[id].uncertainty_;
    ++valid_grid_count;
  }
  if (valid_grid_count == 0) {
    RCLCPP_WARN(node_->get_logger(),
                "checkTriggerDmpc: no valid covered grids for prefix trajectory, skip trigger check.");
    writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "event_check");
    return false;
  }

  const double mean_uncertainty = uncertainty_sum / static_cast<double>(valid_grid_count);
  const EventTriggerResult result = event_trigger_->evaluate(max_probability, mean_uncertainty);
  writeEventMetrics(result, "event_check");
  RCLCPP_INFO(node_->get_logger(), "Event trigger: Pmax=%.4f Umean=%.4f dP=%.4f dU=%.4f eta=%.4f triggered=%d",
              result.max_probability, result.mean_uncertainty, result.probability_delta, result.uncertainty_delta,
              result.metric, result.triggered ? 1 : 0);
  return result.triggered;
}

void SearchManager::resetEventTrigger() {
  if (event_trigger_) {
    event_trigger_->reset();
  }
}

void SearchManager::recordForcedReplan() {
  writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "trajectory_exhausted");
}

void SearchManager::writeEventMetrics(const EventTriggerResult &result, const char *reason) {
  std::lock_guard<std::mutex> lock(event_metrics_file_mutex_);
  if (!event_metrics_file_.is_open()) {
    return;
  }

  double elapsed = 0.0;
  double distance = 0.0;
  {
    std::lock_guard<std::mutex> state_lock(mission_metrics_mutex_);
    elapsed = metrics_started_ ? (node_->now() - metrics_start_time_).seconds() : 0.0;
    distance = total_distance_;
  }
  const auto first_target_time = event_metrics_.firstTargetDetectionTime();
  event_metrics_file_ << std::fixed << std::setprecision(6) << elapsed << ',' << distance << ','
                      << (result.triggered ? 1 : 0) << ',' << result.max_probability << ',' << result.mean_uncertainty
                      << ',' << result.probability_delta << ',' << result.uncertainty_delta << ',' << result.metric
                      << ',' << event_metrics_.gaCallCount() << ',';
  if (first_target_time.has_value()) {
    event_metrics_file_ << *first_target_time;
  }
  event_metrics_file_ << ',' << reason << '\n';
  event_metrics_file_.flush();
}

bool SearchManager::isReceivedAgentData() const {
  const auto &self_history = agents_state_history_[agent_param_.agent_id_];
  if (self_history.size() < 2) {
    return false;
  }

  return true;
}

void SearchManager::targetMsgCallback(const custom_msgs::msg::ObjectComputation::SharedPtr msg) {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  int target_num = msg->objs.size();

  double target_detection_time = -1.0;
  {
    std::lock_guard<std::mutex> lock(mission_metrics_mutex_);
    if (target_num > 0 && metrics_started_) {
      target_detection_time = (node_->now() - metrics_start_time_).seconds();
    }
  }
  if (target_detection_time >= 0.0) {
    event_metrics_.recordTargetDetection(target_detection_time);
    writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "target_detected");
  }

  for (int i = 0; i < target_num; i++) {
    Target target;
    double x = msg->objs[i].x / 1000.0;
    double y = msg->objs[i].y / 1000.0;
    double z = msg->objs[i].z / 1000.0;
    target.position = Eigen::Vector3d(x, y, z);
    env->posToIndex(target.position, target.grid);
    target.time_stamp = rclcpp::Time(msg->header.stamp);
    targets_info_.push_back(target);
    RCLCPP_INFO(node_->get_logger(), "Received target %d: pos(%.2f, %.2f, %.2f)", i, x, y, z);
  }
}
void SearchManager::agentMsgCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg) {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  rclcpp::Time now = node_->now();
  int sender_id = msg->sender_id;
  int uav_id = agent_param_.agent_id_;
  int history_size = agent_param_.history_size_;

  auto last_msg_it = last_agent_msg_times_.find(sender_id);
  if (last_msg_it != last_agent_msg_times_.end() && (now - last_msg_it->second).seconds() < 0.1) {
    return;
  }
  last_agent_msg_times_[sender_id] = now;

  if (msg->sender_id != uav_id) {
    // Merge local and received records so out-of-order or repeated packets
    // cannot discard valid history already stored by this receiver.
    for (int agent_id = 0; agent_id < agent_param_.agent_num_; ++agent_id) {
      if (agent_id == uav_id) {
        continue;
      }

      std::vector<AgentState> merged_history(agents_state_history_[agent_id].begin(),
                                             agents_state_history_[agent_id].end());
      for (const auto &time_step : msg->agents_history) {
        for (const auto &received_state : time_step.history) {
          if (received_state.agent_id != agent_id ||
              (received_state.x == 0.0 && received_state.y == 0.0 && received_state.time_stamp.sec == 0 &&
               received_state.time_stamp.nanosec == 0)) {
            continue;
          }

          AgentState new_state;
          new_state.grid = Eigen::Vector3i::Zero();
          new_state.velocity = Eigen::Vector3d::Zero();
          new_state.position = Eigen::Vector3d(received_state.x, received_state.y, 0.0);
          new_state.yaw = received_state.yaw;
          new_state.time_stamp = rclcpp::Time(received_state.time_stamp);
          merged_history.push_back(new_state);
        }
      }

      if (merged_history.empty()) {
        continue;
      }

      std::stable_sort(merged_history.begin(), merged_history.end(), [](const AgentState &lhs, const AgentState &rhs) {
        return lhs.time_stamp.nanoseconds() < rhs.time_stamp.nanoseconds();
      });

      std::vector<AgentState> deduplicated_history;
      deduplicated_history.reserve(merged_history.size());
      for (const auto &state : merged_history) {
        if (!deduplicated_history.empty() && deduplicated_history.back().time_stamp == state.time_stamp) {
          deduplicated_history.back() = state;
        } else {
          deduplicated_history.push_back(state);
        }
      }

      const size_t keep_count = std::min(static_cast<size_t>(std::max(history_size, 0)), deduplicated_history.size());
      agents_state_history_[agent_id].assign(deduplicated_history.end() - keep_count, deduplicated_history.end());
    }

    // Updatetarget information
    for (const auto &target : msg->targets_info) {
      bool updated = false;
      Eigen::Vector3i received_grid(target.grid_x, target.grid_y, 0);
      const rclcpp::Time received_stamp(target.time_stamp);

      for (auto &local_target : targets_info_) {
        if (local_target.grid == received_grid) {
          if (target.grid_x == local_target.grid[0] && target.grid_y == local_target.grid[1]) {
            if (received_stamp > local_target.time_stamp) {
              local_target.grid = received_grid;
              env->indexToPos(received_grid, local_target.position);
              local_target.time_stamp = received_stamp;
            }
            updated = true;
            break;
          }
        }
      }

      if (!updated) {
        Target new_target;
        new_target.grid = received_grid;
        env->indexToPos(received_grid, new_target.position);
        new_target.time_stamp = received_stamp;
        targets_info_.push_back(new_target);
      }
    }
    env->setOtherAgentsCoveredGrid();
  }
}

void SearchManager::agentOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg) {
    RCLCPP_ERROR(node_->get_logger(), "agentOdomCallback: msg is null");
    odom_received_ = false;
    return;
  }
  odom_received_ = true;
  rclcpp::Time now = node_->now();
  if ((now - last_odom_msg_time).seconds() < 0.01) {
    return;
  }
  last_odom_msg_time = now;
  int uav_id = agent_param_.agent_id_;
  int history_size = agent_param_.history_size_;
  std::lock_guard<std::mutex> state_lock(state_mutex_);

  static rclcpp::Time last_odom_log_time = node_->now();
  if ((node_->now() - last_odom_log_time).seconds() > 1.0) {
    // RCLCPP_INFO(
    //     node_->get_logger(),
    //     "[TrackDebug] Agent %d raw odom (%.2f, %.2f, %.2f) use_init_as_takeoff=%d init(%.2f, %.2f, %.2f)",
    //     uav_id,
    //     msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
    //     use_init_as_takeoff_position ? 1 : 0,
    //     init_x_, init_y_, init_z_
    // );
    last_odom_log_time = node_->now();
  }

  Eigen::Vector3d odom_position;
  if (use_init_as_takeoff_position) {
    double takeoff_x = init_x_ + msg->pose.pose.position.x;
    double takeoff_y = init_y_ + msg->pose.pose.position.y;
    double takeoff_z = init_z_;
    odom_position = Eigen::Vector3d(takeoff_x, takeoff_y, takeoff_z);
  } else {
    odom_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  {
    std::lock_guard<std::mutex> lock(mission_metrics_mutex_);
    if (previous_odom_position_.has_value()) {
      total_distance_ += (odom_position - *previous_odom_position_).head<2>().norm();
    }
    previous_odom_position_ = odom_position;
  }

  Eigen::Vector3d odom_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  if (uav_id >= 0 && uav_id < agents_state_history_.size()) {
    AgentState new_state;
    // std::cout << "[Odometry] world(x: " << odom_position.x() << ", y: " << odom_position.y() << "), theta: " <<
    // tf::getYaw(msg->pose.pose.orientation) << std::endl;
    env->posToIndex(odom_position, new_state.grid);
    new_state.position = odom_position;
    new_state.yaw = tf2::getYaw(msg->pose.pose.orientation);
    new_state.velocity = odom_velocity;
    new_state.time_stamp = node_->now();
    agents_state_history_[uav_id].push_back(new_state);
    if (agents_state_history_[uav_id].size() > history_size) {
      agents_state_history_[uav_id].pop_front();
    }

    updateDMPCRealtimePosition(uav_id, odom_position, new_state.yaw);
    updateTargetCompletion(odom_position);
  }
}

void SearchManager::updateDMPCRealtimePosition(int agent_id, const Eigen::Vector3d &position, double yaw) {
  if (dmpc && agent_id == agent_param_.agent_id_) {
    dmpc->setCurrentRealtimePosition(position, yaw);
  }
}

void SearchManager::updateTargetCompletion(const Eigen::Vector3d &odom_position) {
  if (!dmpc) {
    return;
  }

  Eigen::Vector2d target_position;
  if (!dmpc->getActiveTargetPosition(target_position)) {
    return;
  }

  const double target_distance = (odom_position.head<2>() - target_position).norm();
  target_consecutive_samples_ = target_distance <= target_arrival_distance_ ? target_consecutive_samples_ + 1 : 0;
  if (target_consecutive_samples_ < target_required_consecutive_samples_) {
    return;
  }
  target_consecutive_samples_ = 0;

  dmpc->markTargetTaskCompleted();
  writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "target_reached");
  RCLCPP_INFO(node_->get_logger(), "Target task complete: agent=%d target=(%.2f, %.2f) distance=%.2f; JD disabled",
              agent_param_.agent_id_, target_position.x(), target_position.y(), target_distance);
}

bool SearchManager::getUseInitAsTakeoffPosition() const {
  return use_init_as_takeoff_position;
}

void SearchManager::agentMsgPublish() {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  int uav_id = agent_param_.agent_id_;
  int history_size = agent_param_.history_size_;
  custom_msgs::msg::CommunicationData msg;
  msg.sender_id = uav_id;
  msg.send_timestamp = toBuiltinTime(node_->now());

  // Send only history entries that actually exist; do not fill missing entries with defaults.
  size_t actual_history_size = 0;
  for (size_t agent_id = 0; agent_id < agents_state_history_.size(); ++agent_id) {
    actual_history_size = std::max(actual_history_size, agents_state_history_[agent_id].size());
  }

  static rclcpp::Time last_comm_log_time = node_->now();
  if ((node_->now() - last_comm_log_time).seconds() > 1.0) {
    // RCLCPP_INFO(node_->get_logger(), "Agent %d publishing communication data with %lu agents_history entries",
    // uav_id, actual_history_size);
    last_comm_log_time = node_->now();
  }

  msg.agents_history.resize(actual_history_size);
  for (size_t t = 0; t < actual_history_size; ++t) {
    msg.agents_history[t].history.resize(agents_state_history_.size());
    for (size_t agent_id = 0; agent_id < agents_state_history_.size(); ++agent_id) {
      if (t < agents_state_history_[agent_id].size()) {
        const auto &state = agents_state_history_[agent_id][t];
        msg.agents_history[t].history[agent_id].x = state.position[0];
        msg.agents_history[t].history[agent_id].y = state.position[1];
        msg.agents_history[t].history[agent_id].yaw = state.yaw;
        msg.agents_history[t].history[agent_id].time_stamp = toBuiltinTime(state.time_stamp);
        msg.agents_history[t].history[agent_id].agent_id = agent_id;
      }
    }
  }
  // Publish current target information.
  msg.targets_info.resize(targets_info_.size());
  for (size_t i = 0; i < targets_info_.size(); ++i) {
    msg.targets_info[i].grid_x = targets_info_[i].grid[0];
    msg.targets_info[i].grid_y = targets_info_[i].grid[1];
    msg.targets_info[i].time_stamp = toBuiltinTime(targets_info_[i].time_stamp);
  }
  agent_msg_pub_->publish(msg);
}

void SearchManager::gridDataUpdate() {
  env->updateGridData();
}

void SearchManager::printAgentStateHistory() {
  RCLCPP_INFO(node_->get_logger(), "==== Printing Agent State History ====");
  for (size_t agent_id = 0; agent_id < agents_state_history_.size(); ++agent_id) {
    RCLCPP_INFO(node_->get_logger(), "Agent ID: %d, Current Index: %d", agent_id, comm_history_index_[agent_id]);
    for (size_t t = 0; t < agents_state_history_[agent_id].size(); ++t) {
      const auto &state = agents_state_history_[agent_id][t];
      RCLCPP_INFO(
          node_->get_logger(),
          "  Time Step %d | TimeStamp: %f | Grid: (%d, %d) | Position: (%f, %f) | Yaw: %f | Velocity: (%f, %f, %f)", t,
          state.time_stamp.seconds(), state.grid(0), state.grid(1), state.position.x(), state.position.y(), state.yaw,
          state.velocity.x(), state.velocity.y(), state.velocity.z());
    }
  }
}

void SearchManager::envGridDataPrint() {
  env->printGridData();
}

const std::vector<GridData> &SearchManager::envGridDataGet() {
  return env->getGridData();
}

void SearchManager::executeDMPCDecision(bool use_next_target, const PredictState *next_target) {
  if (!dmpc) {
    RCLCPP_WARN(node_->get_logger(), "DMPC pointer is null!");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mission_metrics_mutex_);
    if (!metrics_started_) {
      metrics_start_time_ = node_->now();
      metrics_started_ = true;
    }
  }
  event_metrics_.recordGaCall();
  writeEventMetrics({0.0, 0.0, 0.0, 0.0, 0.0, false}, "ga_call");

  dmpc->setStartPositonMode(use_next_target, next_target);

  RCLCPP_INFO(node_->get_logger(), "Starting DMPC decision...");
  std::vector<PredictState> new_trajectory = dmpc->RH_GA();

  {
    std::lock_guard<std::mutex> lock(latest_predict_state_mutex_);
    latest_predict_state_ = new_trajectory;
  }

  RCLCPP_INFO(node_->get_logger(), "DMPC returned trajectory with %zu points", new_trajectory.size());

  if (new_trajectory.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "DMPC returned empty trajectory!");
  } else {
    for (size_t i = 0; i < new_trajectory.size(); ++i) {
      const auto &pt = new_trajectory[i];
      RCLCPP_INFO(node_->get_logger(), "Agent[%d],Waypoint %zu: pos(%.2f, %.2f, %.2f), yaw=%.2f",
                  agent_param_.agent_id_, i, pt.position.x(), pt.position.y(), pt.position.z(), pt.yaw);
    }
  }
}

std::vector<PredictState> SearchManager::latestPredictStateGet() {
  std::lock_guard<std::mutex> lock(latest_predict_state_mutex_);
  return latest_predict_state_;
}

const std::vector<std::deque<AgentState>> &SearchManager::agentsStateHistoryGet() const {
  return agents_state_history_;
}

const std::vector<Obstacle> &SearchManager::getObstacles() const {
  return env->obs;
};
