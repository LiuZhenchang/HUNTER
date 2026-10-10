#include "search_env/search_environment.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

SearchEnvironment::SearchEnvironment() {}
SearchEnvironment::~SearchEnvironment() {}

rclcpp::Logger SearchEnvironment::getLogger() const {
  if (auto node = node_.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("SearchEnvironment");
}

std::string expand_path(const std::string &path) {
  if (path.empty())
    return path;

  if (path[0] == '~' && (path.size() == 1 || path[1] == '/')) {
    const char *home = std::getenv("HOME");
    if (home) {
      if (path.size() == 1) {
        return std::string(home);
      } else {
        return std::string(home) + path.substr(1);
      }
    }
  }

  return path;
}

void SearchEnvironment::initEnv(rclcpp::Node::SharedPtr node) {
  node_ = node;
  double x_size, y_size, z_size;
  int target_num;

  node->declare_parameter<double>("map/resolution", -1.0);
  node->declare_parameter<int>("sensor/type", 0);
  node->declare_parameter<double>("sensor/pd", -1.0);
  node->declare_parameter<double>("sensor/pf", -1.0);
  node->declare_parameter<bool>("show_search_result", false);
  node->declare_parameter<double>("search_time", 500.0);
  node->declare_parameter<bool>("plot_on_all_uavs", false);
  node->declare_parameter<double>("plot_interval", 0.5);
  if (!node->has_parameter("target_num")) {
    node->declare_parameter<int>("target_num", 1);
  }
  std::vector<double> region_vertices_raw;
  if (!node->has_parameter("region_vertices")) {
    region_vertices_raw = {-1000.0, -1000.0, -500.0, 0.0, 300.0, 0.0, 1000.0, -1000.0};
    node->declare_parameter<std::vector<double>>("region_vertices", region_vertices_raw);
  }
  region_vertices_raw = node->get_parameter("region_vertices").as_double_array();

  RCLCPP_INFO(getLogger(), "Task region vertices (counter-clockwise):");
  for (size_t i = 0; i < region_vertices_raw.size(); i += 2) {
    RCLCPP_INFO(getLogger(), "  Vertex %zu: (%.1f, %.1f)", i / 2, region_vertices_raw[i], region_vertices_raw[i + 1]);
  }

  region_vertices_ = transformRegionVertices(region_vertices_raw);

  map_param_.resolution_ = node->get_parameter("map/resolution").as_double();
  detect_param_.type = node->get_parameter("sensor/type").as_int();
  detect_param_.pd = node->get_parameter("sensor/pd").as_double();
  detect_param_.pf = node->get_parameter("sensor/pf").as_double();
  target_num = node->get_parameter("target_num").as_int();
  show_search_result = node->get_parameter("show_search_result").as_bool();
  search_time = node->get_parameter("search_time").as_double();
  plot_on_all_uavs = node->get_parameter("plot_on_all_uavs").as_bool();
  plot_interval = node->get_parameter("plot_interval").as_double();

  std::vector<Target> target_prior_(target_num);
  bool use_target_prior_;

  node->declare_parameter<bool>("use_target_prior", false);
  use_target_prior_ = node->get_parameter("use_target_prior").as_bool();

  std::vector<double> target_x(target_num, -1.0), target_y(target_num, -1.0);
  for (int i = 0; i < target_num; i++) {
    std::string key_x = "target" + std::to_string(i) + "_x";
    std::string key_y = "target" + std::to_string(i) + "_y";
    if (!node->has_parameter(key_x)) {
      node->declare_parameter<double>(key_x, -1.0);
    }
    if (!node->has_parameter(key_y)) {
      node->declare_parameter<double>(key_y, -1.0);
    }
    target_x[i] = node->get_parameter("target" + std::to_string(i) + "_x").as_double();
    target_y[i] = node->get_parameter("target" + std::to_string(i) + "_y").as_double();
  }

  int uav_id = agent_param_->agent_id_;
  uncertainty_pub_ = node->create_publisher<std_msgs::msg::Float64>("/uav" + std::to_string(uav_id) +
                                                                        "/uncertainty_data",
                                                                    10);
  probability_pub_ = node->create_publisher<std_msgs::msg::Float64>("/uav" + std::to_string(uav_id) +
                                                                        "/probability_data",
                                                                    10);

  // Initialize map parameters
  map_param_.resolution_inv_ = 1 / map_param_.resolution_;
  map_param_.map_origin_ = Eigen::Vector3d(0, 0, 0);
  // map_param_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  std::vector<double> vertices_x;
  std::vector<double> vertices_y;

  for (size_t i = 0; i < region_vertices_.size(); i++) {
    vertices_x.push_back(region_vertices_[i].x());
    vertices_y.push_back(region_vertices_[i].y());
  }

  // Compute the bounding rectangle
  if (!vertices_x.empty() && !vertices_y.empty()) {
    double min_x = *std::min_element(vertices_x.begin(), vertices_x.end());
    double max_x = *std::max_element(vertices_x.begin(), vertices_x.end());
    double min_y = *std::min_element(vertices_y.begin(), vertices_y.end());
    double max_y = *std::max_element(vertices_y.begin(), vertices_y.end());

    map_param_.map_min_boundary_ = Eigen::Vector3d(min_x, min_y, 0);
    map_param_.map_max_boundary_ = Eigen::Vector3d(max_x, max_y, 0);
    map_param_.map_size_ = Eigen::Vector3d(max_x - min_x, max_y - min_y, z_size);
    map_param_.map_voxel_num_ = Eigen::Vector3i(ceil(map_param_.map_size_(0) / map_param_.resolution_),
                                                ceil(map_param_.map_size_(1) / map_param_.resolution_),
                                                ceil(map_param_.map_size_(2) / map_param_.resolution_));

    RCLCPP_INFO(getLogger(), "Convex region bounding box: X[%.2f, %.2f], Y[%.2f, %.2f]", min_x, max_x, min_y, max_y);
  }

  int grid_count = map_param_.map_voxel_num_(0) * map_param_.map_voxel_num_(1);

  RCLCPP_INFO(getLogger(), "Search environment: creating grid of size %d x %d = %d", map_param_.map_voxel_num_(0),
              map_param_.map_voxel_num_(1), grid_count);
  double c_n, v_n;
  node->declare_parameter<double>("c_n", 0.3);
  node->declare_parameter<double>("v_n", 350.0);
  c_n = node->get_parameter("c_n").as_double();
  v_n = node->get_parameter("v_n").as_double();
  grid_data_.resize(grid_count, {0, 0, 0, false});
  for (size_t i = 0; i < target_prior_.size(); ++i) {
    target_prior_[i].position = Eigen::Vector3d(target_x[i], target_y[i], 0);
    posToIndex(target_prior_[i].position, target_prior_[i].grid);
    target_prior_[i].time_stamp = node->now();
  }

  for (size_t i = 0; i < grid_data_.size(); ++i) {
    Eigen::Vector3i index;
    Eigen::Vector3d position;
    addressToIndex(i, index);
    indexToPos(index, position);
    double x = position(0);
    double y = position(1);
    Eigen::Vector2d position_2d(position(0), position(1));
    bool in_task_region = isPointInConvexPolygon(position_2d, region_vertices_);
    bool in_no_fly_zone = false;
    for (const auto &zone : no_fly_zones_) {
      if (isPointInConvexPolygon(position_2d, zone)) {
        in_no_fly_zone = true;
        break;
      }
    }

    if (in_task_region && !in_no_fly_zone) {
      if (use_target_prior_) {
        for (size_t k = 0; k < target_prior_.size(); ++k) {
          double x_t = target_prior_[k].position[0];
          double y_t = target_prior_[k].position[1];
          grid_data_[i].probability_ += c_n * exp(-((x - x_t) * (x - x_t) + (y - y_t) * (y - y_t)) / (v_n * v_n));
        }
      } else {
        grid_data_[i].probability_ = 0.01;
      }
      grid_data_[i].uncertainty_ = 1.0;
      grid_data_[i].detect_num_ = 0;
      grid_data_[i].covered_ = false;
    } else {
      grid_data_[i].uncertainty_ = 0.0;
      grid_data_[i].detect_num_ = -1;
      grid_data_[i].covered_ = false;
    }
  }

  int valid_grids = 0;
  int no_fly_zone_grids = 0;
  for (size_t i = 0; i < grid_data_.size(); ++i) {
    Eigen::Vector3i index;
    Eigen::Vector3d position;
    addressToIndex(i, index);
    indexToPos(index, position);
    Eigen::Vector2d position_2d(position(0), position(1));

    if (grid_data_[i].detect_num_ != -1) {
      valid_grids++;
    }
    bool in_any_nofly_zone = false;
    for (const auto &zone : no_fly_zones_) {
      if (isPointInConvexPolygon(position_2d, zone)) {
        in_any_nofly_zone = true;
        break;
      }
    }
    if (isPointInConvexPolygon(position_2d, region_vertices_) && in_any_nofly_zone) {
      no_fly_zone_grids++;
    }
  }
  RCLCPP_INFO(getLogger(), "Valid grids in task region: %d / %zu (%.1f%%), no-fly zone grids excluded: %d", valid_grids,
              grid_data_.size(), 100.0 * valid_grids / grid_data_.size(), no_fly_zone_grids);
  sensor_porperty_ = std::make_shared<SensorPorperty>(node);

  // Initializeobstaclesinformation
  Obstacle obs1;
  obs1.position = Eigen::Vector3d(15.0, 15.0, 0.0);
  obs1.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  obs1.radius = 2.5;
  posToIndex(obs1.position, obs1.grid);
  obs.push_back(obs1);

  Obstacle obs2;
  obs2.position = Eigen::Vector3d(-10.0, -5.0, 0.0);
  obs2.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  obs2.radius = 2.5;
  posToIndex(obs2.position, obs2.grid);
  obs.push_back(obs2);

  Obstacle obs3;
  obs3.position = Eigen::Vector3d(5.0, 5.0, 0.0);
  obs3.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  obs3.radius = 2.5;
  posToIndex(obs3.position, obs3.grid);
  obs.push_back(obs3);

  Obstacle obs4;
  obs4.position = Eigen::Vector3d(10.0, -5.0, 0.0);
  obs4.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  obs4.radius = 2.5;
  posToIndex(obs4.position, obs4.grid);
  obs.push_back(obs4);

  Obstacle obs5;
  obs5.position = Eigen::Vector3d(-10.0, 15.0, 0.0);
  obs5.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  obs5.radius = 3.0;
  posToIndex(obs5.position, obs5.grid);
  obs.push_back(obs5);
}

void SearchEnvironment::updateGridData() {
  std::lock_guard<std::mutex> lock(grid_data_mutex);
  if (!agents_state_history_ || agents_state_history_->empty()) {
    return;
  }
  if (!targets_info_) {
    return;
  }

  std::unordered_map<int, rclcpp::Time> latest_time_stamp;

  int uav_id = agent_param_->agent_id_;
  int history_size = agent_param_->history_size_;

  rclcpp::Time self_latest_time_stamp;
  if (!(*agents_state_history_)[uav_id].empty()) {
    self_latest_time_stamp = (*agents_state_history_)[uav_id].front().time_stamp;
  } else {
    if (auto node = node_.lock()) {
      self_latest_time_stamp = node->now();
    } else {
      return;
    }
  }

  for (const auto &state : (*agents_state_history_)[uav_id]) {
    if (state.time_stamp > self_latest_time_stamp) {
      self_latest_time_stamp = state.time_stamp;
    }
  }
  for (const auto &state : (*agents_state_history_)[uav_id]) {
    if (state.time_stamp == self_latest_time_stamp) {
      Eigen::Vector2d position(state.position.x(), state.position.y());
      double theta = state.yaw;

      std::vector<Eigen::Vector2i> affected_grids = sensor_porperty_->seekerForward(position, theta);
      for (const auto &grid : affected_grids) {
        Eigen::Vector3i grid_index(grid[0], grid[1], 0);
        int grid_address = indexToAddress(grid_index);
        if (grid_address >= 0 && grid_address < grid_data_.size()) {
          grid_data_[grid_address].uncertainty_ *= 0;
          // grid_data_[grid_address].detect_num_ += 1;
          grid_data_[grid_address].covered_ = true;

          bool target_flag = false;
          for (const auto &target : *targets_info_) {
            if (target.grid[0] == grid.x() && target.grid[1] == grid.y()) {
              target_flag = true;
              // RCLCPP_INFO(getLogger(), "self_target_flag!!!!!!: %d; grid_x: %d; grid_y: %d; proba: %.4f;uncertainty:
              // %.4f; grid_address: %d; position_x: %.2f; position_y: %.2f; detect_num: %d",
              //     target_flag, grid.x(), grid.y(), grid_data_[grid_address].probability_,
              //     grid_data_[grid_address].uncertainty_, grid_address, target.position[0], target.position[1],
              //     grid_data_[grid_address].detect_num_);
              break;
            }
          }

          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_real_distribution<> dis(0.0, 1.0);
          double rand_val = dis(gen);
          if (target_flag) {
            if (rand_val < detect_param_.pd) {
              grid_data_[grid_address].probability_ = getUpdateProbability(grid_data_[grid_address].probability_,
                                                                           detect_param_.pd, detect_param_.pf, 1);
            } else {
              grid_data_[grid_address].probability_ = getUpdateProbability(grid_data_[grid_address].probability_,
                                                                           detect_param_.pd, detect_param_.pf, 0);
            }
          } else {
            if (rand_val < detect_param_.pf) {
              grid_data_[grid_address].probability_ = getUpdateProbability(grid_data_[grid_address].probability_,
                                                                           detect_param_.pd, detect_param_.pf, 1);
            } else {
              grid_data_[grid_address].probability_ = getUpdateProbability(grid_data_[grid_address].probability_,
                                                                           detect_param_.pd, detect_param_.pf, 0);
            }
          }
        }
      }
    }
  }
  for (int other_uav_id = 0; other_uav_id < agents_state_history_->size(); ++other_uav_id) {
    if (other_uav_id == uav_id)
      continue;

    rclcpp::Time &latest_time = latest_time_stamp[other_uav_id];
    if (!(*agents_state_history_)[other_uav_id].empty()) {
      latest_time = (*agents_state_history_)[other_uav_id].front().time_stamp;
    } else {
      if (auto node = node_.lock()) {
        latest_time = node->now();
      } else {
        continue;
      }
    }

    for (const auto &state : (*agents_state_history_)[other_uav_id]) {
      if (state.time_stamp > latest_time) {
        latest_time = state.time_stamp;

        Eigen::Vector2d position(state.position.x(), state.position.y());
        double theta = state.yaw;

        std::vector<Eigen::Vector2i> affected_grids = sensor_porperty_->seekerForward(position, theta);

        for (const auto &grid : affected_grids) {
          Eigen::Vector3i grid_index(grid[0], grid[1], 0);
          int grid_address = indexToAddress(grid_index);
          if (grid_address >= 0 && grid_address < grid_data_.size()) {
            grid_data_[grid_address].uncertainty_ *= 0;
            // grid_data_[grid_address].detect_num_ += 1;
            grid_data_[grid_address].covered_ = true;

            bool target_flag = false;
            for (const auto &target : *targets_info_) {
              if (target.grid[0] == grid.x() && target.grid[1] == grid.y()) {
                target_flag = true;
                // RCLCPP_INFO(getLogger(), "self_target_flag!!!!!!: %d; grid_x: %d; grid_y: %d; proba:
                // %.4f;uncertainty: %.4f; grid_address: %d; position_x: %.2f; position_y: %.2f; detect_num: %d",
                //     target_flag, grid.x(), grid.y(), grid_data_[grid_address].probability_,
                //     grid_data_[grid_address].uncertainty_, grid_address, target.position[0], target.position[1],
                //     grid_data_[grid_address].detect_num_);
                break;
              }
            }
            // grid_data_[grid_address].probability_ = std::max(grid_data_[grid_address].probability_ - 0.1, 0.0);
            grid_data_[grid_address].probability_ = getUpdateProbability(grid_data_[grid_address].probability_,
                                                                         detect_param_.pd, detect_param_.pf,
                                                                         target_flag);
          }
        }
      }
    }
  }
}

void SearchEnvironment::printGridData() {
  std::lock_guard<std::mutex> lock(grid_data_mutex);
  double total_uncertainty = 0.0;
  double total_probability = 0.0;
  static double uncertainty_max_range = 0.0;
  static double probability_max_range = 0.0;
  static bool set_coordinate_range = false;
  for (size_t i = 0; i < grid_data_.size(); i++) {
    if (grid_data_[i].detect_num_ != -1) {
      total_uncertainty += grid_data_[i].uncertainty_;
      total_probability += grid_data_[i].probability_;
    }
  }

  if (!set_coordinate_range) {
    uncertainty_max_range = total_uncertainty;
    probability_max_range = total_probability;
    set_coordinate_range = true;
  }

  std_msgs::msg::Float64 uncertainty_msg;
  std_msgs::msg::Float64 probability_msg;
  uncertainty_msg.data = total_uncertainty;
  probability_msg.data = total_probability;
  if (uncertainty_pub_) {
    uncertainty_pub_->publish(uncertainty_msg);
  }
  if (probability_pub_) {
    probability_pub_->publish(probability_msg);
  }

  uncertainty_history_.push_back(total_uncertainty);
  probability_history_.push_back(total_probability);

  if (!plot_on_all_uavs) {
    if (agent_param_->agent_id_ != 0) {
      return;
    }
  }
  // ROS_INFO_STREAM("Total Uncertainty:" << total_uncertainty << ", Total Probability:" << total_probability);
}

std::vector<int> SearchEnvironment::getGridInPredictState(const std::vector<PredictState> &perdict_state) {
  std::set<int> unique_grid_ids;

  for (const auto &state : perdict_state) {
    Eigen::Vector2d position(state.position.x(), state.position.y());
    double yaw = state.yaw;

    std::vector<Eigen::Vector2i> affected_grids = sensor_porperty_->seekerForward(position, yaw);

    for (const auto &grid_coord : affected_grids) {
      if (grid_coord[0] >= 0 && grid_coord[0] < map_param_.map_voxel_num_(0) && grid_coord[1] >= 0 &&
          grid_coord[1] < map_param_.map_voxel_num_(1)) {
        Eigen::Vector3i id;
        id << grid_coord[0], grid_coord[1], 0;
        int index = indexToAddress(id);
        unique_grid_ids.insert(index);
      }
    }
  }
  std::vector<int> result(unique_grid_ids.begin(), unique_grid_ids.end());
  return result;
}

int SearchEnvironment::getMaxGridsPerFOVStep() const {
  if (!sensor_porperty_) {
    return 0;
  }
  const double resolution = sensor_porperty_->getlength();
  if (resolution <= 0.0) {
    return 0;
  }
  const int length_cells = static_cast<int>(std::floor(sensor_porperty_->getCoverLength() / resolution)) + 1;
  const int width_cells = static_cast<int>(std::floor(sensor_porperty_->getCoverWidth() / resolution)) + 1;
  return std::max(0, length_cells * width_cells);
}

std::pair<double, double> SearchEnvironment::calculateSearchBenefit(std::vector<int> grid_id) {
  double total_probability = 0.0;
  double total_uncertainty = 0.0;

  const auto &grid_data = grid_data_;
  size_t grid_data_size = grid_data.size();

  for (int id : grid_id) {
    if (id >= 0 && id < grid_data_size && grid_data[id].detect_num_ != -1) {
      total_probability += grid_data[id].probability_;
      total_uncertainty += grid_data[id].uncertainty_;
    }
  }

  return std::make_pair(total_probability, total_uncertainty);
}

void SearchEnvironment::setOtherAgentsCoveredGrid() {
  if (!agents_state_history_ || !sensor_porperty_ || !agent_param_) {
    RCLCPP_WARN(getLogger(), "SearchEnvironment::setOtherAgentsCoveredGrid: Missing pointers.");
    return;
  }

  int self_id = agent_param_->agent_id_;
  std::unordered_set<int> covered_grid_ids;

  for (size_t agent_idx = 0; agent_idx < agents_state_history_->size(); ++agent_idx) {
    if (static_cast<int>(agent_idx) == self_id)
      continue;

    const auto &history = agents_state_history_->at(agent_idx);
    if (history.empty())
      continue;

    for (const auto &state : history) {
      Eigen::Vector2d position(state.position.x(), state.position.y());
      double yaw = state.yaw;

      std::vector<Eigen::Vector2i> fov_grids = sensor_porperty_->seekerForward(position, yaw);

      for (const auto &grid_coord : fov_grids) {
        Eigen::Vector3i id;
        id << grid_coord[0], grid_coord[1], 0;

        int index = indexToAddress(id);
        covered_grid_ids.insert(index);
      }
    }
  }

  for (int index : covered_grid_ids) {
    if (index >= 0 && index < static_cast<int>(grid_data_.size())) {
      grid_data_[index].detect_num_ = -1;
    }
  }
}

void SearchEnvironment::setHistoryFilePath(const std::string &filepath) {
  if (history_file_stream_.is_open()) {
    history_file_stream_.close();
  }

  history_file_path_ = filepath;
  history_file_stream_.open(history_file_path_, std::ios::out | std::ios::trunc);

  if (!history_file_stream_.is_open()) {
    RCLCPP_ERROR(getLogger(), "Failed to open history file: %s", history_file_path_.c_str());
    return;
  }

  history_file_stream_ << "# Grid Data History Export\n";
  history_file_stream_ << "# Format: snapshot_index timestamp(ns) grid_index uncertainty probability detect_num\n";
  history_file_stream_ << "# Total grids per snapshot: " << grid_data_.size() << "\n\n";
  history_file_stream_.flush();

  RCLCPP_INFO(getLogger(), "Grid history file opened: %s", history_file_path_.c_str());
}

void SearchEnvironment::closeHistoryFile() {
  if (history_file_stream_.is_open()) {
    history_file_stream_.close();
    RCLCPP_INFO(getLogger(), "Grid history file closed: %s", history_file_path_.c_str());
  }
}

void SearchEnvironment::exportGridHistory(const std::string &filename) {
  setHistoryFilePath(filename);
}

// Set the recording interval（seconds）
void SearchEnvironment::setRecordInterval(double interval_sec) {
  record_interval_sec_ = interval_sec;
  RCLCPP_INFO(getLogger(), "History record interval set to %.2f seconds", interval_sec);
}

void SearchEnvironment::historyTimerCallback() {
  std::lock_guard<std::mutex> lock(grid_data_mutex);
  if (auto node = node_.lock()) {
    GridSnapshotData snapshot;
    snapshot.timestamp = node->now();
    snapshot.grid_data = grid_data_;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (snapshot_queue_.size() >= max_queue_size_) {
        snapshot_queue_.pop_front();
      }
      snapshot_queue_.push_back(std::move(snapshot));
    }
    queue_cv_.notify_one();
  }
}

// Start the history recording thread
void SearchEnvironment::startHistoryThread() {
  if (running_) {
    return;
  }

  // Create the timer
  auto node = node_.lock();
  if (node) {
    history_timer_ = node->create_wall_timer(std::chrono::duration<double>(record_interval_sec_),
                                             [this]() { this->historyTimerCallback(); });
    RCLCPP_INFO(getLogger(), "History timer created with interval %.2f seconds", record_interval_sec_);
  }

  running_ = true;
  history_thread_ = std::thread(&SearchEnvironment::historyRecordingThread, this);
  RCLCPP_INFO(getLogger(), "History recording thread started");
}

// Stop the history recording thread
void SearchEnvironment::stopHistoryThread() {
  if (!running_) {
    return;
  }
  running_ = false;

  history_timer_.reset();

  queue_cv_.notify_all();
  if (history_thread_.joinable()) {
    history_thread_.join();
  }
  RCLCPP_INFO(getLogger(), "History recording thread stopped");
}

void SearchEnvironment::historyRecordingThread() {
  while (running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait_for(lock, std::chrono::seconds(1), [this] { return !snapshot_queue_.empty() || !running_; });

    while (!snapshot_queue_.empty() && running_) {
      GridSnapshotData snapshot = std::move(snapshot_queue_.front());
      snapshot_queue_.pop_front();
      lock.unlock();

      if (history_file_stream_.is_open()) {
        history_file_stream_ << "# Snapshot " << snapshot_count_ << " time " << snapshot.timestamp.nanoseconds()
                             << "\n";
        for (size_t i = 0; i < snapshot.grid_data.size(); ++i) {
          if (snapshot.grid_data[i].detect_num_ != -1) {
            history_file_stream_ << snapshot_count_ << " " << snapshot.timestamp.nanoseconds() << " " << i << " "
                                 << snapshot.grid_data[i].uncertainty_ << " " << snapshot.grid_data[i].probability_
                                 << " " << snapshot.grid_data[i].detect_num_ << "\n";
          }
        }
        history_file_stream_.flush();
        snapshot_count_++;
      }
      lock.lock();
    }
  }
}
