#ifndef SEARCH_ENVIRONMENT_H
#define SEARCH_ENVIRONMENT_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <unordered_map>
#include <vector>
#include "SensorPorperty.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
inline bool isPointInConvexPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices) {
  if (vertices.size() < 3)
    return false;

  int n = vertices.size();
  bool sign = false;
  bool first = true;

  for (int i = 0; i < n; i++) {
    const Eigen::Vector2d &v1 = vertices[i];
    const Eigen::Vector2d &v2 = vertices[(i + 1) % n];

    // Compute the cross product
    double cross = (v2.x() - v1.x()) * (point.y() - v1.y()) - (v2.y() - v1.y()) * (point.x() - v1.x());

    if (first) {
      sign = cross > 0;
      first = false;
    } else if ((cross > 0) != sign && std::abs(cross) > 1e-10) {
      return false;
    }
  }
  return true;
}

inline std::vector<Eigen::Vector2d> transformRegionVertices(std::vector<double> &region_vertices_raw) {
  if (region_vertices_raw.size() % 2 != 0 || region_vertices_raw.size() < 6) {
    std::cout << "region_vertices invalid, expect even number >=6, got " << region_vertices_raw.size() << std::endl;
    return std::vector<Eigen::Vector2d>();
  } else {
    std::vector<Eigen::Vector2d> region_vertices_;
    region_vertices_.clear();
    region_vertices_.reserve(region_vertices_raw.size() / 2);
    for (size_t i = 0; i + 1 < region_vertices_raw.size(); i += 2) {
      region_vertices_.emplace_back(region_vertices_raw[i], region_vertices_raw[i + 1]);
    }
    return region_vertices_;
  }
}

#ifndef GRIDDATA_STRUCT_DEFINED
#define GRIDDATA_STRUCT_DEFINED
struct GridData {
  double uncertainty_;
  double probability_;
  int detect_num_;
  bool covered_;
};
#endif

struct GridSnapshotData {
  rclcpp::Time timestamp;
  std::vector<GridData> grid_data;
};

struct AgentState {
  Eigen::Vector3d position;
  Eigen::Vector3i grid;
  Eigen::Vector3d velocity;
  double yaw;
  rclcpp::Time time_stamp;
};

struct Target {
  Eigen::Vector3d position;
  Eigen::Vector3i grid;
  rclcpp::Time time_stamp;
};

struct AgentParam {
  int agent_task_;
  int agent_id_;
  int agent_num_;
  int history_size_;
  double safe_uav_;
  double safe_obs_;
  double apf_distance_;
  double apf_param1_, apf_param2_;  // artificial potential field parameters1、2
  int predict_horizon_;             // prediction horizon
  double w_jA1, w_jA2;
  double w_jB;
  double w_jC;
  double w_jD;
  double safe_margin_x_;  // safety boundary
  double safe_margin_y_;
  double k_wJA1 = 1.0;              // jA1weight adjustment factor
  double k_wJA2 = 1.0;              // jA2weight adjustment factor
  double k_wJB = 0.0;               // jBweight adjustment factor
  double k_wJC = 1.0;               // collision weight factor
  double k_wJD = 1.0;               // initial task-assignment weight factor
  double max_dt = 2.0;              // maximum time step
  double velocity_threshold = 2.0;  // search jump step
  double search_jump = 1.0;         // search jump step
  bool use_expert_system_;
  double dis_tar_warnning_;
};

struct PredictState {
  Eigen::Vector3d position;
  double yaw;
};

// Map parameters
struct MapParam {
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double grid_size;
};

// Sensor parameters
struct DetectionParam {
  int type;
  double pd;
  double pf;
};

// Obstacle data
struct Obstacle {
  Eigen::Vector3d position;  // obstacle center position
  Eigen::Vector3i grid;      // obstacle center grid coordinate
  Eigen::Vector3d velocity;  // obstacle center velocity
  double radius;             // obstacle radius
};

class SearchEnvironment {
 public:
  SearchEnvironment();
  ~SearchEnvironment();

  void initEnv(rclcpp::Node::SharedPtr node);
  inline int indexToAddress(const Eigen::Vector3i &id) {
    return id(1) * map_param_.map_voxel_num_(0) + id(0);
  }
  inline void addressToIndex(int i, Eigen::Vector3i &id) {
    id[0] = i % map_param_.map_voxel_num_(0);
    id[1] = i / map_param_.map_voxel_num_(0);
    id[2] = 0;
  }

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - map_param_.map_min_boundary_(i)) * map_param_.resolution_inv_);
  }
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) {
    for (int i = 0; i < 3; ++i)
      pos(i) = id(i) * map_param_.resolution_ + map_param_.map_min_boundary_(i);
  }
  void updateGridData();
  inline double getUpdateProbability(double proba, double pd, double pf, bool target_flag) {
    double numerator, denominator;
    if (target_flag) {
      numerator = pd * proba;
      denominator = numerator + pf * (1 - proba);
    } else {
      numerator = (1 - pd) * proba;
      denominator = numerator + (1 - pf) * (1 - proba);
    }
    return (denominator > 0) ? (numerator / denominator) : 0.0;
  }
  void setAgentsStateHistory(std::vector<std::deque<AgentState>> *agents_history) {
    agents_state_history_ = agents_history;
  }

  void setTargetsInfo(std::vector<Target> *targets) {
    targets_info_ = targets;
  }

  const std::vector<Target> &getTargetsInfoRef() const {
    if (!targets_info_) {
      static std::vector<Target> empty_targets;
      return empty_targets;
    }
    return *targets_info_;
  }

  void setAgentParam(AgentParam *agent_param) {
    agent_param_ = agent_param;
  }
  void setNoFlyZones(const std::vector<std::vector<Eigen::Vector2d>> &zones) {
    no_fly_zones_ = zones;
  }
  void setNoFlyZoneVertices(const std::vector<Eigen::Vector2d> &vertices) {
    no_fly_zones_.clear();
    if (!vertices.empty()) {
      no_fly_zones_.push_back(vertices);
    }
  }
  void printGridData();
  // Getgrid_data_data
  const std::vector<GridData> &getGridData() const {
    return grid_data_;
  }
  std::vector<int> getGridInPredictState(const std::vector<PredictState> &perdict_state_);
  int getMaxGridsPerFOVStep() const;
  std::pair<double, double> calculateSearchBenefit(std::vector<int> grid_id);
  MapParam map_param_;        // Map parameters
  std::vector<Obstacle> obs;  // obstacles
  void setOtherAgentsCoveredGrid();
  std::vector<GridData> grid_data_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr uncertainty_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr probability_pub_;

  void historyRecordingThread();
  void startHistoryThread();  // Start the recording thread
  void stopHistoryThread();
  void exportGridHistory(const std::string &filename);
  void closeHistoryFile();                               // Start the recording thread
  void setHistoryFilePath(const std::string &filepath);  // Set the history file path
  void setRecordInterval(double interval_sec);           // Set the recording interval（seconds）

 private:
  std::mutex grid_data_mutex;
  rclcpp::Node::WeakPtr node_;
  DetectionParam detect_param_;
  AgentParam *agent_param_ = nullptr;
  std::vector<Eigen::Vector2d> region_vertices_;
  std::vector<std::vector<Eigen::Vector2d>> no_fly_zones_;  // multiple no-fly zones

  // std::vector<Obstacle> obstacle_data_;
  // std::shared_ptr<double> search_benefit_;
  std::vector<std::deque<AgentState>> *agents_state_history_ = nullptr;
  std::vector<Target> *targets_info_ = nullptr;
  std::shared_ptr<SensorPorperty> sensor_porperty_;
  std::vector<double> uncertainty_history_;
  std::vector<double> probability_history_;

  std::string history_file_path_;
  std::ofstream history_file_stream_;
  double record_interval_sec_ = 30.0;  // recording interval（seconds）
  rclcpp::TimerBase::SharedPtr history_timer_;
  void historyTimerCallback();  // timer callback
  int snapshot_count_ = 0;

  std::deque<GridSnapshotData> snapshot_queue_;  // snapshot queue
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;  // condition variable
  std::thread history_thread_;
  bool running_ = false;
  int max_queue_size_ = 100;

  bool show_search_result;
  double search_time;
  bool plot_on_all_uavs;
  double plot_interval;
  rclcpp::Logger getLogger() const;
};

#endif  // SEARCH_ENVIRONMENT_H
