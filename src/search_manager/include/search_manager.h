#ifndef SEARCH_MANAGER_H
#define SEARCH_MANAGER_H

#include <search_env/search_environment.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <atomic>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include "custom_msgs/msg/communication_data.hpp"
#include "custom_msgs/msg/object_computation.hpp"
#include "dmpc/dmpc.h"
#include "event_trigger.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SearchManager {
 public:
  SearchManager();
  ~SearchManager();
  void init(rclcpp::Node::SharedPtr &nh);
  void agentMsgCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg);
  void agentOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void agentMsgPublish();
  void gridDataUpdate();
  void printAgentStateHistory();
  void envGridDataPrint();
  const std::vector<GridData> &envGridDataGet();
  void executeDMPCDecision(bool use_next_target, const PredictState *next_target);
  std::vector<PredictState> latestPredictStateGet();
  const std::vector<std::deque<AgentState>> &agentsStateHistoryGet() const;
  const std::vector<Obstacle> &getObstacles() const;
  void targetMsgCallback(const custom_msgs::msg::ObjectComputation::SharedPtr msg);
  bool getUseInitAsTakeoffPosition() const;
  bool isTriggerDmpc(int index);
  int eventTriggerWindowSize() const {
    return event_trigger_window_size_;
  }
  int eventTriggerExecutionStride() const {
    return event_trigger_execution_stride_;
  }
  void resetEventTrigger();
  void recordForcedReplan();
  bool isOdomReceived() const {
    return odom_received_;
  }
  bool isReceivedAgentData() const;
  std::vector<std::vector<double>> parseNoFlyZonesString(const std::string &zones_str);

  void updateDMPCRealtimePosition(int agent_id, const Eigen::Vector3d &position, double yaw);
  std::shared_ptr<SearchEnvironment> getEnv() {
    return env;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<custom_msgs::msg::CommunicationData>::SharedPtr agent_msg_pub_;
  std::vector<rclcpp::Subscription<custom_msgs::msg::CommunicationData>::SharedPtr> agent_msg_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<custom_msgs::msg::ObjectComputation>::SharedPtr target_sub_;
  std::atomic<bool> communication_running_{false};
  std::thread communication_thread_;
  mutable std::mutex state_mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_plan_pub_;
  std::shared_ptr<SearchEnvironment> env;
  std::unique_ptr<DMPC> dmpc;
  std::vector<int> history_index_;
  std::vector<int> comm_history_index_;
  std::vector<std::deque<AgentState>> agents_state_history_;
  std::vector<Target> targets_info_;
  AgentParam agent_param_;
  std::vector<PredictState> latest_predict_state_;
  mutable std::mutex latest_predict_state_mutex_;
  bool use_init_as_takeoff_position;
  double init_x_, init_y_, init_z_;
  rclcpp::Time last_odom_msg_time;
  bool odom_received_ = false;
  std::map<int, rclcpp::Time> last_agent_msg_times_;
  int event_trigger_window_size_ = 3;
  int event_trigger_execution_stride_ = 3;
  std::unique_ptr<EventTrigger> event_trigger_;
  EventTriggerMetrics event_metrics_;
  std::ofstream event_metrics_file_;
  std::mutex event_metrics_file_mutex_;
  std::mutex mission_metrics_mutex_;
  rclcpp::Time metrics_start_time_;
  bool metrics_started_ = false;
  double total_distance_ = 0.0;
  std::optional<Eigen::Vector3d> previous_odom_position_;
  double target_arrival_distance_ = 20.0;
  int target_required_consecutive_samples_ = 3;
  int target_consecutive_samples_ = 0;

  void writeEventMetrics(const EventTriggerResult &result, const char *reason);
  void updateTargetCompletion(const Eigen::Vector3d &odom_position);
};

#endif  // SEARCH_MANAGER_H
