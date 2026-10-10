#ifndef SEARCH_NODE_H
#define SEARCH_NODE_H

#include <Eigen/Eigen>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <vector>
#include "custom_msgs/msg/predict_trajectory.hpp"
#include "mavros_controller.h"
#include "search_env/search_environment.h"
class SearchManager;
class PredictState;

std::unique_ptr<MavrosController> mavros_controller_;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub;
rclcpp::Publisher<custom_msgs::msg::PredictTrajectory>::SharedPtr predict_trajectory_pub_;
double init_z = 1.0;
double init_x, init_y;
bool send_task_start = false;

enum class Phase { TO_INITIAL_POSITION, PLANNING_AND_TRACKING, RETURN_TO_INITIAL_POSITION, LANDING };

struct TaskParam {
  int max_trajectory_iterations;
  double rhoMin;
  int path_points_per_execution;
  int fixed_period_execution_points;
  double ga_start_time;
  double distance_to_next_target;
  double max_task_time;
  bool use_event_trigger;
  rclcpp::Time task_start_time;
};

struct InitMapParam {
  double map_x;
  double map_y;
};

class TrajectoryController {
 private:
  SearchManager *manager_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub_;
  int agent_id_;
  TaskParam task_param_;
  InitMapParam map_param_;

  Phase phase_;
  bool need_new_plan_;
  size_t path_index_;  // currentpathindex
  double rhoMin_;
  std::vector<PredictState> current_trajectory_;
  size_t trajectory_index_;
  rclcpp::Time last_plan_time_;
  double trajectory_progress_;
  size_t current_segment_index_;
  double current_segment_initial_distance_;

  std::atomic<bool> ga_calculate_in_progress_;
  std::thread ga_thread_;
  std::vector<PredictState> next_trajectory_;
  std::atomic<bool> has_next_trajectory_;
  std::mutex trajectory_mutex_;
  int trajectory_count_;
  size_t last_event_check_index_;  ///< Last trajectory index evaluated by the event trigger
  double init_x_;
  double init_y_;

  rclcpp::Node::SharedPtr node_;  // ROSNode pointer

  bool is_new_plan_required_ = false;

  const int &max_trajectory_iterations;
  const double &rhoMin;
  const int &path_points_per_execution;
  const int &fixed_period_execution_points;
  const double &ga_start_time;
  const double &distance_to_next_target;
  const bool &use_event_trigger;

 public:
  TrajectoryController(rclcpp::Node::SharedPtr node, SearchManager *manager,
                       rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub, int agent_id,
                       TaskParam &task_param, InitMapParam &map_param, double init_x, double init_y)
      : node_(node),
        manager_(manager),
        cmd_pub_(cmd_pub),
        agent_id_(agent_id),
        task_param_(task_param),
        map_param_(map_param),
        max_trajectory_iterations(task_param.max_trajectory_iterations),
        rhoMin(task_param.rhoMin),
        path_points_per_execution(task_param.path_points_per_execution),
        fixed_period_execution_points(task_param.fixed_period_execution_points),
        use_event_trigger(task_param.use_event_trigger),
        ga_start_time(task_param.ga_start_time),
        distance_to_next_target(task_param.distance_to_next_target),
        phase_(Phase::TO_INITIAL_POSITION),
        need_new_plan_(true),
        path_index_(0),  // pathindex
        rhoMin_(1.0),
        trajectory_index_(0),
        last_plan_time_(rclcpp::Clock().now()),
        trajectory_progress_(0.0),
        current_segment_index_(0),
        current_segment_initial_distance_(0.0),
        ga_calculate_in_progress_(false),
        has_next_trajectory_(false),
        trajectory_count_(0),
        last_event_check_index_(std::numeric_limits<size_t>::max()),
        init_x_(init_x),
        init_y_(init_y) {
    current_trajectory_.reserve(15);
    next_trajectory_.reserve(15);
    resetSegmentProgress();
  }

  void executeLoop(double init_x, double init_y, double init_z);

  ~TrajectoryController();

 private:
  void handleInitialPositionPhase(const Eigen::Vector3d &cur_pos, double init_x, double init_y, double init_z);
  void handlePlanningAndTrackingPhase(const Eigen::Vector3d &cur_pos, double cur_yaw, double init_z);
  void handleReturnToInitialPositionPhase(const Eigen::Vector3d &cur_pos, double cur_yaw, double init_x, double init_y,
                                          double init_z);
  void executeTrajectoryTracking(const Eigen::Vector3d &cur_pos, double cur_yaw, double init_z);
  void handleGAThreadCompletion();
  void executeDMPCDecision();
  void startPredictiveGACalculation();
  void publishPositionCommand(const PredictState &target, double init_z);
  void resetSegmentProgress();
};

bool getLatestAgentPosition(const SearchManager &manager, int agent_id, Eigen::Vector3d &position_out, double &yaw_out);
void controlExecutionLoop(rclcpp::Node::SharedPtr node, SearchManager *manager,
                          rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub, double init_x,
                          double init_y, double init_z, int agent_id, TaskParam &task_param_, InitMapParam &map_param_);
bool getLatestAgentPosition(const SearchManager &manager, int agent_id, Eigen::Vector3d &position_out, double &yaw_out);
bool isPathSafe(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const std::vector<Obstacle> &obstacles);
void gridDataUpdateCallback(SearchManager *manager);
bool isOutOfTaskArea(const Eigen::Vector3d &cur_pos, double map_x, double map_y);
bool isReachMaxTaskTime(const rclcpp::Time &start_time, double max_task_time);

#endif
