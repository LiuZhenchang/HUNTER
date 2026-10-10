#include "search_node.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include "GridData_visualization.h"
#include "search_manager.h"

#include <atomic>
#include <mutex>

bool getLatestAgentPosition(const SearchManager &manager, int agent_id, Eigen::Vector3d &position_out,
                            double &yaw_out) {
  const auto &history = manager.agentsStateHistoryGet();
  for (size_t i = 0; i < history.size(); ++i) {
    const auto &agent_traj = history[i];
    if (!agent_traj.empty()) {
      const AgentState &state = agent_traj.back();
      // RCLCPP_INFO(manager.getNode()->get_logger(), "  Latest position = " << state.position.transpose() << ", yaw = "
      // << state.yaw);
    }

    for (size_t t = 0; t < agent_traj.size(); ++t) {
      const AgentState &state = agent_traj[t];
    }
  }

  if (agent_id < 0 || agent_id >= static_cast<int>(history.size())) {
    RCLCPP_WARN(rclcpp::get_logger("search_node"), "Invalid agent_id: %d", agent_id);
    return false;
  }

  const auto &agent_history = history[agent_id];
  if (agent_history.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("search_node"), "Agent %d has empty state history.", agent_id);
    return false;
  }

  const AgentState &latest_state = agent_history.back();
  position_out = latest_state.position;
  yaw_out = latest_state.yaw;
  return true;
}

bool isPathSafe(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const std::vector<Obstacle> &obstacles) {
  // Check whether the path is safe
  const int num_samples = 50;
  for (int i = 0; i < num_samples; i++) {
    double t = static_cast<double>(i) / num_samples;
    Eigen::Vector2d sample_point = start + t * (end - start);

    for (const auto &obs : obstacles) {
      double dx = sample_point.x() - obs.position.x();
      double dy = sample_point.y() - obs.position.y();
      double d = std::sqrt(dx * dx + dy * dy);

      if (d <= (obs.radius + 0.5)) {
        RCLCPP_ERROR(rclcpp::get_logger("search_node"), "Path is unsafe, distance: %f", d);
        return false;
      }
    }
  }
  return true;
}

TrajectoryController::~TrajectoryController() {
  if (ga_calculate_in_progress_ && ga_thread_.joinable()) {
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "Waiting for GA thread to complete before destruction...");
    ga_thread_.join();
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "GA thread completed successfully");
  }

  ga_calculate_in_progress_ = false;
  has_next_trajectory_ = false;

  RCLCPP_INFO(rclcpp::get_logger("search_node"), "TrajectoryController destroyed safely");
}

void TrajectoryController::handleInitialPositionPhase(const Eigen::Vector3d &cur_pos, double init_x, double init_y,
                                                      double init_z) {
  double dx, dy, dist;
  if (manager_->getUseInitAsTakeoffPosition()) {
    dist = std::sqrt(dx * dx + dy * dy);
  } else {
    dx = cur_pos.x() - init_x;
    dy = cur_pos.y() - init_y;
    dist = std::sqrt(dx * dx + dy * dy);
  }

  //     RCLCPP_INFO(rclcpp::get_logger("search_node"), "Reached initial position (%.2f, %.2f), start DMPC planning",
  //     init_x, init_y);
  phase_ = Phase::PLANNING_AND_TRACKING;
  need_new_plan_ = true;
  //     mavros_msgs::CommandLong fw_mode_cmd;
  //     fw_mode_cmd.request.broadcast = false;
  //     fw_mode_cmd.request.command = 3000;
  //     fw_mode_cmd.request.confirmation = 0;
  //     fw_mode_cmd.request.param1 = 4.0f;

  //     if(change_fw_mode_client.call(fw_mode_cmd) && fw_mode_cmd.response.success){
  //         is_fw_mode_ = true;
  //         phase_ = Phase::PLANNING_AND_TRACKING;
  //         RCLCPP_INFO(rclcpp::get_logger("search_node"), "switch to fw mode");
  //     }else{
  //         RCLCPP_INFO(rclcpp::get_logger("search_node"), "switch to fw mode failed");
  //     }
  //     return;
  // }

  // if (!mavros_controller_->isFwMode()) {
  //     need_new_plan_ = true;
  //     mavros_controller_->changeFwMode();
  //     if(mavros_controller_->isFwMode()){
  //         phase_ = Phase::PLANNING_AND_TRACKING;
  //     }
  //     return;
  // }

  quadrotor_msgs::msg::PositionCommand cmd;
  cmd.position.x = init_x;
  cmd.position.y = init_y;
  cmd.position.z = init_z;
  cmd.yaw = 0.0;
  cmd.header.stamp = rclcpp::Clock().now();
  cmd_pub_->publish(cmd);

  if (mavros_controller_) {
    Eigen::Vector3d position(0, 0, init_z);
    mavros_controller_->publishPositionNoVelocity_NoYaw(position);
  }
}

bool isOutOfTaskArea(const Eigen::Vector3d &cur_pos, double map_x, double map_y) {
  if (cur_pos.x() < -map_x / 2 || cur_pos.x() > map_x / 2 || cur_pos.y() < -map_y / 2 || cur_pos.y() > map_y / 2) {
    return true;
  }
  return false;
}

bool isReachMaxTaskTime(const rclcpp::Time &start_time, double max_task_time) {
  rclcpp::Time now = rclcpp::Clock().now();
  double duration = (now - start_time).seconds();
  if (duration > max_task_time) {
    return true;
  }
  return false;
}

void TrajectoryController::handlePlanningAndTrackingPhase(const Eigen::Vector3d &cur_pos, double cur_yaw,
                                                          double init_z) {
  if (ga_calculate_in_progress_ && ga_thread_.joinable() && has_next_trajectory_ && trajectory_progress_ >= 0.999 &&
      !need_new_plan_) {
    handleGAThreadCompletion();
  }

  if (need_new_plan_ && manager_->isOdomReceived()) {
    executeDMPCDecision();
  }

  executeTrajectoryTracking(cur_pos, cur_yaw, init_z);
}

void TrajectoryController::executeTrajectoryTracking(const Eigen::Vector3d &cur_pos, double cur_yaw, double init_z) {
  if (current_trajectory_.empty()) {
    trajectory_progress_ = 1.0;
    return;
  }

  static bool is_published = false;
  if (!is_published) {
    {
      custom_msgs::msg::PredictTrajectory predict_trajectory;
      custom_msgs::msg::PredictState predict_state;
      predict_trajectory.agent_id = agent_id_ + 1;
      predict_trajectory.time = rclcpp::Clock().now();
      for (size_t i = 0; i < current_trajectory_.size(); ++i) {
        Eigen::Vector3d global_pos = current_trajectory_[i].position - Eigen::Vector3d(init_x_, init_y_, 0);
        predict_state.y = global_pos.x();
        predict_state.x = global_pos.y();
        predict_state.z = -global_pos.z();
        predict_state.yaw = current_trajectory_[i].yaw;
        predict_trajectory.waypoints.push_back(predict_state);
      }
      predict_trajectory_pub_->publish(predict_trajectory);
      std::cout << "Predict trajectory has been published" << std::endl;
    }
    is_published = true;
  }

  // Evaluate once after each execution interval and keep following the old trajectory when false.
  const size_t event_stride = static_cast<size_t>(manager_->eventTriggerExecutionStride());
  const size_t event_window = static_cast<size_t>(manager_->eventTriggerWindowSize());
  if (use_event_trigger && trajectory_index_ < current_trajectory_.size() && trajectory_index_ % event_stride == 0 &&
      trajectory_index_ != last_event_check_index_) {
    last_event_check_index_ = trajectory_index_;
    if (manager_->isTriggerDmpc(static_cast<int>(trajectory_index_))) {
      current_trajectory_.resize(std::min(trajectory_index_ + event_window, current_trajectory_.size()));
      resetSegmentProgress();
      trajectory_progress_ = static_cast<double>(trajectory_index_) / static_cast<double>(current_trajectory_.size());
      is_new_plan_required_ = true;
      startPredictiveGACalculation();
      return;
    }
  }

  if (trajectory_index_ < current_trajectory_.size()) {
    PredictState target = current_trajectory_[trajectory_index_];

    // Eigen::Vector3d cur_pos;
    // cur_pos = mavros_controller_->getCurrentPosition();

    Eigen::Vector3d target_pos_local = target.position;
    double dx, dy, dist;
    if (manager_->getUseInitAsTakeoffPosition()) {
      target.position = target_pos_local - Eigen::Vector3d(init_x_, init_y_, 0);
      dx = cur_pos.x() - target_pos_local.x();
      dy = cur_pos.y() - target_pos_local.y();
      dist = std::sqrt(dx * dx + dy * dy);
    } else {
      dx = cur_pos.x() - target.position.x();
      dy = cur_pos.y() - target.position.y();
      dist = std::sqrt(dx * dx + dy * dy);
    }

    if (current_segment_index_ != trajectory_index_) {
      current_segment_index_ = trajectory_index_;
      current_segment_initial_distance_ = std::max(dist, distance_to_next_target);
    }

    double seg_progress = 0.0;
    if (current_segment_initial_distance_ > 1e-6) {
      seg_progress = std::clamp((current_segment_initial_distance_ - dist) / current_segment_initial_distance_, 0.0,
                                1.0);
    }

    size_t total_waypoints = std::max<size_t>(1, current_trajectory_.size());
    trajectory_progress_ = std::clamp((static_cast<double>(trajectory_index_) + seg_progress) /
                                          static_cast<double>(total_waypoints),
                                      0.0, 1.0);

    if (dist < distance_to_next_target) {
      RCLCPP_INFO(rclcpp::get_logger("search_node"), "Agent %d: Reached waypoint %zu/%zu (dist=%.2f < threshold=%.2f)",
                  agent_id_, trajectory_index_ + 1, current_trajectory_.size(), dist, distance_to_next_target);
      trajectory_index_++;

      if (trajectory_index_ + 1 >= current_trajectory_.size()) {
        trajectory_count_++;
        RCLCPP_INFO(rclcpp::get_logger("search_node"), "Trajectory generation count: %d", trajectory_count_);
        // if(trajectory_count_ >= max_trajectory_iterations) {
        //     phase_ = Phase::RETURN_TO_INITIAL_POSITION;
        // }
      }

      if (current_trajectory_.size() == 1 && trajectory_index_ >= current_trajectory_.size()) {
        return;
      }
    }

    if (!use_event_trigger && !ga_calculate_in_progress_ && !has_next_trajectory_) {
      if (trajectory_progress_ >= ga_start_time) {
        startPredictiveGACalculation();
        is_new_plan_required_ = false;
      }
    }

    PredictState extended_target = target;

    if (dist > distance_to_next_target) {
      Eigen::Vector3d dir = target_pos_local - cur_pos;
      double dir_norm = dir.norm();
      // RCLCPP_INFO(rclcpp::get_logger("search_node"), "target.position: (%f, %f, %f), cur_pos: (%f, %f, %f), dir_norm:
      // %f", target.position.x(), target.position.y(), target.position.z(), cur_pos.x(), cur_pos.y(), cur_pos.z(),
      // dir_norm);
      if (dir_norm > 1e-6) {
        dir.normalize();
        extended_target.position = target.position + dir * 10.0;
        extended_target.yaw = target.yaw;
        // RCLCPP_INFO(rclcpp::get_logger("search_node"), "[DEBUG] Extended direction: norm(%.2f, %.2f, %.2f),
        // extended_target: (%.2f, %.2f, %.2f)",
        //         dir.x(), dir.y(), dir.z(), extended_target.position.x(), extended_target.position.y(),
        //         extended_target.position.z());
      }

      publishPositionCommand(target, init_z);
    }

    // RCLCPP_INFO(rclcpp::get_logger("search_node"), "Agent %d: Flying to extended target %zu at (%.2f, %.2f, %.2f)",
    //          agent_id_, trajectory_index_, extended_target.position.x(), extended_target.position.y(),
    //          extended_target.position.z());
  } else {
    if (ga_calculate_in_progress_ && !has_next_trajectory_) {
      RCLCPP_INFO(rclcpp::get_logger("search_node"), "Waiting for GA thread to complete...");
    }
    if (trajectory_index_ >= current_trajectory_.size()) {
      trajectory_progress_ = 1.0;
      if (use_event_trigger && !ga_calculate_in_progress_ && !has_next_trajectory_) {
        manager_->recordForcedReplan();
        startPredictiveGACalculation();
      }
    }
  }
}

void TrajectoryController::resetSegmentProgress() {
  current_segment_index_ = std::numeric_limits<size_t>::max();
  current_segment_initial_distance_ = 0.0;
}

void TrajectoryController::startPredictiveGACalculation() {
  if (ga_calculate_in_progress_ && ga_thread_.joinable()) {
    RCLCPP_WARN(rclcpp::get_logger("search_node"), "GA thread is already running, skipping new thread creation");
    return;
  }

  if (ga_thread_.joinable()) {
    RCLCPP_WARN(rclcpp::get_logger("search_node"), "Previous GA thread was not properly cleaned up, joining now...");
    ga_thread_.join();
  }

  RCLCPP_INFO(rclcpp::get_logger("search_node"), "Progress reached %.1f%%, starting predictive GA calculation...",
              trajectory_progress_ * 100);
  ga_calculate_in_progress_ = true;

  PredictState captured_next_target;
  bool valid_next_target = false;
  if (trajectory_index_ < current_trajectory_.size()) {
    captured_next_target = current_trajectory_.back();
    valid_next_target = true;
  }

  ga_thread_ = std::thread([this, captured_next_target, valid_next_target]() {
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "=== BACKGROUND GA THREAD STARTED ===");
    if (valid_next_target) {
      bool use_next_target = true;
      const PredictState *next_target = &captured_next_target;
      auto start_time = std::chrono::high_resolution_clock::now();
      manager_->executeDMPCDecision(use_next_target, next_target);
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      RCLCPP_INFO(rclcpp::get_logger("search_node"), "GA calculation time: %ld milliseconds", duration.count());
    } else {
      auto start_time = std::chrono::high_resolution_clock::now();
      manager_->executeDMPCDecision(false, nullptr);
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      RCLCPP_INFO(rclcpp::get_logger("search_node"), "GA calculation time: %ld milliseconds", duration.count());
    }

    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      next_trajectory_ = manager_->latestPredictStateGet();
    }
    has_next_trajectory_ = true;
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "=== BACKGROUND GA THREAD COMPLETED ===");
  });
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "Background thread launched, continuing flight...");
}

void TrajectoryController::handleGAThreadCompletion() {
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "=== WAITING FOR GA THREAD TO COMPLETE ===");
  ga_thread_.join();
  ga_calculate_in_progress_ = false;

  if (has_next_trajectory_ && trajectory_progress_ >= 0.999) {
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "=== SWITCHING TO NEW TRAJECTORY ===");
    manager_->resetEventTrigger();

    if (ga_thread_.joinable()) {
      ga_thread_.join();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("search_node"), "GA thread is not joinable, may have already completed");
    }

    ga_calculate_in_progress_ = false;
    has_next_trajectory_ = false;

    std::vector<PredictState> temp_trajectory;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      temp_trajectory = next_trajectory_;
    }
    if (temp_trajectory.size() > 1) {
      const size_t execution_points = static_cast<size_t>(use_event_trigger ? path_points_per_execution
                                                                            : fixed_period_execution_points);
      temp_trajectory.resize(std::min(temp_trajectory.size(), execution_points));
      current_trajectory_ = temp_trajectory;
      trajectory_index_ = 0;
      last_event_check_index_ = std::numeric_limits<size_t>::max();

      {
        custom_msgs::msg::PredictTrajectory predict_trajectory;
        custom_msgs::msg::PredictState predict_state;
        predict_trajectory.agent_id = agent_id_ + 1;
        predict_trajectory.time = rclcpp::Clock().now();
        for (size_t i = 0; i < current_trajectory_.size(); ++i) {
          Eigen::Vector3d global_pos = current_trajectory_[i].position - Eigen::Vector3d(init_x_, init_y_, 0);
          predict_state.y = global_pos.x();
          predict_state.x = global_pos.y();
          predict_state.z = -global_pos.z();
          predict_state.yaw = current_trajectory_[i].yaw;
          predict_trajectory.waypoints.push_back(predict_state);
        }
        predict_trajectory_pub_->publish(predict_trajectory);
        std::cout << "Predict trajectory has been published" << std::endl;
      }
      resetSegmentProgress();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("search_node"), "GA calculation failed, setting hover state");

      Eigen::Vector3d current_pos;
      double current_yaw;
      if (getLatestAgentPosition(*manager_, agent_id_, current_pos, current_yaw)) {
        current_trajectory_.clear();
        PredictState hover_state;
        if (manager_->getUseInitAsTakeoffPosition()) {
          hover_state.position = current_pos - Eigen::Vector3d(init_x_, init_y_, 0.0);
        } else {
          hover_state.position = current_pos;
        }
        hover_state.yaw = current_yaw;
        current_trajectory_.push_back(hover_state);
        trajectory_index_ = 0;
        last_event_check_index_ = std::numeric_limits<size_t>::max();
        resetSegmentProgress();
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("search_node"), "current_trajectory size: %zu", current_trajectory_.size());
    std::ostringstream oss;
    oss << "New trajectory: ";
    for (size_t i = 0; i < current_trajectory_.size(); ++i) {
      const auto &pt = current_trajectory_[i];
      oss << "agent_id_" << agent_id_ << "(" << pt.position.x() << ", " << pt.position.y() << ", " << pt.position.z()
          << ", " << pt.yaw << ") ";
    }
    RCLCPP_INFO(rclcpp::get_logger("search_node"), oss.str());
    trajectory_index_ = 0;
    last_event_check_index_ = std::numeric_limits<size_t>::max();
    path_index_ = 0;
    has_next_trajectory_ = false;
  }
}

void TrajectoryController::executeDMPCDecision() {
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "Executing DMPC Decision...");
  rclcpp::Time t_start = rclcpp::Clock().now();
  bool use_next_target = false;
  PredictState *next_target = nullptr;
  manager_->executeDMPCDecision(use_next_target, next_target);

  rclcpp::Time t_end = rclcpp::Clock().now();
  double duration = (t_end - t_start).seconds() * 1000.0;
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "[DMPC] Decision execution time: %.2f ms", duration);

  current_trajectory_ = manager_->latestPredictStateGet();
  manager_->resetEventTrigger();

  if (current_trajectory_.size() == 1) {
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "DMPC returned single point (hover mode), keeping as is");
  } else {
    const size_t execution_points = static_cast<size_t>(use_event_trigger ? path_points_per_execution
                                                                          : fixed_period_execution_points);
    current_trajectory_.resize(std::min(current_trajectory_.size(), execution_points));
  }

  RCLCPP_INFO(rclcpp::get_logger("search_node"), "current_trajectory size: %zu", current_trajectory_.size());

  if (agent_id_ == 1) {
    for (size_t i = 0; i < current_trajectory_.size(); ++i) {
      const auto &pt = current_trajectory_[i];
      RCLCPP_INFO(rclcpp::get_logger("search_node"),
                  "[DMPC Decision] Waypoint %zu: Position (%.3f, %.3f, %.3f), Yaw: %.2f deg", i, pt.position.x(),
                  pt.position.y(), pt.position.z(), pt.yaw * 180.0 / M_PI);
    }
  }

  trajectory_index_ = 0;
  last_event_check_index_ = std::numeric_limits<size_t>::max();
  resetSegmentProgress();
  need_new_plan_ = false;
  last_plan_time_ = rclcpp::Clock().now();
}

void TrajectoryController::handleReturnToInitialPositionPhase(const Eigen::Vector3d &cur_pos, double cur_yaw,
                                                              double init_x, double init_y, double init_z) {
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "Return to initial position");

  // if (!getLatestAgentPosition(*manager_, agent_id_, cur_pos_, cur_yaw)) {
  //     return;
  // }

  // Eigen::Vector3d cur_pos_;
  // cur_pos_ = mavros_controller_->getCurrentPosition();

  double dx = cur_pos.x() - init_x;
  double dy = cur_pos.y() - init_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.3) {
    RCLCPP_INFO(rclcpp::get_logger("search_node"), "Reached initial position (%.2f, %.2f), start landing", init_x,
                init_y);
    phase_ = Phase::LANDING;
    return;
  }

  quadrotor_msgs::msg::PositionCommand cmd;
  cmd.position.x = init_x;
  cmd.position.y = init_y;
  cmd.position.z = init_z;
  cmd.yaw = 0.0;
  cmd.header.stamp = rclcpp::Clock().now();
  cmd_pub_->publish(cmd);

  if (mavros_controller_) {
    Eigen::Vector3d position(0, 0, init_z);
    mavros_controller_->publishPositionNoVelocity_NoYaw(position);
  }
}

void TrajectoryController::publishPositionCommand(const PredictState &target, double init_z) {
  quadrotor_msgs::msg::PositionCommand cmd;
  cmd.position.x = target.position.x();
  cmd.position.y = target.position.y();
  cmd.position.z = init_z;
  cmd.yaw = target.yaw;
  cmd.header.stamp = rclcpp::Clock().now();
  cmd_pub_->publish(cmd);

  if (mavros_controller_) {
    Eigen::Vector3d position(target.position.x(), target.position.y(), init_z);
    mavros_controller_->publishPositionVelocity(position, 6.0);
    // mavros_controller_->publishPositionNoVelocity(position);
  }
}

void TrajectoryController::executeLoop(double init_x, double init_y, double init_z) {
  Eigen::Vector3d cur_pos;
  double cur_yaw;

  // Getcurrentposition
  if (!getLatestAgentPosition(*manager_, agent_id_, cur_pos, cur_yaw)) {
    RCLCPP_WARN(rclcpp::get_logger("search_node"), "Cannot get current position for agent %d. Skipping this loop.",
                agent_id_);
    return;
  }

  // cur_pos = mavros_controller_->getCurrentPosition();
  // cur_yaw = mavros_controller_->getCurrentYaw();

  // if(isOutOfTaskArea(cur_pos, map_param_.map_x, map_param_.map_y) || isReachMaxTaskTime(task_param_.task_start_time,
  // task_param_.max_task_time)){
  //     RCLCPP_INFO(rclcpp::get_logger("search_node"), "Agent %d: Out of task area, return to initial position",
  //     agent_id_); phase_ = Phase::RETURN_TO_INITIAL_POSITION; current_trajectory_.clear(); path_index_ = 0;
  //     trajectory_index_ = 0;
  // }

  switch (phase_) {
    case Phase::TO_INITIAL_POSITION:
      handleInitialPositionPhase(cur_pos, init_x, init_y, init_z);
      break;

    case Phase::PLANNING_AND_TRACKING:
      handlePlanningAndTrackingPhase(cur_pos, cur_yaw, init_z);
      break;

    case Phase::RETURN_TO_INITIAL_POSITION:
      handleReturnToInitialPositionPhase(cur_pos, cur_yaw, init_x, init_y, init_z);
      break;

    case Phase::LANDING:
      RCLCPP_INFO(rclcpp::get_logger("search_node"), "Landing");
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("search_node"), "Invalid phase: %d", static_cast<int>(phase_));
      break;
  }
}

void controlExecutionLoop(rclcpp::Node::SharedPtr node, SearchManager *manager,
                          rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub, double init_x,
                          double init_y, double init_z, int agent_id, TaskParam &task_param_,
                          InitMapParam &map_param_) {
  static std::unique_ptr<TrajectoryController> controller;
  if (!controller) {
    controller = std::make_unique<TrajectoryController>(node, manager, cmd_pub, agent_id, task_param_, map_param_,
                                                        init_x, init_y);
  }
  controller->executeLoop(init_x, init_y, init_z);
}

void gridDataUpdateCallback(SearchManager *manager) {
  manager->gridDataUpdate();
  // manager->envGridDataPrint();
}

void printGridDataCallback(SearchManager *manager) {
  manager->envGridDataPrint();
}

void taskStartServiceCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                              std_srvs::srv::SetBool::Response::SharedPtr response) {
  send_task_start = request->data;
  RCLCPP_INFO(rclcpp::get_logger("search_node"), "Task start service called: send_task_start = %s",
              send_task_start ? "true" : "false");
  response->success = true;
  response->message = send_task_start ? "Task started" : "Task stopped";
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("search_manager_node");
  SearchManager manager;
  manager.init(node);

  int agent_id;
  node->get_parameter("agent_id", agent_id);
  agent_id = agent_id;
  node->get_parameter("init_z_", init_z);
  node->get_parameter("init_x_", init_x);
  node->get_parameter("init_y_", init_y);
  bool use_real_uav = false;
  std::string uav_id = "uav1";
  if (!node->has_parameter("use_real_uav")) {
    node->declare_parameter<bool>("use_real_uav", use_real_uav);
  }
  if (!node->has_parameter("uav_id")) {
    node->declare_parameter<std::string>("uav_id", uav_id);
  }
  node->get_parameter("use_real_uav", use_real_uav);
  node->get_parameter("uav_id", uav_id);
  RCLCPP_INFO(rclcpp::get_logger("search_node"),
              "Initial position: x = %.2f, y = %.2f, z = %.2f, use_real_uav=%d, uav_id=%s", init_x, init_y, init_z,
              use_real_uav ? 1 : 0, uav_id.c_str());

  // Set the history file path
  std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : ".";
  std::string history_path = home_dir + "/grid_history_" + std::to_string(agent_id) + ".txt";
  manager.getEnv()->setHistoryFilePath(history_path);
  manager.getEnv()->startHistoryThread();  // Start the history recording thread

  InitMapParam map_param_;
  TaskParam task_param_;
  node->declare_parameter("task_param/max_trajectory_iterations", 10);
  node->get_parameter("task_param/max_trajectory_iterations", task_param_.max_trajectory_iterations);
  node->declare_parameter("task_param/rhoMin", 1.0);
  node->get_parameter("task_param/rhoMin", task_param_.rhoMin);
  node->declare_parameter("task_param/use_event_trigger", true);
  node->get_parameter("task_param/use_event_trigger", task_param_.use_event_trigger);
  node->get_parameter("task_param/path_points_per_execution", task_param_.path_points_per_execution);
  node->declare_parameter("task_param/fixed_period_execution_points", 3);
  node->get_parameter("task_param/fixed_period_execution_points", task_param_.fixed_period_execution_points);
  node->declare_parameter("task_param/ga_start_time", 0.5);
  node->get_parameter("task_param/ga_start_time", task_param_.ga_start_time);
  node->declare_parameter("task_param/distance_to_next_target", 0.5);
  node->get_parameter("task_param/distance_to_next_target", task_param_.distance_to_next_target);
  node->declare_parameter("task_param/max_task_time", 100.0);
  node->get_parameter("task_param/max_task_time", task_param_.max_task_time);
  cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);
  predict_trajectory_pub_ =
      node->create_publisher<custom_msgs::msg::PredictTrajectory>("outer/information/predict_trajectory", 30);

  auto task_start_service = node->create_service<std_srvs::srv::SetBool>("/search_manager/start_task",
                                                                         taskStartServiceCallback);

  std::unique_ptr<GaussianPlaneVisualizer> visualizer;
  visualizer = std::make_unique<GaussianPlaneVisualizer>(node, "/gaussian_plane_marker");
  visualizer->initGrid(node);

  rclcpp::Rate rate(20);

  auto timer = node->create_wall_timer(std::chrono::milliseconds(50),
                                       [&manager]() { gridDataUpdateCallback(&manager); });

  static rclcpp::Time print_time = rclcpp::Clock().now();
  rclcpp::Duration print_duration(0, 500000000);

  if (!use_real_uav) {
    while (rclcpp::ok()) {
      task_param_.task_start_time = rclcpp::Clock().now();
      if (manager.isReceivedAgentData()) {
        controlExecutionLoop(node, &manager, cmd_pub, init_x, init_y, init_z, agent_id, task_param_, map_param_);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("search_node"), "Agent %d: No agent data received, skipping execution loop",
                    agent_id);
      }

      if (rclcpp::Clock().now() - print_time > print_duration) {
        print_time = rclcpp::Clock().now();
        printGridDataCallback(&manager);
        const std::vector<GridData> *convertedGrids = &manager.envGridDataGet();
        visualizer->updateGrid(convertedGrids);
      }

      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  } else {
    mavros_controller_ = std::make_unique<MavrosController>(node, uav_id);
    mavros_controller_->setTakeoffHeight(init_z);
    if (!mavros_controller_->initialize()) {
      return -1;
    }
    if (!mavros_controller_->waitForConnection(8.0)) {
      RCLCPP_ERROR(rclcpp::get_logger("search_node"), "Failed to connect to MAVROS");
      return -1;
    }
    // for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    //     mavros_controller_->publishPositionNoVelocity_NoYaw(Eigen::Vector3d(0, 0, init_z));
    //     rclcpp::spin_some(node);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    // mavros_controller_->setOffboardMode();

    // mavros_controller_->arm();
    static bool offboard_set = false;

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (!mavros_controller_->isTakeoffFinished()) {
        // mavros_controller_->publishPositionNoVelocity_NoYaw(Eigen::Vector3d(0, 0, init_z));
      } else {
        if (!offboard_set) {
          for (int i = 0; i < 30 && rclcpp::ok(); ++i) {
            mavros_controller_->publishPositionNoVelocity_NoYaw(Eigen::Vector3d(0, 0, init_z));
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
          offboard_set = mavros_controller_->setOffboardMode();
        }

        // std::cout << "offboard_set: " << offboard_set << std::endl;
        // std::cout << "isFwMode: " << mavros_controller_->isFwMode() << std::endl;
        if (offboard_set) {
          task_param_.task_start_time = rclcpp::Clock().now();
          // manager.agentMsgPublish();
          if (manager.isReceivedAgentData() && send_task_start) {
            controlExecutionLoop(node, &manager, cmd_pub, init_x, init_y, init_z, agent_id, task_param_, map_param_);
          } else {
            RCLCPP_WARN(rclcpp::get_logger("search_node"), "Agent %d: No agent data received, skipping execution loop",
                        agent_id);
          }
        }
      }

      if (rclcpp::Clock().now() - print_time > print_duration) {
        print_time = rclcpp::Clock().now();
        printGridDataCallback(&manager);
        const std::vector<GridData> *convertedGrids = &manager.envGridDataGet();
        visualizer->updateGrid(convertedGrids);
      }

      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

  manager.getEnv()->stopHistoryThread();
  manager.getEnv()->closeHistoryFile();

  return 0;
}
