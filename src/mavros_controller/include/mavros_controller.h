#ifndef MAVROS_CONTROLLER_H
#define MAVROS_CONTROLLER_H

#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class MavrosController {
 public:
  explicit MavrosController(rclcpp::Node::SharedPtr node, const std::string &uav_id);
  ~MavrosController();

  bool initialize();
  bool waitForConnection(double timeout);
  bool setOffboardMode();
  bool arm();
  bool publishPositionNoVelocity(const Eigen::Vector3d &pos);
  bool publishPositionNoVelocity_NoYaw(const Eigen::Vector3d &pos);
  bool publishPositionVelocity(const Eigen::Vector3d &pos, double velocity);
  bool changeFwMode();
  bool disarm();
  void publishHover();
  void extendedStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
  // Status queries
  bool isConnected() const {
    return is_connected_;
  }
  bool isArmed() const {
    return is_armed_;
  }
  bool isOffboard() const {
    return is_offboard_;
  }
  bool isTakeoff() const {
    return is_takeoff_;
  }
  bool isTakeoffFinished();
  bool isFwMode() const {
    return is_fw_mode_;
  }

  void setTakeoffFinished() {
    is_takeoff_ = true;
  }

  // Position access
  Eigen::Vector3d getCurrentPosition();
  double getCurrentYaw();

  // Control interface
  void setTakeoffHeight(double height);

  mavros_msgs::msg::PositionTarget getLatestPositionTarget() const;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string uav_id_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pos_target_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr change_fw_mode_client_;

  mavros_msgs::msg::State current_state_;
  mavros_msgs::msg::ExtendedState current_extended_state_;
  geometry_msgs::msg::PoseStamped current_pose_;
  mutable std::mutex pose_mutex_;
  mutable std::mutex pos_target_mutex_;
  mavros_msgs::msg::PositionTarget latest_pos_target_cmd_;
  double takeoff_height_ = 80.0;

  std::atomic<bool> keep_publishing_;
  std::thread offboard_thread_;

  bool is_connected_ = false;
  bool is_armed_ = false;
  bool is_offboard_ = false;
  bool is_takeoff_ = false;
  bool is_fw_mode_ = false;

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void offboardPublisherThread();
};

#endif