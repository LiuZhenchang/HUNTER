#include "mavros_controller.h"

#include <chrono>
#include <future>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

MavrosController::MavrosController(rclcpp::Node::SharedPtr node, const std::string &uav_id)
    : node_(std::move(node)), uav_id_(uav_id), keep_publishing_(false) {
  if (!node_) {
    throw std::invalid_argument("MavrosController requires a valid node");
  }

  current_pose_.pose.orientation.w = 1.0;

  auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto sensor_qos = rclcpp::SensorDataQoS();

  std::string ns = "/" + uav_id_;
  pos_target_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(ns + "/setpoint_raw/local", cmd_qos);
  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(ns + "/state", state_qos,
                                                                   std::bind(&MavrosController::stateCallback, this,
                                                                             std::placeholders::_1));
  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(ns + "/local_position/odom", sensor_qos,
                                                                  std::bind(&MavrosController::poseCallback, this,
                                                                            std::placeholders::_1));
  extended_state_sub_ = node_->create_subscription<mavros_msgs::msg::ExtendedState>(
      ns + "/extended_state", sensor_qos,
      std::bind(&MavrosController::extendedStateCallback, this, std::placeholders::_1));

  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(ns + "/cmd/arming");
  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(ns + "/set_mode");
  change_fw_mode_client_ = node_->create_client<mavros_msgs::srv::CommandLong>(ns + "/cmd/command");
}

void MavrosController::extendedStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
  current_extended_state_ = *msg;
  is_fw_mode_ = (current_extended_state_.vtol_state == 4);
}

MavrosController::~MavrosController() {
  keep_publishing_ = false;
  if (offboard_thread_.joinable()) {
    offboard_thread_.join();
  }
}

void MavrosController::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_.pose = msg->pose.pose;
}

void MavrosController::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
  is_connected_ = current_state_.connected;
  is_armed_ = current_state_.armed;
  is_offboard_ = (current_state_.mode == "OFFBOARD");
}

void MavrosController::offboardPublisherThread() {
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok() && keep_publishing_) {
    mavros_msgs::msg::PositionTarget cmd_copy;
    {
      std::lock_guard<std::mutex> lock(pos_target_mutex_);
      cmd_copy = latest_pos_target_cmd_;
    }
    pos_target_pub_->publish(cmd_copy);
    rate.sleep();
  }
}

bool MavrosController::initialize() {
  mavros_msgs::msg::PositionTarget init_cmd;
  init_cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  init_cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                       mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  init_cmd.position.x = 0.0;
  init_cmd.position.y = 0.0;
  init_cmd.position.z = takeoff_height_;

  {
    std::lock_guard<std::mutex> lock(pos_target_mutex_);
    latest_pos_target_cmd_ = init_cmd;
  }

  keep_publishing_ = true;
  offboard_thread_ = std::thread(&MavrosController::offboardPublisherThread, this);
  return true;
}

bool MavrosController::waitForConnection(double timeout) {
  RCLCPP_INFO(node_->get_logger(), "Waiting for MAVROS connection...");
  const auto start_time = node_->now();
  rclcpp::WallRate rate(10.0);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  while (rclcpp::ok() && !is_connected_) {
    if (timeout > 0.0) {
      const auto elapsed = node_->now() - start_time;
      if (elapsed.seconds() > timeout) {
        RCLCPP_ERROR(node_->get_logger(), "MAVROS connection timeout!");
        return false;
      }
    }
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "MAVROS connected!");
  return true;
}

bool MavrosController::setOffboardMode() {
  // Create the request
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "OFFBOARD";

  // Make the asynchronous call
  auto future = set_mode_client_->async_send_request(request);

  // Wait for the service response (up to 1 second)
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->mode_sent) {
      RCLCPP_INFO(node_->get_logger(), "Offboard enabled");
      return true;
    }
  }

  RCLCPP_WARN(node_->get_logger(), "Failed to enable Offboard mode");
  return false;
}

bool MavrosController::arm() {
  if (!arming_client_->wait_for_service(2s)) {
    RCLCPP_WARN(node_->get_logger(), "Arming service not available");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;

  auto future = arming_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Arming service call timed out");
    return false;
  }

  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "Vehicle armed");
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Arming failed");
  return false;
}

bool MavrosController::changeFwMode() {
  if (!change_fw_mode_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "CommandLong service not available");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->broadcast = false;
  request->command = 3000;
  request->confirmation = 0;
  request->param1 = 4.0f;

  auto future = change_fw_mode_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "CommandLong service call timed out");
    return false;
  }

  auto response = future.get();
  if (response->success) {
    is_fw_mode_ = true;
    RCLCPP_INFO(node_->get_logger(), "Switch to FW mode");
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to switch to FW mode");
  return false;
}

bool MavrosController::publishPositionNoVelocity(const Eigen::Vector3d &pos) {
  mavros_msgs::msg::PositionTarget cmd;
  cmd.header.stamp = node_->get_clock()->now();
  cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                  mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  cmd.position.x = pos.x();
  cmd.position.y = pos.y();
  cmd.position.z = pos.z();

  const Eigen::Vector3d current_pos = getCurrentPosition();
  Eigen::Vector3d direction = pos - current_pos;
  double yaw = std::atan2(direction.y(), direction.x());
  cmd.yaw = yaw;

  {
    std::lock_guard<std::mutex> lock(pos_target_mutex_);
    latest_pos_target_cmd_ = cmd;
  }
  return true;
}

bool MavrosController::publishPositionNoVelocity_NoYaw(const Eigen::Vector3d &pos) {
  mavros_msgs::msg::PositionTarget cmd;
  cmd.header.stamp = node_->get_clock()->now();
  cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                  mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  cmd.position.x = pos.x();
  cmd.position.y = pos.y();
  cmd.position.z = pos.z();

  // RCLCPP_INFO(node_->get_logger(), "[TrackDebug] MAVROS setpoint (setpoint_raw/local): pos(%.2f, %.2f, %.2f)",
  // cmd.position.x, cmd.position.y, cmd.position.z);
  {
    std::lock_guard<std::mutex> lock(pos_target_mutex_);
    latest_pos_target_cmd_ = cmd;
  }
  return true;
}

bool MavrosController::publishPositionVelocity(const Eigen::Vector3d &pos, double velocity) {
  mavros_msgs::msg::PositionTarget cmd;
  cmd.header.stamp = node_->get_clock()->now();
  cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX | mavros_msgs::msg::PositionTarget::IGNORE_PY |
                  mavros_msgs::msg::PositionTarget::IGNORE_PZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                  mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

  const Eigen::Vector3d current_pos = getCurrentPosition();
  Eigen::Vector3d direction = pos - current_pos;
  double distance = direction.norm();

  // Stop when the distance is small and keep the current yaw
  if (distance < 1.0) {
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    // Keep the current yaw without changing the heading
    cmd.yaw = getCurrentYaw();
  } else {
    direction.normalize();
    cmd.velocity.x = direction.x() * velocity;
    cmd.velocity.y = direction.y() * velocity;
    cmd.velocity.z = direction.z() * velocity;
    double yaw = std::atan2(direction.y(), direction.x());
    cmd.yaw = yaw;
  }

  {
    std::lock_guard<std::mutex> lock(pos_target_mutex_);
    latest_pos_target_cmd_ = cmd;
  }
  return true;
}

Eigen::Vector3d MavrosController::getCurrentPosition() {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return Eigen::Vector3d(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
}

double MavrosController::getCurrentYaw() {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  tf2::Quaternion q(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
                    current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

bool MavrosController::disarm() {
  if (!arming_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "Arming service not available");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = false;

  auto future = arming_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Disarming service call timed out");
    return false;
  }

  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "Vehicle disarmed");
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Disarming failed");
  return false;
}

bool MavrosController::isTakeoffFinished() {
  if (is_takeoff_) {
    return true;
  }

  if (std::fabs(current_pose_.pose.position.z - takeoff_height_) < 3.0 && !is_takeoff_) {
    RCLCPP_INFO(node_->get_logger(), "current altitude = %.2f", current_pose_.pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "takeoff finished");
    is_takeoff_ = true;
    return true;
  }
  RCLCPP_INFO(node_->get_logger(), "current altitude = %.2f", current_pose_.pose.position.z);
  return false;
}

void MavrosController::setTakeoffHeight(double height) {
  takeoff_height_ = height;
  RCLCPP_INFO(node_->get_logger(), "takeoff height = %.2f", takeoff_height_);
}

void MavrosController::publishHover() {
  publishPositionVelocity(getCurrentPosition(), 0.0);
}

mavros_msgs::msg::PositionTarget MavrosController::getLatestPositionTarget() const {
  std::lock_guard<std::mutex> lock(pos_target_mutex_);
  return latest_pos_target_cmd_;
}
