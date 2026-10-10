#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "SensorPorperty.h"
#include "armadillo"
#include "custom_msgs/msg/communication_data.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "visualization_msgs/msg/marker.hpp"
class OdomVisualization {
 public:
  OdomVisualization();
  ~OdomVisualization();
  rclcpp::Node::SharedPtr get_node() const {
    return node_;
  }

 private:
  // Node handle, publishers and subscribers
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  // rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covVelPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensorPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr heightPub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  bool cross_config_, tf45_, cov_pos_, cov_vel_, cov_color_, origin_, isOriginSet_;
  double cov_scale_, color_r_, color_g_, color_b_, color_a_, scale_, traj_scale_;
  int drone_num_;
  std::string mesh_resource_, frame_id_;
  int drone_id_;
  arma::colvec poseOrigin_;
  nav_msgs::msg::Odometry odom;
  SensorPorperty sensor_;

  geometry_msgs::msg::PoseStamped poseROS_;
  nav_msgs::msg::Path pathROS_;
  visualization_msgs::msg::Marker velROS_, covROS_, covVelROS_, trajROS_, sensorROS_, meshROS_;
  sensor_msgs::msg::Range heightROS_;
  std::vector<rclcpp::Subscription<custom_msgs::msg::CommunicationData>::SharedPtr> subs_odom_;
  std::map<int, visualization_msgs::msg::Marker> agent_traj_markers;
  std::map<int, nav_msgs::msg::Path> agent_paths_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // void cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd);
  void communicationDataCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg, int agent_id);
};
