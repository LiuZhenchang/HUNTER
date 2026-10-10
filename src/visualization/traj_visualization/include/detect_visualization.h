#ifndef DETECT_VISUALIZATION_H
#define DETECT_VISUALIZATION_H
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "SensorPorperty.h"
#include "custom_msgs/msg/communication_data.hpp"

class TrajectoryVisualization {
 public:
  explicit TrajectoryVisualization(const rclcpp::Node::SharedPtr &node);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int color_value);
  void updateGrid(const nav_msgs::msg::Odometry::SharedPtr msg, int color_value);
  void communicationDataCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg, int color_value,
                                 int agent_id);

 private:
  rclcpp::Node::SharedPtr node_;
  SensorPorperty sensor_;

  int drone_count_{2};
  double resolution_{1.0};
  double map_xmin_{-100.0};
  double map_ymin_{-100.0};

  std::vector<rclcpp::Subscription<custom_msgs::msg::CommunicationData>::SharedPtr> odom_subs_;
  // std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  nav_msgs::msg::OccupancyGrid grid_;
  std::mutex grid_mutex_;
};

#endif  // DETECT_VISUALIZATION_H
