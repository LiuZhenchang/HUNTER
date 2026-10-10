#pragma once

#include <rclcpp/rclcpp.hpp>
#include "obstacle_vis/msg/obstacle_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ObstacleVisualization : public rclcpp::Node {
 public:
  explicit ObstacleVisualization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void obstacleCallback(const obstacle_vis::msg::ObstacleArray::SharedPtr msg);

  rclcpp::Subscription<obstacle_vis::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};