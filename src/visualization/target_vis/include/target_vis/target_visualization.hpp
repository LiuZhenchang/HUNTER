#pragma once
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "target_vis/msg/target_array.hpp"

class TargetVisualization : public rclcpp::Node {
 public:
  explicit TargetVisualization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void targetCallback(const target_vis::msg::TargetArray::SharedPtr msg);

  rclcpp::Subscription<target_vis::msg::TargetArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  double scale_{10.0};
};
