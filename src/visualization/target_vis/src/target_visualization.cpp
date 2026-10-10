#include "target_vis/target_visualization.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

TargetVisualization::TargetVisualization(const rclcpp::NodeOptions &options)
    : rclcpp::Node("target_visualization", options) {
  scale_ = this->declare_parameter<double>("scale", 10.0);
  sub_ = this->create_subscription<target_vis::msg::TargetArray>("target_info", rclcpp::QoS(10),
                                                                 std::bind(&TargetVisualization::targetCallback, this,
                                                                           std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_markers", rclcpp::QoS(10));
}

void TargetVisualization::targetCallback(const target_vis::msg::TargetArray::SharedPtr msg) {
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;
  const auto now = this->now();

  for (size_t i = 0; i < msg->targets.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = now;
    marker.ns = "targets";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = msg->targets[i].position_x;
    marker.pose.position.y = msg->targets[i].position_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale_;
    marker.scale.y = scale_;
    marker.scale.z = scale_;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetVisualization>());
  rclcpp::shutdown();
  return 0;
}