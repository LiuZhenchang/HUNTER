#include "obstacle_decy/obstacle_visualization.h"

ObstacleVisualization::ObstacleVisualization(const rclcpp::NodeOptions &options)
    : rclcpp::Node("obstacle_visualization", options) {
  sub_ = this->create_subscription<obstacle_vis::msg::ObstacleArray>("obstacle_info", 10,
                                                                     std::bind(&ObstacleVisualization::obstacleCallback,
                                                                               this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
}

void ObstacleVisualization::obstacleCallback(const obstacle_vis::msg::ObstacleArray::SharedPtr msg) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < msg->obstacles.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = msg->obstacles[i].position_x;
    marker.pose.position.y = msg->obstacles[i].position_y;
    marker.pose.position.z = 0.0;

    marker.scale.x = msg->obstacles[i].radius * 2;
    marker.scale.y = msg->obstacles[i].radius * 2;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleVisualization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
