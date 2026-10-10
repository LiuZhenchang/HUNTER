#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "GridData_visualization.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("grid_data_visualization_node");
  node->declare_parameter("Map_xmin", -1500.0);
  node->declare_parameter("Map_xmax", 1500.0);
  node->declare_parameter("Map_ymin", -1500.0);
  node->declare_parameter("Map_ymax", 1500.0);
  node->declare_parameter("length_", 15.0);
  node->declare_parameter("time_interval", 2.0);

  auto visualizer = std::make_unique<GaussianPlaneVisualizer>(node, "/gaussian_plane_marker");
  visualizer->initGrid(node);

  RCLCPP_INFO(node->get_logger(), "GridData visualization node started");
  RCLCPP_INFO(node->get_logger(), "Waiting for grid data updates...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
