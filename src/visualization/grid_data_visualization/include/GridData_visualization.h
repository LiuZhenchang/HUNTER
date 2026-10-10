#ifndef GRIDDATA_VISUALIZATION_H
#define GRIDDATA_VISUALIZATION_H

#include <Eigen/Dense>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#ifndef GRIDDATA_STRUCT_DEFINED
#define GRIDDATA_STRUCT_DEFINED
struct GridData {
  double uncertainty_;
  double probability_;
  int detect_num_;
};
#endif

class GaussianPlaneVisualizer {
 public:
  GaussianPlaneVisualizer(rclcpp::Node::SharedPtr node, const std::string &topic);

  void initGrid(rclcpp::Node::SharedPtr node);
  void visualizeUncertainty(const std::vector<GridData> &grids);
  void visualizeProbability(const std::vector<GridData> &grids);
  void startTimer(double period);
  void updateGrid(const std::vector<GridData> *newGridsPtr);

  std::mutex &getGridMutex();

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uncertainty_pub_;  // Uncertainty publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr probability_pub_;  // Probability publisher
  rclcpp::Subscription<GridData>::SharedPtr grid_sub_;
  double xMin_, xMax_, yMin_, yMax_, resolution_, time_interval_;
  int samplerRowCount_, samplerColCount_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex grids_mutex_;
  std::vector<GridData> grids_;
  std::vector<Eigen::Vector2d> region_vertices_;

  void timerCallback();
};

#endif  // GRIDDATA_VISUALIZATION_H
