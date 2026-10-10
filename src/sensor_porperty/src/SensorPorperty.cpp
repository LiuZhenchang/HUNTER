#include "SensorPorperty.h"
#include <cmath>
#include <functional>
#include <unordered_set>
#include <vector>

namespace std {
template <>
struct hash<Eigen::Vector2i> {
  size_t operator()(const Eigen::Vector2i &v) const noexcept {
    return std::hash<long long>()(((long long)v.x() << 32) ^ (long long)v.y());
  }
};
}  // namespace std

inline std::vector<Eigen::Vector2d> transformRegionVertices(std::vector<double> &region_vertices_raw) {
  if (region_vertices_raw.size() % 2 != 0 || region_vertices_raw.size() < 6) {
    std::cout << "region_vertices invalid, expect even number >=6, got " << region_vertices_raw.size() << std::endl;
    return std::vector<Eigen::Vector2d>();
  } else {
    std::vector<Eigen::Vector2d> region_vertices_;
    region_vertices_.clear();
    region_vertices_.reserve(region_vertices_raw.size() / 2);
    for (size_t i = 0; i + 1 < region_vertices_raw.size(); i += 2) {
      region_vertices_.emplace_back(region_vertices_raw[i], region_vertices_raw[i + 1]);
    }
    return region_vertices_;
  }
}

SensorPorperty::SensorPorperty(rclcpp::Node::SharedPtr node) : node_(node) {
  node_->declare_parameter<double>("length_", 0.1);
  node_->declare_parameter<int>("seeker_type", 0);
  node_->declare_parameter<double>("cover_length", 2.0);
  node_->declare_parameter<double>("cover_width", 5.0);

  std::vector<double> region_vertices_raw;
  if (!node->has_parameter("region_vertices")) {
    region_vertices_raw = {-800.0, -1000.0, -500.0, 0.0, 300.0, 0.0, 800.0, -800.0};
    node->declare_parameter<std::vector<double>>("region_vertices", region_vertices_raw);
  }
  region_vertices_raw = node->get_parameter("region_vertices").as_double_array();
  auto region_vertices_ = transformRegionVertices(region_vertices_raw);

  std::vector<double> vertices_x, vertices_y;
  for (size_t i = 0; i < region_vertices_.size(); i++) {
    vertices_x.push_back(region_vertices_[i].x());
    vertices_y.push_back(region_vertices_[i].y());
  }

  if (!vertices_x.empty() && !vertices_y.empty()) {
    Map_xmin_ = *std::min_element(vertices_x.begin(), vertices_x.end());
    Map_xmax_ = *std::max_element(vertices_x.begin(), vertices_x.end());
    Map_ymin_ = *std::min_element(vertices_y.begin(), vertices_y.end());
    Map_ymax_ = *std::max_element(vertices_y.begin(), vertices_y.end());
    RCLCPP_INFO(node_->get_logger(), "SensorPorperty: Region vertices: xMin_=%f, xMax_=%f, yMin_=%f, yMax_=%f",
                Map_xmin_, Map_xmax_, Map_ymin_, Map_ymax_);
  }

  length_ = node_->get_parameter("length_").as_double();
  seeker_type_ = node_->get_parameter("seeker_type").as_int();
  cover_length_ = node_->get_parameter("cover_length").as_double();
  cover_width_ = node_->get_parameter("cover_width").as_double();

  std::string node_name = node_->get_name();
  RCLCPP_INFO(node_->get_logger(), "SensorPorperty created for node: %s", node_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "  cover_length: %.2f", cover_length_);
  RCLCPP_INFO(node_->get_logger(), "  cover_width: %.2f", cover_width_);
  RCLCPP_INFO(node_->get_logger(), "  seeker_type: %d", seeker_type_);

  if (seeker_type_ == 1) {
    bias_ = 0.5;
    // cover_length_ = 2.0;
    // cover_width_ = 5.0;
  } else {
    bias_ = 0.0;
    // cover_length_ = 2.0;
    // cover_width_ = 5.0;
  }
}

int SensorPorperty::getType() {
  return seeker_type_;
}

double SensorPorperty::getBias() {
  return bias_;
}

double SensorPorperty::getCoverLength() {
  return cover_length_;
}

double SensorPorperty::getCoverWidth() {
  return cover_width_;
}

double SensorPorperty::getMapXmin() {
  return Map_xmin_;
}

double SensorPorperty::getMapXmax() {
  return Map_xmax_;
}

double SensorPorperty::getMapYmin() {
  return Map_ymin_;
}

double SensorPorperty::getMapYmax() {
  return Map_ymax_;
}

double SensorPorperty::getlength() {
  return length_;
}

std::vector<Eigen::Vector2i> SensorPorperty::seekerForward(const Eigen::Vector2d &position, double theta) {
  std::unordered_set<Eigen::Vector2i, std::hash<Eigen::Vector2i>> unique_indices;

  Eigen::Vector2d center = position + bias_ * Eigen::Vector2d(std::cos(theta), std::sin(theta));
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  double step_size = length_ * 1.0;
  double half_length = cover_length_ / 2.0;
  double half_width = cover_width_ / 2.0;

  int width_in_cells = static_cast<int>((Map_xmax_ - Map_xmin_) / length_);
  int height_in_cells = static_cast<int>((Map_ymax_ - Map_ymin_) / length_);
  int max_index = width_in_cells * height_in_cells;

  for (double lx = -half_length; lx <= half_length; lx += step_size) {
    for (double ly = -half_width; ly <= half_width; ly += step_size) {
      double offset_x = lx * cos_theta - ly * sin_theta;
      double offset_y = lx * sin_theta + ly * cos_theta;

      Eigen::Vector2d grid_point = center + Eigen::Vector2d(offset_x, offset_y);
      if (grid_point.x() >= Map_xmin_ && grid_point.x() < Map_xmax_ && grid_point.y() >= Map_ymin_ &&
          grid_point.y() < Map_ymax_) {
        int grid_x = static_cast<int>(std::floor((grid_point.x() - Map_xmin_) / length_));
        int grid_y = static_cast<int>(std::floor((grid_point.y() - Map_ymin_) / length_));
        int index = grid_y * width_in_cells + grid_x;

        if (index >= 0 && index < max_index) {
          unique_indices.insert(Eigen::Vector2i(grid_x, grid_y));
        }
      }
    }
  }

  return std::vector<Eigen::Vector2i>(unique_indices.begin(), unique_indices.end());
}

double SensorPorperty::estEnvValue(std::vector<Grid> &grids, const Eigen::Vector2d &position, double theta) {
  std::vector<Eigen::Vector2i> indices = seekerForward(position, theta);
  double env_value = 0.0;
  for (const Eigen::Vector2i &index : indices) {
    int grid_index = index.y() * static_cast<int>((Map_xmax_ - Map_xmin_) / length_) + index.x();
    env_value += grids[grid_index].uncertainty_;
  }
  return env_value;
}

double SensorPorperty::estTargetValue(std::vector<Grid> &grids, const Eigen::Vector2d &position, double theta) {
  std::vector<Eigen::Vector2i> indices = seekerForward(position, theta);
  double target_value = 0.0;
  for (const Eigen::Vector2i &index : indices) {
    int grid_index = index.y() * static_cast<int>((Map_xmax_ - Map_xmin_) / length_) + index.x();
    target_value += grids[grid_index].probability_;
  }
  return target_value;
}
