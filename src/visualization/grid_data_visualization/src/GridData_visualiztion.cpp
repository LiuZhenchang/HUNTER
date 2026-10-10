#include "GridData_visualization.h"

template <typename T>
T clamp(T val, T low, T high) {
  return std::max(low, std::min(val, high));
}

std::vector<Eigen::Vector2d> transformRegionVertices(std::vector<double> &region_vertices_raw) {
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

bool isPointInConvexPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices) {
  if (vertices.size() < 3)
    return false;

  int n = vertices.size();
  bool sign = false;
  bool first = true;

  for (int i = 0; i < n; i++) {
    const Eigen::Vector2d &v1 = vertices[i];
    const Eigen::Vector2d &v2 = vertices[(i + 1) % n];

    // Compute the cross product
    double cross = (v2.x() - v1.x()) * (point.y() - v1.y()) - (v2.y() - v1.y()) * (point.x() - v1.x());

    if (first) {
      sign = cross > 0;
      first = false;
    } else if ((cross > 0) != sign && std::abs(cross) > 1e-10) {
      return false;
    }
  }
  return true;
}

GaussianPlaneVisualizer::GaussianPlaneVisualizer(rclcpp::Node::SharedPtr node, const std::string &topic)
    : node_(node),
      uncertainty_pub_(
          node_->create_publisher<visualization_msgs::msg::Marker>("/uncertainty_marker", rclcpp::QoS(10))),
      probability_pub_(
          node_->create_publisher<visualization_msgs::msg::Marker>("/probability_marker", rclcpp::QoS(10))) {
  RCLCPP_INFO(node_->get_logger(), "Initializing GaussianPlaneVisualizer");
  RCLCPP_INFO(node_->get_logger(), "  - Uncertainty topic: /uncertainty_marker");
  RCLCPP_INFO(node_->get_logger(), "  - Probability topic: /probability_marker");

  if (!uncertainty_pub_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize uncertainty publisher");
  }
  if (!probability_pub_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize probability publisher");
  }
  if (!node_->has_parameter("length_")) {
    node_->declare_parameter("length_", 15.0);
  }
  if (!node_->has_parameter("time_interval")) {
    node_->declare_parameter("time_interval", 2.0);
  }
  std::vector<double> region_vertices_raw;
  if (!node->has_parameter("region_vertices")) {
    region_vertices_raw = {-1000.0, -1000.0, -500.0, 0.0, 300.0, 0.0, 1000.0, -1000.0};
    node->declare_parameter<std::vector<double>>("region_vertices", region_vertices_raw);
  }
  region_vertices_raw = node->get_parameter("region_vertices").as_double_array();
  region_vertices_ = transformRegionVertices(region_vertices_raw);
  node_->get_parameter("length_", resolution_);
  node_->get_parameter("time_interval", time_interval_);

  std::vector<double> vertices_x, vertices_y;
  for (size_t i = 0; i < region_vertices_.size(); i++) {
    vertices_x.push_back(region_vertices_[i].x());
    vertices_y.push_back(region_vertices_[i].y());
  }

  // Compute the bounding rectangle
  if (!vertices_x.empty() && !vertices_y.empty()) {
    xMin_ = *std::min_element(vertices_x.begin(), vertices_x.end());
    xMax_ = *std::max_element(vertices_x.begin(), vertices_x.end());
    yMin_ = *std::min_element(vertices_y.begin(), vertices_y.end());
    yMax_ = *std::max_element(vertices_y.begin(), vertices_y.end());
    RCLCPP_INFO(node_->get_logger(), "Grid_visualization: Region vertices: xMin_=%f, xMax_=%f, yMin_=%f, yMax_=%f",
                xMin_, xMax_, yMin_, yMax_);
  }
  samplerRowCount_ = static_cast<int>(std::ceil((yMax_ - yMin_) / resolution_));
  samplerColCount_ = static_cast<int>(std::ceil((xMax_ - xMin_) / resolution_));
  startTimer(time_interval_);
}

void GaussianPlaneVisualizer::initGrid(rclcpp::Node::SharedPtr node) {}

void GaussianPlaneVisualizer::visualizeUncertainty(const std::vector<GridData> &grids) {
  if (grids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "visualizeUncertainty: grids is empty");
    return;
  }

  int colCount = samplerColCount_;
  int rowCount = samplerRowCount_;
  if (colCount <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "visualizeUncertainty: invalid colCount=%d", colCount);
    return;
  }

  if (grids.size() % colCount != 0) {
    RCLCPP_ERROR(node_->get_logger(), "visualizeUncertainty: grids.size()=%zu is not divisible by colCount=%d",
                 grids.size(), colCount);
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "uncertainty";
  marker.header.stamp = node_->now();
  marker.ns = "gaussian_plane_colored";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.orientation.w = 1.0;
  double dx = resolution_;
  double dy = resolution_;
  double xMin = xMin_, xMax = xMax_;
  double yMin = yMin_, yMax = yMax_;

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(colCount * rowCount);

  double zMin = std::numeric_limits<double>::infinity();
  double zMax = -std::numeric_limits<double>::infinity();

  for (int j = 0; j < rowCount; ++j) {
    for (int i = 0; i < colCount; ++i) {
      int idx = j * colCount + i;
      double x = xMin + i * dx;
      double y = yMin + j * dy;
      double z = grids[idx].uncertainty_;
      if (std::isnan(z) || std::isinf(z)) {
        z = 0.0;
      }
      zMin = std::min(zMin, z);
      zMax = std::max(zMax, z);
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      points.push_back(p);
    }
  }

  for (int j = 0; j < rowCount - 1; ++j) {
    for (int i = 0; i < colCount - 1; ++i) {
      int idx = j * colCount + i;
      int idxRight = idx + 1;
      int idxDown = idx + colCount;
      int idxDiag = idxDown + 1;

      auto addTriangle = [&](int i1, int i2, int i3) {
        bool in_region = isPointInConvexPolygon(Eigen::Vector2d(points[i1].x, points[i1].y), region_vertices_) &&
                         isPointInConvexPolygon(Eigen::Vector2d(points[i2].x, points[i2].y), region_vertices_) &&
                         isPointInConvexPolygon(Eigen::Vector2d(points[i3].x, points[i3].y), region_vertices_);
        if (!in_region) {
          return;
        }

        // vertices
        marker.points.push_back(points[i1]);
        marker.points.push_back(points[i2]);
        marker.points.push_back(points[i3]);
        // colors
        for (auto k : {i1, i2, i3}) {
          int grid_idx = (k / colCount) * colCount + (k % colCount);
          double uncertainty_val = grids[grid_idx].uncertainty_;

          double val;
          if ((zMax - zMin) > 0.01) {
            val = (uncertainty_val - zMin) / (zMax - zMin);
          } else {
            val = std::min(1.0, uncertainty_val / 1.0);
          }

          std_msgs::msg::ColorRGBA c;
          c.r = val;
          c.g = 1.0 - val;
          c.b = 0.2;
          c.a = 1.0;
          marker.colors.push_back(c);
        }
      };

      addTriangle(idx, idxRight, idxDown);
      addTriangle(idxRight, idxDiag, idxDown);
    }
  }
  if (uncertainty_pub_) {
    uncertainty_pub_->publish(marker);
  }
}

void GaussianPlaneVisualizer::visualizeProbability(const std::vector<GridData> &grids) {
  if (grids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "visualizeProbability: grids is empty");
    return;
  }

  int colCount = samplerColCount_;
  int rowCount = samplerRowCount_;
  if (colCount <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "visualizeProbability: invalid colCount=%d", colCount);
    return;
  }

  if (grids.size() % colCount != 0) {
    RCLCPP_ERROR(node_->get_logger(), "visualizeProbability: grids.size()=%zu is not divisible by colCount=%d",
                 grids.size(), colCount);
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "probability";
  marker.header.stamp = node_->now();
  marker.ns = "gaussian_plane_colored";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.orientation.w = 1.0;
  double dx = resolution_;
  double dy = resolution_;
  double xMin = xMin_, yMin = yMin_;

  const double z_scale = 100.0;
  const double gamma = 0.5;

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(colCount * rowCount);

  for (int j = 0; j < rowCount; ++j) {
    for (int i = 0; i < colCount; ++i) {
      int idx = j * colCount + i;
      double x = xMin + i * dx;
      double y = yMin + j * dy;
      double z = grids[idx].probability_ * z_scale;

      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      points.push_back(p);
    }
  }

  auto clamp = [](double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); };

  for (int j = 0; j < rowCount - 1; ++j) {
    for (int i = 0; i < colCount - 1; ++i) {
      int idx = j * colCount + i;
      int idxRight = idx + 1;
      int idxDown = idx + colCount;
      int idxDiag = idxDown + 1;

      auto addTriangle = [&](int i1, int i2, int i3) {
        bool in_region = isPointInConvexPolygon(Eigen::Vector2d(points[i1].x, points[i1].y), region_vertices_) &&
                         isPointInConvexPolygon(Eigen::Vector2d(points[i2].x, points[i2].y), region_vertices_) &&
                         isPointInConvexPolygon(Eigen::Vector2d(points[i3].x, points[i3].y), region_vertices_);
        if (!in_region) {
          return;
        }
        // vertex
        marker.points.push_back(points[i1]);
        marker.points.push_back(points[i2]);
        marker.points.push_back(points[i3]);

        for (int k : {i1, i2, i3}) {
          double raw_val = grids[k].probability_;
          double norm_val = clamp(raw_val / 0.3, 0.0, 1.0);
          double val = std::pow(norm_val, gamma);

          std_msgs::msg::ColorRGBA c;
          c.r = val;
          c.g = 1.0 - val;
          c.b = 0.1;
          c.a = 1.0;
          marker.colors.push_back(c);
        }
      };

      addTriangle(idx, idxRight, idxDown);
      addTriangle(idxRight, idxDiag, idxDown);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Preparing to publish probability marker. Marker points size: %lu",
               marker.points.size());
  if (probability_pub_) {
    probability_pub_->publish(marker);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Probability marker publisher is not initialized.");
  }
}

void GaussianPlaneVisualizer::startTimer(double period) {
  timer_ = node_->create_wall_timer(std::chrono::duration<double>(period),
                                    std::bind(&GaussianPlaneVisualizer::timerCallback, this));
}

void GaussianPlaneVisualizer::updateGrid(const std::vector<GridData> *newGridsPtr) {
  std::lock_guard<std::mutex> lock(grids_mutex_);
  if (newGridsPtr) {
    grids_ = *newGridsPtr;
  } else {
    grids_.clear();
  }
}

std::mutex &GaussianPlaneVisualizer::getGridMutex() {
  return grids_mutex_;
}

void GaussianPlaneVisualizer::timerCallback() {
  std::lock_guard<std::mutex> lock(grids_mutex_);

  if (!grids_.empty()) {
    visualizeUncertainty(grids_);
    visualizeProbability(grids_);
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "timerCallback: grids_ is empty, skipping visualization");
  }
}
