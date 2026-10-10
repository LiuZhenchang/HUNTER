#include "dmpc/cvt_system_polygon.h"
#include <algorithm>
#include <ctime>
#include <random>

WeightedCVTPolygon::WeightedCVTPolygon(const std::vector<Point> &target_points, int num_drones, int grid_res,
                                       double mask_radius, const std::vector<Eigen::Vector2d> &polygon_vertices)
    : target_points_(target_points),
      grid_resolution_(grid_res),
      mask_radius_(mask_radius),
      polygon_vertices_(polygon_vertices) {
  preprocessValidGridPoints();
}

bool WeightedCVTPolygon::isPointInConvexPolygon(const Eigen::Vector2d &point,
                                                const std::vector<Eigen::Vector2d> &vertices) const {
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

void WeightedCVTPolygon::preprocessValidGridPoints() {
  valid_grid_points_.clear();

  std::vector<double> polygon_vertices_x, polygon_vertices_y;
  double resolution_ = grid_resolution_;
  Eigen::Vector3d map_min_boundary_, map_size_;
  Eigen::Vector3i map_voxel_num_;

  for (size_t i = 0; i < polygon_vertices_.size(); i++) {
    polygon_vertices_x.push_back(polygon_vertices_[i].x());
    polygon_vertices_y.push_back(polygon_vertices_[i].y());
  }

  // Compute the bounding rectangle
  if (!polygon_vertices_x.empty() && !polygon_vertices_y.empty()) {
    double min_x = *std::min_element(polygon_vertices_x.begin(), polygon_vertices_x.end());
    double max_x = *std::max_element(polygon_vertices_x.begin(), polygon_vertices_x.end());
    double min_y = *std::min_element(polygon_vertices_y.begin(), polygon_vertices_y.end());
    double max_y = *std::max_element(polygon_vertices_y.begin(), polygon_vertices_y.end());

    map_min_boundary_ = Eigen::Vector3d(min_x, min_y, 0);
    map_size_ = Eigen::Vector3d(max_x - min_x, max_y - min_y, 0);
    map_voxel_num_ = Eigen::Vector3i(ceil(map_size_(0) / resolution_), ceil(map_size_(1) / resolution_),
                                     ceil(map_size_(2) / resolution_));

    std::cout << "DMPC::Convex region bounding box: X[%.2f, %.2f], Y[%.2f, %.2f]", min_x, max_x, min_y, max_y;
  }

  int grid_count = map_voxel_num_(0) * map_voxel_num_(1);

  for (size_t i = 0; i < grid_count; ++i) {
    Eigen::Vector3i index;
    Eigen::Vector3d position;
    Eigen::Vector3i id;
    Eigen::Vector3d pos;

    id[0] = i % map_voxel_num_(0);
    id[1] = i / map_voxel_num_(0);
    id[2] = 0;
    for (int i = 0; i < 3; ++i) {
      pos(i) = id(i) * resolution_ + map_min_boundary_(i);
    }
    double x = pos(0);
    double y = pos(1);
    Eigen::Vector2d position_2d(pos(0), pos(1));
    if (isPointInConvexPolygon(position_2d, polygon_vertices_)) {
      valid_grid_points_.emplace_back(x, y);
    }
  }

  std::cout << "Preprocessing complete: generated " << valid_grid_points_.size() << " valid grid points" << std::endl;
}

void WeightedCVTPolygon::setInitialDrones(const std::vector<Point> &initial) {
  drones_.clear();
  for (const auto &p : initial)
    drones_.push_back(p);
}

double WeightedCVTPolygon::calculateWeight(double x, double y) const {
  for (const auto &target_point_ : target_points_) {
    double dist_sq = std::pow(x - target_point_.x, 2) + std::pow(y - target_point_.y, 2);
    if (dist_sq < std::pow(mask_radius_, 2)) {
      return 0;
    }
  }
  return 1.0;
}

Eigen::Vector2d WeightedCVTPolygon::constrainToPolygon(const Eigen::Vector2d &point) const {
  if (isPointInConvexPolygon(point, polygon_vertices_)) {
    return point;
  }

  return projectToPolygonBoundary(point);
}

Eigen::Vector2d WeightedCVTPolygon::projectToPolygonBoundary(const Eigen::Vector2d &point) const {
  Eigen::Vector2d closest_vertex = polygon_vertices_[0];
  double min_distance = (point - closest_vertex).norm();

  for (const auto &vertex : polygon_vertices_) {
    double distance = (point - vertex).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_vertex = vertex;
    }
  }

  return closest_vertex;
}

void WeightedCVTPolygon::computeCVT(int iterations) {
  if (drones_.empty()) {
    std::cout << "No explorer UAVs require assignment; skipping CVT computation" << std::endl;
    return;
  }
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<double> sum_wx(drones_.size(), 0.0);
    std::vector<double> sum_wy(drones_.size(), 0.0);
    std::vector<double> sum_w(drones_.size(), 0.0);

    for (const auto &grid_point : valid_grid_points_) {
      int best_idx = 0;
      double min_dist_sq = 1e18;
      for (size_t k = 0; k < drones_.size(); ++k) {
        double d2 = std::pow(grid_point.x - drones_[k].x, 2) + std::pow(grid_point.y - drones_[k].y, 2);
        if (d2 < min_dist_sq) {
          min_dist_sq = d2;
          best_idx = k;
        }
      }

      double w = calculateWeight(grid_point.x, grid_point.y);
      sum_wx[best_idx] += grid_point.x * w;
      sum_wy[best_idx] += grid_point.y * w;
      sum_w[best_idx] += w;
    }

    for (size_t k = 0; k < drones_.size(); ++k) {
      if (sum_w[k] > 0) {
        double new_x = sum_wx[k] / sum_w[k];
        double new_y = sum_wy[k] / sum_w[k];

        Eigen::Vector2d new_pos = constrainToPolygon(Eigen::Vector2d(new_x, new_y));
        drones_[k].x = new_pos.x();
        drones_[k].y = new_pos.y();
      }
    }

    if ((iter + 1) % 5 == 0) {
      std::cout << "Completed iteration " << (iter + 1) << std::endl;
    }
  }
}

void WeightedCVTPolygon::printResults() const {
  std::cout << "\n=== Weighted CVT results for the convex polygon region ===" << std::endl;
  std::cout << "Polygon vertices:" << std::endl;
  for (size_t i = 0; i < polygon_vertices_.size(); ++i) {
    std::cout << "  Vertex " << (i + 1) << ": (" << polygon_vertices_[i].x() << ", " << polygon_vertices_[i].y() << ")"
              << std::endl;
  }
  std::cout << "Mask radius: " << mask_radius_ << "m" << std::endl;
  std::cout << "Valid grid point count: " << valid_grid_points_.size() << std::endl;
  std::cout << "\nFinal UAV positions:" << std::endl;

  for (size_t i = 0; i < drones_.size(); ++i) {
    std::cout << "UAV " << (i + 1) << " centroid target: " << drones_[i] << std::endl;
  }
}
