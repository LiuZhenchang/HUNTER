#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

struct Point {
  double x, y;

  Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}

  double distance(const Point &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }

  Eigen::Vector2d toVector2d() const {
    return Eigen::Vector2d(x, y);
  }

  friend std::ostream &operator<<(std::ostream &os, const Point &p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
  }
};

class WeightedCVTPolygon {
 public:
  std::vector<Point> drones_;
 private:
  double region_size_;
  std::vector<Point> target_points_;
  double grid_resolution_;
  double grid_step_; 
  double mask_radius_; 
  std::vector<Point> valid_grid_points_;
  std::vector<Eigen::Vector2d> polygon_vertices_;

  bool isPointInConvexPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices) const;
  void preprocessValidGridPoints();
  double calculateWeight(double x, double y) const;
  Eigen::Vector2d constrainToPolygon(const Eigen::Vector2d &point) const;
  Eigen::Vector2d projectToPolygonBoundary(const Eigen::Vector2d &point) const;

 public:
  WeightedCVTPolygon(const std::vector<Point> &target_points = {}, int num_drones = 3, int grid_res = 80,
                     double mask_radius = 50.0, const std::vector<Eigen::Vector2d> &polygon_vertices = {});
  void setInitialDrones(const std::vector<Point> &initial);
  void computeCVT(int iterations = 20);
  const std::vector<Point> &getDrones() const {return drones_;}
  size_t getValidGridPointsCount() const {return valid_grid_points_.size();}
  void printResults() const;
};
