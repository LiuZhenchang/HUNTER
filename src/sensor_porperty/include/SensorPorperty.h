#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

struct Grid {
  double uncertainty_;  // environment uncertainty
  double probability_;  // target existence probability
  int detect_num_;
};

class SensorPorperty {
 public:
  SensorPorperty(rclcpp::Node::SharedPtr node);
  int getType();
  double getBias();
  double getCoverLength();
  double getCoverWidth();
  std::vector<Eigen::Vector2i> seekerForward(const Eigen::Vector2d &position, double theta);
  double estEnvValue(std::vector<Grid> &grids, const Eigen::Vector2d &position, double theta);
  double estTargetValue(std::vector<Grid> &grids, const Eigen::Vector2d &position, double theta);
  double getMapXmin();
  double getMapXmax();
  double getMapYmin();
  double getMapYmax();
  double getlength();
  typedef std::shared_ptr<SensorPorperty> Ptr;

 private:
  int seeker_type_;
  double bias_;
  double cover_length_;
  double cover_width_;
  double Map_xmax_;
  double Map_ymax_;
  double Map_xmin_;
  double Map_ymin_;

  double length_;

  rclcpp::Node::SharedPtr node_;  // ROS2 Node pointer
};
