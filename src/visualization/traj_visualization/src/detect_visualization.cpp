#include "detect_visualization.h"
#include <tf2/utils.h>
#include <Eigen/Core>
#include "custom_msgs/msg/communication_data.hpp"

TrajectoryVisualization::TrajectoryVisualization(const rclcpp::Node::SharedPtr &node)
    : node_(std::move(node)), sensor_(node_) {
  node_->declare_parameter("drone_count", 2);
  node_->get_parameter("drone_count", drone_count_);
  RCLCPP_INFO(node_->get_logger(), "Sensor parameters:");
  RCLCPP_INFO(node_->get_logger(), "  Map bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]", sensor_.getMapXmin(),
              sensor_.getMapXmax(), sensor_.getMapYmin(), sensor_.getMapYmax());
  RCLCPP_INFO(node_->get_logger(), "  Resolution: %.2f", sensor_.getlength());

  resolution_ = sensor_.getlength();
  map_xmin_ = sensor_.getMapXmin();
  map_ymin_ = sensor_.getMapYmin();

  auto sensor_qos = rclcpp::SensorDataQoS();

  odom_subs_.resize(drone_count_);
  for (int i = 0; i < drone_count_; i++) {
    int colorValue = 100 - i * 10;
    {
      std::string topic_name = "/drone_" + std::to_string(i) + "_communication_data";
      // odom_subs_[i] = node_->create_subscription<nav_msgs::msg::Odometry>(topic_name, 10, [this, colorValue](const
      // nav_msgs::msg::Odometry::SharedPtr msg) {
      //     this->odomCallback(msg, colorValue);
      // });
      // RCLCPP_INFO(node_->get_logger(), "Subscribed to: %s", topic_name.c_str());
      odom_subs_[i] = node_->create_subscription<custom_msgs::msg::CommunicationData>(
          topic_name, sensor_qos, [this, colorValue, i](const custom_msgs::msg::CommunicationData::SharedPtr msg) {
            this->communicationDataCallback(msg, colorValue, i);
          });
    }
  }

  // Publish occupancy grid
  std::string grid_topic;
  node_->declare_parameter<std::string>("grid_topic", "/trajectory_grid");
  node_->get_parameter("grid_topic", grid_topic);
  grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic, 1);
  RCLCPP_INFO(node_->get_logger(), "Publishing grid on: %s", grid_topic.c_str());

  // Initialize occupancy grid
  double resolution = sensor_.getlength();
  double map_length_x = sensor_.getMapXmax() - sensor_.getMapXmin();
  double map_length_y = sensor_.getMapYmax() - sensor_.getMapYmin();

  if (resolution <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid resolution: %.2f", resolution);
    resolution = 1.0;
  }

  if (map_length_x <= 0.0 || map_length_y <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid map dimensions: X=%.2f, Y=%.2f", map_length_x, map_length_y);
    map_length_x = std::max(100.0, map_length_x);
    map_length_y = std::max(100.0, map_length_y);
  }

  grid_.header.frame_id = "world";
  grid_.info.resolution = resolution;
  grid_.info.origin.position.x = sensor_.getMapXmin();
  grid_.info.origin.position.y = sensor_.getMapYmin();
  grid_.info.origin.orientation.w = 1.0;

  unsigned int width = static_cast<unsigned int>(map_length_x / resolution);
  unsigned int height = static_cast<unsigned int>(map_length_y / resolution);

  grid_.info.width = std::max(1u, width);
  grid_.info.height = std::max(1u, height);

  size_t size = grid_.info.width * grid_.info.height;
  RCLCPP_INFO(node_->get_logger(), "Creating grid of size %u x %u = %zu cells", grid_.info.width, grid_.info.height,
              size);

  try {
    grid_.data.assign(size, 0);
    RCLCPP_INFO(node_->get_logger(), "Grid successfully initialized");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to resize grid: %s", e.what());
  }

  RCLCPP_INFO(node_->get_logger(), "TrajectoryVisualization initialization complete");
}

void TrajectoryVisualization::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int colorValue) {
  updateGrid(msg, colorValue);
}

void TrajectoryVisualization::updateGrid(const nav_msgs::msg::Odometry::SharedPtr msg, int colorValue) {
  std::lock_guard<std::mutex> lock(grid_mutex_);

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double theta = tf2::getYaw(msg->pose.pose.orientation);
  Eigen::Vector2d position(x, y);
  std::vector<Eigen::Vector2i> coveredCells = sensor_.seekerForward(position, theta);

  int updatedCells = 0;
  for (const auto &cell : coveredCells) {
    int idx = cell.y() * grid_.info.width + cell.x();
    if (idx >= 0 && idx < static_cast<int>(grid_.data.size())) {
      grid_.data[idx] = colorValue;
      updatedCells++;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid cell index: %d", idx);
    }
  }
  grid_.header.stamp = node_->now();
  grid_pub_->publish(grid_);
}

void TrajectoryVisualization::communicationDataCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg,
                                                        int colorValue, int agent_id) {
  std::lock_guard<std::mutex> lock(grid_mutex_);
  if (msg->agents_history.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No agents history in message");
    return;
  }
  const auto &latest_time_step = msg->agents_history.back();
  if (latest_time_step.history.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No agent states in latest time step");
    return;
  }

  int agent_index = -1;
  for (size_t i = 0; i < latest_time_step.history.size(); ++i) {
    if (latest_time_step.history[i].agent_id == agent_id) {
      agent_index = i;
      break;
    }
  }
  if (agent_index < 0) {
    RCLCPP_WARN(node_->get_logger(), "Could not find agent with id %d, using first agent", msg->sender_id);
    return;
  }

  double world_x = latest_time_step.history[agent_index].x;
  double world_y = latest_time_step.history[agent_index].y;

  if (std::abs(world_x) < 0.001 && std::abs(world_y) < 0.001) {
    // RCLCPP_WARN(node_->get_logger(), "ERROR: grid coordinates are (0,0), skipping this callback (likely initial
    // odometry message)");
    return;
  }
  double theta = latest_time_step.history[agent_index].yaw;

  // std::cout << "agent_id: " << agent_id << ", grid(x: " << grid_x << ", y: " << grid_y << "), world(x: " << world_x
  // << ", y: " << world_y << "), theta: " << theta << std::endl; std::cout << "[CommunicationData] world(x: " <<
  // world_x << ", y: " << world_y << "), theta: " << theta << std::endl;

  Eigen::Vector2d position2d(world_x, world_y);
  std::vector<Eigen::Vector2i> coveredCells = sensor_.seekerForward(position2d, theta);

  int updatedCells = 0;
  for (const auto &cell : coveredCells) {
    int idx = cell.y() * grid_.info.width + cell.x();
    if (idx >= 0 && idx < static_cast<int>(grid_.data.size())) {
      grid_.data[idx] = colorValue;
      updatedCells++;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid cell index: %d", idx);
    }
  }

  grid_.header.stamp = node_->now();
  grid_pub_->publish(grid_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trajectory_visualization");
  auto traj = std::make_shared<TrajectoryVisualization>(node);
  (void)traj;
  RCLCPP_INFO(node->get_logger(), "Trajectory visualization node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
