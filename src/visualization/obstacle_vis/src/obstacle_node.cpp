#include <rclcpp/rclcpp.hpp>
#include "obstacle_vis/msg/obstacle_array.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("obstacle_node");

  auto obstacle_pub = node->create_publisher<obstacle_vis::msg::ObstacleArray>("obstacle_info", 10);

  obstacle_vis::msg::Obstacle obs1;
  obs1.position_x = 15.0;
  obs1.position_y = 15.0;
  obs1.radius = 2.5;
  obs1.time_stamp = node->now().seconds();

  obstacle_vis::msg::Obstacle obs2;
  obs2.position_x = -10.0;
  obs2.position_y = -5.0;
  obs2.radius = 2.5;
  obs2.time_stamp = node->now().seconds();

  obstacle_vis::msg::Obstacle obs3;
  obs3.position_x = 5.0;
  obs3.position_y = 5.0;
  obs3.radius = 2.5;
  obs3.time_stamp = node->now().seconds();

  obstacle_vis::msg::Obstacle obs4;
  obs4.position_x = 10.0;
  obs4.position_y = -5.0;
  obs4.radius = 2.5;
  obs4.time_stamp = node->now().seconds();

  obstacle_vis::msg::Obstacle obs5;
  obs5.position_x = -10.0;
  obs5.position_y = 15.0;
  obs5.radius = 3.0;
  obs5.time_stamp = node->now().seconds();

  obstacle_vis::msg::ObstacleArray obstacle_array;
  obstacle_array.obstacles.push_back(obs1);
  obstacle_array.obstacles.push_back(obs2);
  obstacle_array.obstacles.push_back(obs3);
  obstacle_array.obstacles.push_back(obs4);
  obstacle_array.obstacles.push_back(obs5);

  // Publishobstaclesinformation
  obstacle_pub->publish(obstacle_array);

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    // Update the timestamp
    for (auto &obs : obstacle_array.obstacles) {
      obs.time_stamp = node->now().seconds();
    }

    // Publishobstaclesinformation
    obstacle_pub->publish(obstacle_array);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}