#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "target_vis/msg/target.hpp"
#include "target_vis/msg/target_array.hpp"

class TargetMoveNode : public rclcpp::Node {
 public:
  TargetMoveNode() : rclcpp::Node("target_move") {
    declareParameters();
    publisher_ = this->create_publisher<target_vis::msg::TargetArray>("target_info", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetMoveNode::publishTargets, this));
  }

 private:
  void declareParameters() {
    target_num_ = this->declare_parameter<int>("target_num", 3);
    target_x_.resize(target_num_);
    target_y_.resize(target_num_);

    for (int i = 0; i < target_num_; ++i) {
      std::string param_x = "target" + std::to_string(i) + "_x";
      std::string param_y = "target" + std::to_string(i) + "_y";

      double default_x = -1050.0 + i * 100.0;
      double default_y = 1050.0 - i * 750.0;

      target_x_[i] = this->declare_parameter<double>(param_x, default_x);
      target_y_[i] = this->declare_parameter<double>(param_y, default_y);

      RCLCPP_INFO(this->get_logger(), "Target %d: x=%.1f, y=%.1f", i, target_x_[i], target_y_[i]);
    }
  }

  void publishTargets() {
    target_vis::msg::TargetArray target_array;
    target_array.targets.resize(target_num_);
    const double stamp = this->now().seconds();

    for (int i = 0; i < target_num_; ++i) {
      target_array.targets[i].position_x = target_x_[i];
      target_array.targets[i].position_y = target_y_[i];
      target_array.targets[i].grid_x = 3.0 + i * 4.0;
      target_array.targets[i].grid_y = 4.0 + i * 4.0;
      target_array.targets[i].time_stamp = stamp;
    }

    publisher_->publish(target_array);
  }

  rclcpp::Publisher<target_vis::msg::TargetArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int target_num_;
  std::vector<double> target_x_;
  std::vector<double> target_y_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetMoveNode>());
  rclcpp::shutdown();
  return 0;
}
