#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

std::vector<double> points_param;
std::vector<std::vector<double>> no_fly_zones_param;
bool enable_no_fly_zone;
double line_width;

const std::vector<std::tuple<double, double, double>> zone_colors = {
    {0.0, 0.0, 1.0},  // blue
    {1.0, 0.0, 0.0},  // red
    {0.0, 1.0, 0.0},  // green
    {1.0, 1.0, 0.0},  // yellow
    {1.0, 0.0, 1.0},  // pink
    {0.0, 1.0, 1.0},  // cyan
    {0.5, 0.5, 0.5},  // gray
    {1.0, 0.5, 0.0},  // orange
};

std::vector<std::vector<double>> parseNoFlyZonesString(const std::string &zones_str) {
  std::vector<std::vector<double>> result;
  if (zones_str.empty()) {
    return result;
  }

  std::vector<double> current_zone;
  std::string num_str;

  auto processNumber = [&]() {
    if (!num_str.empty()) {
      try {
        current_zone.push_back(std::stod(num_str));
      } catch (...) {
      }
      num_str.clear();
    }
  };

  for (char c : zones_str) {
    if (c == ';') {
      processNumber();
      if (!current_zone.empty()) {
        result.push_back(current_zone);
        current_zone.clear();
      }
    } else if (c == ',') {
      processNumber();
    } else if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
      num_str += c;
    }
  }
  processNumber();
  if (!current_zone.empty()) {
    result.push_back(current_zone);
  }

  return result;
}

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr no_fly_zone_publisher_;
rclcpp::TimerBase::SharedPtr timer_;

class PolygonVisualizer : public rclcpp::Node {
 public:
  PolygonVisualizer() : Node("polygon_visualizer") {
    this->declare_parameter<std::vector<double>>("polygon_points", std::vector<double>());
    this->declare_parameter<double>("line_width", 2.0);
    this->declare_parameter<std::string>("no_fly_zones", "[]");
    this->declare_parameter<bool>("enable_no_fly_zone", false);

    points_param = this->get_parameter("polygon_points").as_double_array();
    std::string no_fly_zones_str = this->get_parameter("no_fly_zones").as_string();
    no_fly_zones_param = parseNoFlyZonesString(no_fly_zones_str);
    enable_no_fly_zone = this->get_parameter("enable_no_fly_zone").as_bool();
    line_width = this->get_parameter("line_width").as_double();

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("polygon_marker", 10);
    no_fly_zone_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("no_fly_zone_marker", 10);

    if (no_fly_zones_param.size() >= 2 && enable_no_fly_zone) {
      RCLCPP_INFO(this->get_logger(), "No-fly zone visualization enabled with %zu zones", no_fly_zones_param.size());
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&PolygonVisualizer::timerCallback, this));
  }

 private:
  void timerCallback() {
    if (points_param.size() >= 6 && points_param.size() % 2 == 0) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = now();
      marker.ns = "polygon";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = line_width;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      for (size_t i = 0; i < points_param.size(); i += 2) {
        geometry_msgs::msg::Point pt;
        pt.x = points_param[i];
        pt.y = points_param[i + 1];
        pt.z = 0.1;
        marker.points.push_back(pt);
      }

      // close polygon: end point == start point
      {
        geometry_msgs::msg::Point first;
        first.x = points_param[0];
        first.y = points_param[1];
        first.z = 0.1;
        marker.points.push_back(first);
      }

      publisher_->publish(marker);
    }

    if (!enable_no_fly_zone || no_fly_zones_param.empty()) {
      return;
    }

    for (size_t zone_idx = 0; zone_idx < no_fly_zones_param.size(); ++zone_idx) {
      const auto &zone_vertices = no_fly_zones_param[zone_idx];

      // Validate vertex data
      if (zone_vertices.size() < 6 || zone_vertices.size() % 2 != 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No-fly zone %zu has invalid vertices (need even number >= 6, got %zu)", zone_idx,
                             zone_vertices.size());
        continue;
      }

      visualization_msgs::msg::Marker no_fly_marker;
      no_fly_marker.header.frame_id = "world";
      no_fly_marker.header.stamp = now();
      no_fly_marker.ns = "no_fly_zone";
      no_fly_marker.id = zone_idx;
      no_fly_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      no_fly_marker.action = visualization_msgs::msg::Marker::ADD;

      no_fly_marker.scale.x = line_width;

      auto [r, g, b] = zone_colors[zone_idx % zone_colors.size()];
      no_fly_marker.color.r = r;
      no_fly_marker.color.g = g;
      no_fly_marker.color.b = b;
      no_fly_marker.color.a = 1.0;

      for (size_t i = 0; i < zone_vertices.size(); i += 2) {
        geometry_msgs::msg::Point pt;
        pt.x = zone_vertices[i];
        pt.y = zone_vertices[i + 1];
        pt.z = 0.1;
        no_fly_marker.points.push_back(pt);
      }

      // close polygon
      {
        geometry_msgs::msg::Point first_nf;
        first_nf.x = zone_vertices[0];
        first_nf.y = zone_vertices[1];
        first_nf.z = 0.1;
        no_fly_marker.points.push_back(first_nf);
      }

      no_fly_zone_publisher_->publish(no_fly_marker);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonVisualizer>());
  rclcpp::shutdown();
  return 0;
}
