cmake_minimum_required(VERSION 3.8)
project(obstacle_vis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

add_executable(obstacle_node src/obstacle_node.cpp)
ament_target_dependencies(obstacle_node
  rclcpp
  std_msgs
  visualization_msgs
)
rosidl_target_interfaces(obstacle_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(obstacle_visualization src/obstacle_visualization.cpp)
ament_target_dependencies(obstacle_visualization
  rclcpp
  std_msgs
  visualization_msgs
)
rosidl_target_interfaces(obstacle_visualization
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  obstacle_visualization
  obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()