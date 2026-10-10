cmake_minimum_required(VERSION 3.8)
project(target_vis)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/Target.msg"
  "msg/TargetArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(target_move src/target_move.cpp)
ament_target_dependencies(target_move rclcpp)
rosidl_target_interfaces(target_move
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(target_visualization src/target_visualization.cpp)
target_include_directories(target_visualization PRIVATE include)
ament_target_dependencies(target_visualization rclcpp visualization_msgs)
rosidl_target_interfaces(target_visualization
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  target_move
  target_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()