cmake_minimum_required(VERSION 3.8)
project(traj_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_porperty REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(detect_visualization
  src/detect_visualization.cpp
)

ament_target_dependencies(detect_visualization
  rclcpp
  nav_msgs
  sensor_porperty
  custom_msgs
  tf2
)

install(TARGETS
  detect_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()