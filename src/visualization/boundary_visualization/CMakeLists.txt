cmake_minimum_required(VERSION 3.5)
project(boundary_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(boundary_visualization src/boundary_visualization.cpp)
ament_target_dependencies(boundary_visualization rclcpp std_msgs geometry_msgs visualization_msgs)

install(TARGETS boundary_visualization DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
