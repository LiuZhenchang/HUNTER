cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_porperty REQUIRED)
find_package(pose_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
  src/odom_visualization.cpp
)


target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${ARMADILLO_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  custom_msgs
  sensor_porperty
  pose_utils
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  custom_msgs
  sensor_porperty
  pose_utils
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Installmesh
install(DIRECTORY meshes 
  DESTINATION share/${PROJECT_NAME})

ament_package()
