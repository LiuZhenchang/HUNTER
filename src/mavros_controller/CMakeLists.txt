cmake_minimum_required(VERSION 3.8)
project(mavros_controller)

# Compiler options
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Header directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mavros_controller.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  mavros_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# add_executable(mavros_controller_node src/mavros_controller_node.cpp)
# ament_target_dependencies(mavros_controller_node
#   rclcpp mavros_msgs geometry_msgs tf2 tf2_ros
# )
# target_link_libraries(mavros_controller_node ${PROJECT_NAME})


# Install the library and headers
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp mavros_msgs geometry_msgs nav_msgs tf2 tf2_ros)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()