cmake_minimum_required(VERSION 3.8)
project(search_env)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_porperty REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/search_environment.cpp
)

set(PROJECT_MAIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_MAIN_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  sensor_porperty
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  sensor_porperty
)

set(CMAKE_BUILD_TYPE Debug)

ament_package()

