cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(uav_utils REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
ament_target_dependencies(quadrotor_dynamics rclcpp)

add_executable(so3_quadrotor_simulator src/quadrotor_simulator_so3.cpp)

ament_target_dependencies(so3_quadrotor_simulator
  rclcpp
  nav_msgs
  sensor_msgs
  Eigen3
  quadrotor_msgs
  uav_utils
)
target_link_libraries(so3_quadrotor_simulator quadrotor_dynamics)

install(TARGETS so3_quadrotor_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
