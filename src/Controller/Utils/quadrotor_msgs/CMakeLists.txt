cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs std_msgs rclcpp) 

include_directories(include) 
ament_export_include_directories(include)

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

ament_target_dependencies(decode_msgs 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  rclcpp
  rosidl_default_runtime
) 
ament_target_dependencies(encode_msgs 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  rclcpp
  rosidl_default_runtime
) 

find_package(rosidl_cmake REQUIRED)
rosidl_target_interfaces(decode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(encode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install the library
install(TARGETS
  decode_msgs
  encode_msgs
  DESTINATION lib
)
ament_export_libraries(encode_msgs decode_msgs)

ament_package()
