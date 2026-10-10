cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/TargetInfo.msg"
  "msg/AgentState.msg"
  "msg/AgentStateList.msg"
  "msg/CommunicationData.msg"
  "msg/ObjectLocation.msg"
  "msg/ObjectComputation.msg"
  "msg/PredictTrajectory.msg"
  "msg/PredictState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()