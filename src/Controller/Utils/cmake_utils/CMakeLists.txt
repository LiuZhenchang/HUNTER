cmake_minimum_required(VERSION 3.8)
project(cmake_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

file(GLOB CMAKE_UILTS_FILES cmake/*.cmake)

ament_package(CONFIG_EXTRAS ${CMAKE_UTILS_FILES})
