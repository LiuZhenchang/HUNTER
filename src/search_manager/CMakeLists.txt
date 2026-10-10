cmake_minimum_required(VERSION 3.8)
project(search_manager)

# -----------------------------
# Compiler options
# -----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# add_compile_options(-O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined)
# add_link_options(-fsanitize=address,undefined)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(search_env REQUIRED)
find_package(dmpc REQUIRED)
find_package(mavros_controller REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

find_package(grid_data_visualization REQUIRED)


# -----------------------------
# Header paths
# -----------------------------
set(OPEN_GA_DIR "../../src/dmpc/openGA/src")
set(PROJECT_MAIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
  ${PROJECT_MAIN_INCLUDE_DIR}
  ${OPEN_GA_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

# -----------------------------
# -----------------------------
add_executable(${PROJECT_NAME}
  src/search_manager.cpp
  src/search_node.cpp
)

# -----------------------------
# -----------------------------
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_MAIN_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${mavros_controller_INCLUDE_DIRS}
)

# -----------------------------
# -----------------------------
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  grid_data_visualization::grid_data_visualization
)


# -----------------------------
# Link dependencies
# -----------------------------
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  search_env
  dmpc
  mavros_controller
  custom_msgs
  quadrotor_msgs
)


# -----------------------------
# Install
# -----------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
