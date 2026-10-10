cmake_minimum_required(VERSION 3.8)
project(dmpc)

# -----------------------------
# Compiler options
# -----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(search_env REQUIRED)
find_package(Eigen3 REQUIRED)

# -----------------------------
# openGA source files
# -----------------------------
set(OPEN_GA_DIR "../../src/dmpc/openGA/src")
set(CVT_DIR "../../src/dmpc/include")
file(GLOB OPEN_GA_SOURCES "${OPEN_GA_DIR}/*.cpp")  # glob all cpp files

# -----------------------------
# Header paths
# -----------------------------
set(PROJECT_MAIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
  ${PROJECT_MAIN_INCLUDE_DIR}
  ${OPEN_GA_DIR}         # openGA headers
  ${CVT_DIR}
)

# -----------------------------
# Build the DMPC library
# -----------------------------
add_library(${PROJECT_NAME}
  src/dmpc.cpp
  src/cvt_system_polygon.cpp
  ${OPEN_GA_SOURCES}   # compile openGA sources
)

target_include_directories(${PROJECT_NAME}
PUBLIC
    $<BUILD_INTERFACE:${PROJECT_MAIN_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  search_env
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)

# -----------------------------
# Export targets and dependencies
# -----------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  search_env
)

ament_package()
