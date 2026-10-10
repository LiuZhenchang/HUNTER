cmake_minimum_required(VERSION 3.8)
project(grid_data_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(grid_data_visualization
    src/GridData_visualiztion.cpp
)

ament_target_dependencies(grid_data_visualization
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    Eigen3
)

target_include_directories(grid_data_visualization
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(grid_data_visualization_node
    src/grid_data_visualization_node.cpp
)

ament_target_dependencies(grid_data_visualization_node
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    Eigen3
)

target_link_libraries(grid_data_visualization_node
    grid_data_visualization
)

install(TARGETS 
    grid_data_visualization
    grid_data_visualization_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    Eigen3
)

ament_package()
