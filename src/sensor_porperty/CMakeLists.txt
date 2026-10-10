cmake_minimum_required(VERSION 3.8)
project(sensor_porperty)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Create the library
add_library(sensor_porperty
    src/SensorPorperty.cpp
)

ament_target_dependencies(sensor_porperty
    rclcpp
)

target_link_libraries(sensor_porperty
    Eigen3::Eigen
)

# Install the library and headers
install(TARGETS sensor_porperty
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(sensor_porperty)
ament_export_dependencies(rclcpp)

ament_package()