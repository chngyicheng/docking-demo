cmake_minimum_required(VERSION 3.8)
project(docking_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(docking_node
    src/docking_main.cpp
    src/docking_node.cpp
)

ament_target_dependencies(docking_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# add_executable(disturbance_node src/disturbance_node.cpp)
# ament_target_dependencies(disturbance_node
#   rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs
# )

# Install
install(TARGETS docking_node #disturbance_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/run_demo.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
