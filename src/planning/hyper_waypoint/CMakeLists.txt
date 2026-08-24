cmake_minimum_required(VERSION 3.8)
project(hyper_waypoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(waypoint_recorder_node src/waypoint_recorder_node.cpp)
ament_target_dependencies(waypoint_recorder_node
  rclcpp geometry_msgs nav_msgs sensor_msgs diagnostic_msgs)

install(TARGETS waypoint_recorder_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
