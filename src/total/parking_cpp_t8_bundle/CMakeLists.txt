cmake_minimum_required(VERSION 3.8)
project(parking_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(parking_scan_costmap src/parking_scan_costmap.cpp)
ament_target_dependencies(parking_scan_costmap
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(hybrid_astar_planner src/hybrid_astar_planner.cpp)
ament_target_dependencies(hybrid_astar_planner
  rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)

add_executable(controller_with_parking src/controller_with_parking.cpp)
ament_target_dependencies(controller_with_parking
  rclcpp ackermann_msgs geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs)

add_executable(behavior_supervisor_with_parking src/behavior_supervisor_with_parking.cpp)
ament_target_dependencies(behavior_supervisor_with_parking
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(behavior_supervisor_with_parking yaml-cpp)

install(TARGETS
  parking_scan_costmap
  hybrid_astar_planner
  controller_with_parking
  behavior_supervisor_with_parking
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
