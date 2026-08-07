cmake_minimum_required(VERSION 3.8)
project(hyper_waypoint_tracker)

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
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(waypoint_tracker_node src/waypoint_tracker_node.cpp)
ament_target_dependencies(waypoint_tracker_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(waypoint_tracker_node yaml-cpp)

install(TARGETS waypoint_tracker_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
