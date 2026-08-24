cmake_minimum_required(VERSION 3.8)
project(hyper_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(controller_with_parking_node src/controller_with_parking_node.cpp)
ament_target_dependencies(controller_with_parking_node
  rclcpp ackermann_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)

add_executable(behavior_supervisor_with_parking_node src/behavior_supervisor_with_parking_node.cpp)
ament_target_dependencies(behavior_supervisor_with_parking_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(behavior_supervisor_with_parking_node yaml-cpp)

add_executable(follow_path_client_node src/follow_path_client_node.cpp)
ament_target_dependencies(follow_path_client_node
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs std_srvs tf2 tf2_ros)

add_executable(cmd_vel_to_ackermann_node src/cmd_vel_to_ackermann_node.cpp)
ament_target_dependencies(cmd_vel_to_ackermann_node
  rclcpp geometry_msgs std_msgs)

install(TARGETS controller_with_parking_node behavior_supervisor_with_parking_node
    follow_path_client_node cmd_vel_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
