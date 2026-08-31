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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)


generate_parameter_library(mission_manager_parameters
  src/mission_manager_parameters.yaml)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs std_msgs std_srvs tf2 tf2_ros)
target_link_libraries(mission_manager_node mission_manager_parameters yaml-cpp)

add_executable(cmd_vel_to_ackermann_node src/cmd_vel_to_ackermann_node.cpp)
ament_target_dependencies(cmd_vel_to_ackermann_node
  rclcpp geometry_msgs std_msgs)

install(TARGETS mission_manager_node cmd_vel_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
