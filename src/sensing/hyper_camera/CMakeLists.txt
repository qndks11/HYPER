cmake_minimum_required(VERSION 3.8)
project(hyper_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# The vehicle's one camera. Built as a component so hyper_lane_detection can load it into its own
# ComposableNodeContainer for zero-copy intra-process delivery (input_backend:=intra_process);
# object_detection_node picks the same frames up off the published topic. See
# hyper_object_detection's perception.launch.py for the container that loads both.
add_library(logitech_camera_publisher_component SHARED
  src/logitech_camera_publisher_node.cpp
)
target_include_directories(logitech_camera_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(logitech_camera_publisher_component
  rclcpp rclcpp_components sensor_msgs cv_bridge OpenCV)
rclcpp_components_register_node(logitech_camera_publisher_component
  PLUGIN "hyper_camera::LogitechCameraPublisherNode"
  EXECUTABLE logitech_camera_publisher_node)

install(TARGETS logitech_camera_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
