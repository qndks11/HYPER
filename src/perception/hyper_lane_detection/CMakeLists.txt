cmake_minimum_required(VERSION 3.8)
project(hyper_lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Built as a component (see rclcpp_components_register_node below) so it can be loaded into the
# same ComposableNodeContainer as hyper_camera's LogitechCameraPublisherNode for zero-copy
# intra-process delivery under input_backend:=intra_process -- see
# hyper_object_detection/launch/perception.launch.py. rclcpp_components_register_node also
# generates the plain `lane_detection_node` executable used for input_backend:=ros_raw.
add_library(lane_detection_component SHARED
  src/lane_detection_node.cpp
  src/ground_projection.cpp
  src/drivable_area.cpp
  src/lane_detector.cpp
  src/stopline_detector.cpp
  src/input_backend.cpp
)
target_include_directories(lane_detection_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lane_detection_component
  rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs cv_bridge std_srvs OpenCV)
rclcpp_components_register_node(lane_detection_component
  PLUGIN "LaneDetection"
  EXECUTABLE lane_detection_node)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS lane_detection_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
