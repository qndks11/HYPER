cmake_minimum_required(VERSION 3.8)
project(hyper_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# input_backend:=intra_process's camera source: opens the ELP directly (device, rectification)
# and publishes the result as a plain sensor_msgs/Image, so it can be loaded into the same
# ComposableNodeContainer as hyper_lane_detection's LaneDetection component for zero-copy
# intra-process delivery -- see hyper_lane_detection/lane_detection_node.cpp.
add_library(elp_camera_publisher_component SHARED
  src/elp_camera_capture.cpp
  src/elp_camera_publisher_node.cpp
)
target_include_directories(elp_camera_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(elp_camera_publisher_component
  rclcpp rclcpp_components sensor_msgs cv_bridge ament_index_cpp
  camera_calibration_parsers OpenCV)
rclcpp_components_register_node(elp_camera_publisher_component
  PLUGIN "hyper_camera::ElpCameraPublisherNode"
  EXECUTABLE elp_camera_publisher_node)

install(TARGETS elp_camera_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# input_backend:=usb_camera's camera source for object_detection_node -- see
# hyper_object_detection's perception.launch.py. Plain rclpy node (no zero-copy: rclpy has no
# use_intra_process_comms equivalent to rclcpp's), so it just publishes a regular topic instead
# of being loaded into a container.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  hyper_camera/logitech_camera_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME logitech_camera_publisher_node
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
