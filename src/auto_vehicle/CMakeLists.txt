cmake_minimum_required(VERSION 3.8)
project(auto_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp std_msgs sensor_msgs)

add_executable(lane_detection src/lane_detection.cpp)
ament_target_dependencies(lane_detection rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(
  DIRECTORY launch urdf config models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  joystick_controller
  lane_detection
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_controller
)

install(PROGRAMS
  scripts/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detection
)

install(PROGRAMS
  scripts/image_saver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME image_saver
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