cmake_minimum_required(VERSION 3.8)
project(hyper_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/imu_enu_relay.py
    scripts/gps_heading.py
    scripts/gps_accuracy_gui.py
    scripts/rtk_heading_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
