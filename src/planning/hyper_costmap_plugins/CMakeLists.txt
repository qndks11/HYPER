cmake_minimum_required(VERSION 3.8)
project(hyper_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# The library name here is what costmap_plugins.xml's <library path=...> must say, and what
# pluginlib dlopen()s at costmap startup. They have to match or the layer fails to load with a
# "could not find library" that names neither file.
add_library(hyper_costmap_layers SHARED
  src/drivable_area_layer.cpp
)
target_include_directories(hyper_costmap_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hyper_costmap_layers
  rclcpp nav_msgs nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS hyper_costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hyper_costmap_layers)
ament_export_dependencies(nav2_costmap_2d pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
