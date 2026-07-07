
cmake_minimum_required(VERSION 3.8)
project(controller)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# --- 의존성 ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)   # 파이썬 노드 설치용
find_package(rclpy REQUIRED)
 
# 파이썬 패키지(controller/controller/) 설치 — __init__.py 기준
ament_python_install_package(${PROJECT_NAME})
 
# 실행 노드 설치: lib/<pkg> 에 넣어야 ros2 run 으로 부를 수 있음
install(PROGRAMS
  controller/controller.py
  DESTINATION lib/${PROJECT_NAME}
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()

