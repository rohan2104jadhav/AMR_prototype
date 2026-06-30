cmake_minimum_required(VERSION 3.5)
project(nav_bot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Install directories ---
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- Install Python package ---
ament_python_install_package(scripts)

# --- Install executable scripts ---
install(
  PROGRAMS
  scripts/multi_robot_teleop.py
  scripts/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()