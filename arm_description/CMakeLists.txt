cmake_minimum_required(VERSION 3.5)
project(arm_description)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# install(TARGETS
#     joint_publisher
#     DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY 
  model 
  rviz
  launch
  control
  DESTINATION
  share/${PROJECT_NAME}
)

install(
  PROGRAMS
  control/arm_control.py
  control/trajectory_control.py
  DESTINATION
  lib/${PROJECT_NAME}/
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
