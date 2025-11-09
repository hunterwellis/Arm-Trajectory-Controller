cmake_minimum_required(VERSION 3.8)
project(arm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

include_directories(
  include
  src
)

# Core Library
add_library(${PROJECT_NAME}_core
  src/interface.cpp
  src/serial_port/serial_port.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Serial Port Node
add_executable(serial_node
  src/serial_port/serial_port_node.cpp
)

ament_target_dependencies(serial_node
  rclcpp
  sensor_msgs
)

target_link_libraries(serial_node arm_interface_core)

# Headless Node
add_executable(${PROJECT_NAME}_node
  src/interface_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)

target_link_libraries(serial_node arm_interface_core)


# Install
install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
    serial_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}/launch
)

# Export and package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
)

ament_package()
