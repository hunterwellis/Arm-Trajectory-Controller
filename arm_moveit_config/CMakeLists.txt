cmake_minimum_required(VERSION 3.22)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(moveit_control src/moveit_control.cpp)
add_executable(moveit_return src/moveit_return.cpp)
add_executable(moveit_ready src/moveit_ready.cpp)
add_executable(moveit_seq src/moveit_seq.cpp)
ament_target_dependencies(moveit_control rclcpp moveit_ros_planning_interface geometry_msgs)
ament_target_dependencies(moveit_return rclcpp moveit_ros_planning_interface geometry_msgs)
ament_target_dependencies(moveit_ready rclcpp moveit_ros_planning_interface geometry_msgs)
ament_target_dependencies(moveit_seq rclcpp moveit_ros_planning_interface geometry_msgs)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(TARGETS
  moveit_control
  moveit_return
  moveit_ready
  moveit_seq
  DESTINATION lib/${PROJECT_NAME}
)
