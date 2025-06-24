#include <iostream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

using moveit::planning_interface::MoveGroupInterface;

void goToNamedTarget(MoveGroupInterface &group, const std::string &target_name, rclcpp::Logger logger) {
  RCLCPP_INFO(logger, "Moving to %s", target_name.c_str());
  group.setNamedTarget(target_name);
  group.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group.plan(plan)) {
    group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Failed to plan to %s", target_name.c_str());
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place_node");

  auto logger = rclcpp::get_logger("pick_place_node");

  MoveGroupInterface arm(node, "ros_arm");
  MoveGroupInterface gripper(node, "end_effector");

  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  gripper.setMaxVelocityScalingFactor(0.5);
  gripper.setMaxAccelerationScalingFactor(0.5);

  std::string input = "";
  // Arm motion
  goToNamedTarget(arm, "rl_prep", logger);

  std::cout << "================================================================================" << std::endl;
  std::cout << "Waiting for input: " << std::endl;
  std::cin >> input;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::this_thread::sleep_for(std::chrono::seconds(3));

  goToNamedTarget(arm, "pick", logger);

  // Close gripper (update joint values based on your robot)
  goToNamedTarget(gripper, "close", logger); // JLeft, JRight closed

  goToNamedTarget(arm, "rl_prep", logger);

  std::cout << "================================================================================" << std::endl;
  std::cout << "Waiting for input: " << std::endl;
  std::cin >> input;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::this_thread::sleep_for(std::chrono::seconds(3));


  goToNamedTarget(arm, "place", logger);

  // Open gripper
  goToNamedTarget(gripper, "open", logger); // JLeft, JRight open

  goToNamedTarget(arm, "rl_prep", logger);

  std::cout << "================================================================================" << std::endl;
  std::cout << "Waiting for input: " << std::endl;
  std::cin >> input;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::this_thread::sleep_for(std::chrono::seconds(3));


  rclcpp::shutdown();
  return 0;
}
