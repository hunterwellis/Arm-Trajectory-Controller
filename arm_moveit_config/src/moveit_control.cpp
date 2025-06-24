#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
 

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
 
  auto const node = std::make_shared<rclcpp::Node>(
    "ik_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("ik_control");
 
  RCLCPP_INFO(logger, "Initializing move group");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "ros_arm");

  RCLCPP_INFO(logger, "Setting interfaces");
 
  arm_group_interface.setPlanningPipelineId("ompl");
 
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
 
  arm_group_interface.setPlanningTime(1.0);
 
  arm_group_interface.setMaxVelocityScalingFactor(0.5);
 
  arm_group_interface.setMaxAccelerationScalingFactor(0.5);
 
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
 
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.343;
    msg.pose.position.y = 0.0153;
    msg.pose.position.z = 0.1;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();

  RCLCPP_INFO(logger, "Setting pose target");

  arm_group_interface.setPoseTarget(arm_target_pose);

  arm_group_interface.setStartStateToCurrentState();
 
  RCLCPP_INFO(logger, "Logging Result");
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Success");
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
 
  rclcpp::shutdown();
  return 0;
}
