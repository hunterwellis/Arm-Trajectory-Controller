#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_pkg = get_package_share_directory('arm_moveit_config')
    config_path = os.path.join(moveit_pkg, 'config')

    initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
    joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
    kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
    moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
    srdf_model_path = os.path.join(config_path, 'arm.srdf')
    pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

    moveit_config = (
        MoveItConfigsBuilder('arm', package_name='arm_moveit_config')
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'start_state': {'content': initial_positions_file_path}},
            move_group_capabilities,
        ],
    )

    return LaunchDescription([
        move_group
    ])
