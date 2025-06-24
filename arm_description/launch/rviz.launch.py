"""
RViz Launch File

Author: Hunter Ellis
Date: 3-29-25
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # package path
    desc_pkg_path = get_package_share_directory('arm_description')

    # configuration path
    rviz_config = os.path.join(
        desc_pkg_path,
        'rviz',
        'arm_description.rviz'
    )

    # config launch argument
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to the config file'
    )

    # sim time flag used for gazebo simulation
    sim_time = LaunchConfiguration('sim_time')
    declare_sim_time = DeclareLaunchArgument(
        name='sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': sim_time}],
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        rviz_arg,
        declare_sim_time,
        rviz
    ])
