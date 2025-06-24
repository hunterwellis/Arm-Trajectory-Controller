"""
Joint State Publisher Link Launch File

This launch file initializes the link between hardware and ROS2 via serial.

Author: Hunter Ellis
Date: 3-29-25
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    desc_pkg_path = os.path.join(
        get_package_share_directory('arm_description'))

    rviz_jsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'rviz_jsp.launch.py'
                         )
        ),
    )

    jss = Node(
        package='arm_link',
        executable='joint_state_serial_subscriber.py',
        name='jss'
    )

    return LaunchDescription([
        rviz_jsp,
        jss
    ])
