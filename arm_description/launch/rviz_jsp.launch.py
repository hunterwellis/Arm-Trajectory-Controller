"""
RViz with Joint State Publisher GUI Launch File

This launch file launches the JSP GUI to control RViz.

Author: Hunter Ellis
Date: 3-29-25
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # package path
    desc_pkg_path = get_package_share_directory('arm_description')

    jsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'jsp.launch.py'
                         )
        ),
        launch_arguments={'jsp_gui': 'true'}.items()
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'arm.launch.py'
                         )
        ),
        launch_arguments={'spawn': 'false'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'rviz.launch.py'
                         )
        )
    )

    return LaunchDescription([
        jsp,
        state_publisher,
        rviz
    ])
