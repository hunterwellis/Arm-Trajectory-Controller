"""
Serial Port Launch File

This Launch file starts a serial connection between the arm hardware and ROS2.

Author: Hunter Ellis
Date: 11-25-2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    serial_port_sub = Node(
        pacages='arm_interface',
        executable='SerialNode',
        name='serial_subscriber'
    )

    return LaunchDescription([
        serial_port_sub
    ])
