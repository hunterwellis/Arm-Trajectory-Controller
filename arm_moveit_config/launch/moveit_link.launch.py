from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_moveit_config',
            executable='moveit_control',
            name='ik_controller_node',
            output='screen',
            parameters=[],
        )
    ])
