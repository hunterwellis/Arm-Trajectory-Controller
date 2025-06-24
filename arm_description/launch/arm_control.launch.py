from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_trajectory_controller = Node(
        package='arm_description',
        executable='trajectory_control.py',
        name='joint_trajectory_controller'
    )

    print('loading controller... \n')

    return LaunchDescription([
        joint_trajectory_controller
    ])
