#!/usr/bin/env python3
"""
Arm Control Publisher

This creates a node to send basic joint positions to the ROS2 Controller

Author: Hunter Elis
Date: 4-02-25
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header

joints = ['J1',
          'J2',
          'J3',
          'J4',
          'J5',
          'J6']

class JointTrajectoryPublisher(Node):
    def __init__(self):

        super().__init__('joint_trajectory_publisher')
        self.get_logger().info('initializing pub...')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            1
        )

        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.duration_sec = 2
        self.duration_nanosec = 0.5 * 1e9

        self.arm_poses = []
        self.arm_poses.append([3.1415, 1.5708, -1.5708, 0.0, 0.0, 0.0])  # straight
        self.arm_poses.append([0.0, 0.0, 0.0, 0.0, 0.0])  # home

        self.index = 0

    def timer_callback(self):
        print('callback\n')
        msg = JointTrajectory()
        msg.header = Header()
        msg.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = self.arm_poses[self.index]
        point.time_from_start = Duration(
            sec=int(self.duration_sec),
            nanosec=int(self.duration_nanosec)
        )

        msg.points.append(point)
        self.trajectory_pub.publish(msg)

        self.get_logger().info(
            '\nJ1: ' + str(self.arm_poses[self.index][0]) +
            '\nJ2: ' + str(self.arm_poses[self.index][1]) +
            '\nJ3: ' + str(self.arm_poses[self.index][2]) +
            '\nJ4: ' + str(self.arm_poses[self.index][3]) +
            '\nJ5: ' + str(self.arm_poses[self.index][4]) +
            '\nJ5: ' + str(self.arm_poses[self.index][5]) +
            '\n'
        )

        self.index = (self.index + 1) % 2


def main(args=None):
    rclpy.init(args=args)

    joint_trajectory_pub = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_pub)

    joint_trajectory_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
