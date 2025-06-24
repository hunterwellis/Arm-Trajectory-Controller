#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.ee_pub = self.create_publisher(JointTrajectory, '/ee_controller/joint_trajectory', 10)

        # One-shot timers
        self.arm_timer_1 = self.create_timer(1.0, self.send_arm_first_command)
        self.ee_timer = self.create_timer(3.0, self.send_ee_command)
        self.arm_timer_2 = self.create_timer(5.0, self.send_arm_second_command)

    def send_arm_first_command(self):
        self.arm_timer_1.cancel()
        msg = JointTrajectory()
        msg.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        point = JointTrajectoryPoint()
        point.positions = [-0.35, 1.2, -0.3, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        msg.points.append(point)

        self.arm_pub.publish(msg)
        self.get_logger().info('Sent arm command 1')

    def send_ee_command(self):
        self.ee_timer.cancel()
        msg = JointTrajectory()
        msg.joint_names = ['JLeft', 'JRight']
        point = JointTrajectoryPoint()
        point.positions = [0.024, 0.024]
        point.time_from_start.sec = 1
        msg.points.append(point)

        self.ee_pub.publish(msg)
        self.get_logger().info('Sent end-effector command')

    def send_arm_second_command(self):
        self.arm_timer_2.cancel()
        msg = JointTrajectory()
        msg.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        msg.points.append(point)

        self.arm_pub.publish(msg)
        self.get_logger().info('Sent arm command 2')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

