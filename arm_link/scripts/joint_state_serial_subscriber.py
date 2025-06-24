#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial_comm import SerialComm
import time
import numpy as np


class JointStateSerialSubscriber(Node):
    def __init__(self):
        super().__init__('jss_subscriber')

        self.serial = SerialComm()
        self.serial.open_connection()

        self.start_time = time.time()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.jss_callback,
            10
        )
        self.subscription

    def __del__(self):
        self.serial.close_connection()

    def timer(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0.1:
            self.start_time = time.time()
            return True
        else:
            return False

    def sign_bit(self, num):
        if np.sign(num) == -1:
            return "1"
        else:
            return "0"

    def jss_callback(self, msg):
        # self.get_logger().info('Received Joint State:')
        # self.get_logger().info(f'Names: {msg.name}')
        # self.get_logger().info(f'Positions: {msg.position}')
        # self.get_logger().info(f'Velocities: {msg.velocity}')
        # self.get_logger().info(f'Efforts: {msg.effort}')

        # self.get_logger().info('Serial Output')
        if self.timer():
            j1_pos = self.sign_bit(msg.position[0]) + \
                str(round(1000*abs(msg.position[0]))).zfill(4)
            j2_pos = self.sign_bit(msg.position[1]) + \
                str(round(1000*abs(msg.position[1]))).zfill(4)
            j3_pos = self.sign_bit(msg.position[2]) + \
                str(round(1000*abs(msg.position[2]))).zfill(4)
            j4_pos = self.sign_bit(msg.position[3]) + \
                str(round(1000*abs(msg.position[3]))).zfill(4)

            serial_msg = j1_pos + j2_pos + j3_pos + j4_pos + "\n"

            self.get_logger().info(serial_msg)
            self.serial.send_message(serial_msg)


def main(args=None):
    rclpy.init(args=args)

    jss_subscriber = JointStateSerialSubscriber()

    rclpy.spin(jss_subscriber)

    jss_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
