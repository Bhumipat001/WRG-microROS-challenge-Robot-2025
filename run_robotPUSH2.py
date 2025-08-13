#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Simple Mover Initialized')

        time.sleep(1)  # Give time for the publisher to establish connection
        self.move_robot()

    def move_robot(self):
        twist = Twist()
        
        #second push
        for ii in range(60):
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Moving forward")
            self.send_cmd(twist, 0.1)

        for ii in range(19):#11 about=45 deg
            twist.linear.x = 0.0
            twist.angular.z = 1.0
            self.get_logger().info("Turning right")
            self.send_cmd(twist, 0.05)
            
        for ii in range(50):
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Moving forward")
            self.send_cmd(twist, 0.1)
            
        for ii in range(40):
            twist.linear.x = -0.2
            twist.angular.z = 0.0
            self.get_logger().info("Moving forward")
            self.send_cmd(twist, 0.1)
            
        for ii in range(24):
            twist.linear.x = 0.0
            twist.angular.z = -1.0
            self.get_logger().info("Turning right")
            self.send_cmd(twist, 0.1)
            
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Stopping")
        self.publisher_.publish(twist)

    def send_cmd(self, twist_msg, duration):
        start_time = time.time()
        rate = 0.1  # seconds (10 Hz)
        while time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(rate)

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()