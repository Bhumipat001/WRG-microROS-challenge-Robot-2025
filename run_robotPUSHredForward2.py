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

        time.sleep(1)  # Wait 1 second to ensure publisher is ready
        self.move_robot()

    def move_robot(self):
        twist = Twist()

        # ───── First push forward ─────
        # Move forward at 0.2 m/s for (30 x 0.05s = 1.5 seconds)
        for ii in range(70):  # 30 iterations
            twist.linear.x = 0.2           # move forward at 0.2 m/s
            twist.angular.z = 0.0          # no rotation
            self.get_logger().info("Moving forward")
            self.send_cmd(twist, 0.05)     # send for 0.05s each
            
        # ───── Stop ─────
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Stopping")
        self.publisher_.publish(twist)

    def send_cmd(self, twist_msg, duration):
        start_time = time.time()
        rate = 0.1  # seconds between commands (10 Hz)
        while time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(rate)  # sleep for rate seconds

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()