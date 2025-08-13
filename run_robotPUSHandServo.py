#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int32, '/servo_s2', 10)
        self.get_logger().info('Simple Mover Initialized')

        self.servo_angle = -60  # Starting at default position (you can adjust)
        time.sleep(1)  # Give time for publishers to connect
        self.move_robot()

    def move_robot(self):
        twist = Twist()

        # First: move forward
        for ii in range(30):
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Moving forward")
            self.send_cmd(twist, 0.05)

        # Stop robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Stopping")
        self.publisher_.publish(twist)

        # Then: move servo2 +90°
        self.move_servo_relative(90)
        time.sleep(1)

        # Then: move servo2 -90° back
        self.move_servo_relative(-90)
        time.sleep(1)

    def send_cmd(self, twist_msg, duration):
        start_time = time.time()
        rate = 0.1  # seconds (10 Hz)
        while time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(rate)

    def move_servo_relative(self, delta):
        new_angle = self.servo_angle + delta
        # Limit for servo2: -90 to +20 (based on Yahboom docs)
        new_angle = max(-90, min(20, new_angle))

        msg = Int32()
        msg.data = new_angle
        self.servo_pub.publish(msg)
        self.get_logger().info(f"Set servo2 to {new_angle}° (Δ{delta})")

        self.servo_angle = new_angle

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()