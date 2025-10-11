#!/usr/bin/env python3
"""
Simple keyboard teleop for Ackermann drive
w/s: increase/decrease speed
a/d: left/right steering
space: stop
q: quit
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import tty
import termios


class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.speed = 0.0
        self.steering = 0.0
        self.max_speed = 5.0
        self.max_steering = 0.4
        self.speed_step = 0.5
        self.steering_step = 0.1

        self.get_logger().info("Ackermann Teleop Started")
        self.get_logger().info("w/s: speed up/down, a/d: steer left/right, space: stop, q: quit")

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering
        self.pub.publish(msg)

        self.get_logger().info(f"Speed: {self.speed:.2f} m/s, Steering: {self.steering:.2f} rad",
                              throttle_duration_sec=0.5)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    teleop = AckermannTeleop()

    try:
        while rclpy.ok():
            key = get_key()

            if key == 'w':
                teleop.speed = min(teleop.speed + teleop.speed_step, teleop.max_speed)
            elif key == 's':
                teleop.speed = max(teleop.speed - teleop.speed_step, -teleop.max_speed)
            elif key == 'a':
                teleop.steering = min(teleop.steering + teleop.steering_step, teleop.max_steering)
            elif key == 'd':
                teleop.steering = max(teleop.steering - teleop.steering_step, -teleop.max_steering)
            elif key == ' ':
                teleop.speed = 0.0
                teleop.steering = 0.0
            elif key == 'q':
                break
            elif key == '\x03':  # Ctrl+C
                break

            teleop.publish_command()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Stop the car
        teleop.speed = 0.0
        teleop.steering = 0.0
        teleop.publish_command()

        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
