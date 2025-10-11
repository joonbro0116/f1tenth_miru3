#!/usr/bin/env python3
"""
Record waypoints from odometry while manually driving the car.
Press Ctrl+C to save waypoints to CSV.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import sys


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.waypoints = []
        self.last_pos = None
        self.min_distance = 0.3  # Record waypoint every 0.3m

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("Waypoint recorder started. Drive the car manually.")
        self.get_logger().info("Press Ctrl+C to save waypoints.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Only record if we've moved enough distance from last waypoint
        if self.last_pos is None:
            self.waypoints.append([x, y, 2.0])  # Default speed 2.0 m/s
            self.last_pos = (x, y)
            self.get_logger().info(f"Recorded waypoint 1: ({x:.2f}, {y:.2f})")
        else:
            dist = ((x - self.last_pos[0])**2 + (y - self.last_pos[1])**2)**0.5
            if dist >= self.min_distance:
                self.waypoints.append([x, y, 2.0])
                self.last_pos = (x, y)
                self.get_logger().info(f"Recorded waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})")

    def save_waypoints(self, filename):
        if len(self.waypoints) < 2:
            self.get_logger().error("Not enough waypoints to save!")
            return

        with open(filename, 'w') as f:
            writer = csv.writer(f)
            for wp in self.waypoints:
                writer.writerow(wp)

        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {filename}")


def main(args=None):
    rclpy.init(args=args)
    recorder = Node('waypoint_recorder')

    # Get output filename from command line or use default
    if len(sys.argv) > 1:
        output_file = sys.argv[1]
    else:
        output_file = '/home/sh/projects/f1tenth_miru3/maps_racelines/raceline/levine_recorded.csv'

    recorder = WaypointRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_waypoints(output_file)
        print(f"\nWaypoints saved to {output_file}")

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
