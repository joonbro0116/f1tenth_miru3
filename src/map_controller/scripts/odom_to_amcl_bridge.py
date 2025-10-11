#!/usr/bin/env python3
"""
Simple bridge to convert nav_msgs/Odometry to geometry_msgs/PoseWithCovarianceStamped
For use in gym-ros simulator where odom is already in map frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class OdomToAmclBridge(Node):
    def __init__(self):
        super().__init__('odom_to_amcl_bridge')

        qos_sensor = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            Odometry,
            'odom_in',
            self.odom_callback,
            qos_profile=qos_sensor
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'amcl_pose_out',
            10
        )

        self.get_logger().info('Odom to AMCL bridge started')

    def odom_callback(self, msg):
        """Convert Odometry to PoseWithCovarianceStamped"""
        amcl_msg = PoseWithCovarianceStamped()

        # Copy header
        amcl_msg.header = msg.header
        amcl_msg.header.frame_id = 'map'  # Ensure map frame

        # Copy pose
        amcl_msg.pose.pose = msg.pose.pose

        # Copy covariance (or use odometry covariance)
        amcl_msg.pose.covariance = msg.pose.covariance

        self.pub.publish(amcl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToAmclBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
