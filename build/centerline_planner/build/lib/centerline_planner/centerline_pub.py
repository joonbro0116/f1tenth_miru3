#!/usr/bin/env python3
import csv
from math import atan2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat

class CenterlinePub(Node):
    def __init__(self):
        super().__init__('centerline_pub')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').value

        if not csv_path:
            raise RuntimeError('param "csv_path" 가 비었습니다. maps/*.csv 경로를 주십시오.')

        # 읽어서 Path 구성
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        pts = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)

        # 헤더 있는/없는 파일 모두 처리
        def try_float(s):
            try:
                return float(s)
            except:
                return None

        for row in rows:
            if len(row) < 2: 
                continue
            x = try_float(row[0]); y = try_float(row[1])
            if x is None or y is None:
                continue
            pts.append((x, y))

        # yaw(진행방향)까지 넣어서 포즈 생성
        n = len(pts)
        for i, (x, y) in enumerate(pts):
            # 다음 점과의 접선으로 yaw 추정
            j = (i + 1) % n if n > 1 else i
            dx = pts[j][0] - x
            dy = pts[j][1] - y
            yaw = atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0
            qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw, axes='sxyz')

            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            self.path_msg.poses.append(ps)

        # late-joiner도 받도록 latch(TRANSIENT_LOCAL)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, '/global_path', qos)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'Centerline loaded: {len(self.path_msg.poses)} points from {csv_path}')

    def on_timer(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = CenterlinePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
