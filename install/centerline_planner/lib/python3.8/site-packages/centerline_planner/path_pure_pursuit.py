#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class PathPurePursuit(Node):
    def __init__(self):
        super().__init__('path_pure_pursuit')

        # 파라미터
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead_base', 1.0)      # m, 기본
        self.declare_parameter('lookahead_k', 0.3)         # v 연동
        self.declare_parameter('lookahead_min', 0.7)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('max_steer', 0.4)           # rad
        self.declare_parameter('speed_min', 1.5)
        self.declare_parameter('speed_max', 3.0)
        self.declare_parameter('frame_id', 'base_link')    # drive msg frame

        self.wb = float(self.get_parameter('wheelbase').value)
        self.Ld0 = float(self.get_parameter('lookahead_base').value)
        self.k    = float(self.get_parameter('lookahead_k').value)
        self.Ldmin = float(self.get_parameter('lookahead_min').value)
        self.Ldmax = float(self.get_parameter('lookahead_max').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.vmin = float(self.get_parameter('speed_min').value)
        self.vmax = float(self.get_parameter('speed_max').value)
        self.drive_frame = self.get_parameter('frame_id').get_parameter_value().string_value

        self.path = None
        self.odom = None
        self.prev_idx = 0

        self.sub_path = self.create_subscription(Path, '/global_path', self.on_path, 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.on_odom, 20)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('PathPurePursuit started.')

    def on_path(self, msg: Path):
        self.path = msg

    def on_odom(self, msg: Odometry):
        self.odom = msg
        self.step()

    def step(self):
        if self.path is None or len(self.path.poses) == 0 or self.odom is None:
            return

        # 차량 상태 (map frame)
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        qx = self.odom.pose.pose.orientation.x
        qy = self.odom.pose.pose.orientation.y
        qz = self.odom.pose.pose.orientation.z
        qw = self.odom.pose.pose.orientation.w

        # yaw 추출
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)) 공식
        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        vx = self.odom.twist.twist.linear.x
        # lookahead 조절
        Ld = self.Ld0 + self.k * max(0.0, vx)
        Ld = max(self.Ldmin, min(self.Ldmax, Ld))

        # 가장 가까운 점 찾고, 그로부터 Ld 앞 목표점 선택
        idx_closest = self._nearest_index(px, py, start_idx=self.prev_idx)
        goal = self._find_goal_point(px, py, start_idx=idx_closest, Ld=Ld)
        if goal is None:
            # 끝에 가깝다면 마지막 점을 목표로
            goal = self.path.poses[-1].pose.position

        # 차량 좌표계로 변환 (map->base_link)
        dx = goal.x - px
        dy = goal.y - py
        # 회전: 차량 yaw 기준
        x_car =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y_car =  math.sin(-yaw)*dx + math.cos(-yaw)*dy
        Ld_eff = math.hypot(x_car, y_car) + 1e-6

        # Pure Pursuit 조향각
        steer = math.atan2(2.0 * self.wb * y_car, Ld_eff**2)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 속도: 조향각 커질수록 감속
        ang = abs(steer)
        v = self.vmax - (ang / self.max_steer) * (self.vmax - self.vmin)
        v = max(self.vmin, min(self.vmax, v))

        # 메시지 퍼블리시
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.drive_frame
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(v)
        self.pub_drive.publish(msg)

    def _nearest_index(self, px, py, start_idx=0):
        """가장 가까운 path index (선형 탐색, 충분히 빠름)"""
        poses = self.path.poses
        n = len(poses)
        best_i = start_idx % n
        best_d = 1e18
        search_window = min(200, n)  # 너무 멀리까지 볼 필요 없음
        for k in range(search_window):
            i = (start_idx + k) % n
            dx = poses[i].pose.position.x - px
            dy = poses[i].pose.position.y - py
            d2 = dx*dx + dy*dy
            if d2 < best_d:
                best_d = d2
                best_i = i
        self.prev_idx = best_i
        return best_i

    def _find_goal_point(self, px, py, start_idx, Ld):
        """start_idx부터 누적거리 Ld 이상 떨어진 첫 점"""
        poses = self.path.poses
        n = len(poses)
        if n == 0:
            return None

        last = poses[start_idx].pose.position
        accum = 0.0
        for k in range(1, n):
            i = (start_idx + k) % n
            cur = poses[i].pose.position
            accum += math.hypot(cur.x - last.x, cur.y - last.y)
            if accum >= Ld:
                return cur
            last = cur
        return None

def main():
    rclpy.init()
    node = PathPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
