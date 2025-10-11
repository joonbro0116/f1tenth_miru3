#!/usr/bin/env python3
import csv
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

def read_csv_xy(path: str) -> List[Tuple[float, float]]:
    pts = []
    with open(path, "r") as f:
        rd = csv.reader(f)
        for row in rd:
            if len(row) < 2:
                continue
            x = float(row[0]); y = float(row[1])
            pts.append((x, y))
    return pts

def point_in_polygon(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    x0, y0 = poly[-1]
    for i in range(n):
        x1, y1 = poly[i]
        # 경계 특이점 줄이기: x < xin
        if ((y1 > y) != (y0 > y)):
            xin = (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0
            if x < xin:
                inside = not inside
        x0, y0 = x1, y1
    return inside

def point_to_polyline_distance(px: float, py: float, poly: List[Tuple[float, float]]) -> float:
    if len(poly) < 2:
        return float('inf')
    min_d2 = float('inf')
    x0, y0 = poly[-1]
    for (x1, y1) in poly:
        vx, vy = x1 - x0, y1 - y0
        wx, wy = px - x0, py - y0
        vv = vx*vx + vy*vy
        t = 0.0 if vv < 1e-12 else max(0.0, min(1.0, (wx*vx + wy*vy) / vv))
        projx, projy = x0 + t*vx, y0 + t*vy
        dx, dy = px - projx, py - projy
        d2 = dx*dx + dy*dy
        if d2 < min_d2:
            min_d2 = d2
        x0, y0 = x1, y1
    return math.sqrt(min_d2)

class CsvObstacleNode(Node):
    def __init__(self):
        super().__init__('csv_obstacle')

        # params
        self.declare_parameter('outer_csv', 'bounds_out/outer_bound_world.csv')
        self.declare_parameter('inner_csv', 'bounds_out/inner_bound_world.csv')
        self.declare_parameter('bubble_topic', '/bubble_grid')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('marker_stride', 2)
        self.declare_parameter('lane_half_width', 0.6)
        self.declare_parameter('block_dist', 8.0)
        self.declare_parameter('publish_blocked', True)
        self.declare_parameter('publish_boundaries', True)
        self.declare_parameter('boundary_clearance_m', 0.25)

        # 카드(요약 HUD) 파라미터
        self.declare_parameter('enable_card', True)
        self.declare_parameter('card_offset_x', 0.8)   # 차량 기준 앞쪽(m)
        self.declare_parameter('card_offset_y', -0.6)  # 차량 기준 좌/우(m): 음수면 오른쪽
        self.declare_parameter('card_scale', 0.35)     # 텍스트 크기
        self.declare_parameter('card_bg_alpha', 0.55)  # 배경 투명도

        self.outer_csv = self.get_parameter('outer_csv').value
        self.inner_csv = self.get_parameter('inner_csv').value
        self.bubble_topic = self.get_parameter('bubble_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.occ_th = int(self.get_parameter('occ_threshold').value)
        self.stride = max(1, int(self.get_parameter('marker_stride').value))
        self.lane_half_width = float(self.get_parameter('lane_half_width').value)
        self.block_dist = float(self.get_parameter('block_dist').value)
        self.pub_blocked_flag = bool(self.get_parameter('publish_blocked').value)
        self.pub_bounds_flag = bool(self.get_parameter('publish_boundaries').value)
        self.boundary_clearance_m = float(self.get_parameter('boundary_clearance_m').value)

        self.enable_card = bool(self.get_parameter('enable_card').value)
        self.card_offset_x = float(self.get_parameter('card_offset_x').value)
        self.card_offset_y = float(self.get_parameter('card_offset_y').value)
        self.card_scale = float(self.get_parameter('card_scale').value)
        self.card_bg_alpha = float(self.get_parameter('card_bg_alpha').value)

        # load polygons
        self.outer_poly = read_csv_xy(self.outer_csv)
        self.inner_poly = read_csv_xy(self.inner_csv)

        if len(self.outer_poly) < 3:
            self.get_logger().error(f'outer csv invalid: {self.outer_csv}')
        if len(self.inner_poly) < 3:
            self.get_logger().warn(f'inner csv invalid or empty: {self.inner_csv} (no inner hole)')

        # state
        self.pose = (0.0, 0.0, 0.0)
        self.latest_obstacles: List[Tuple[float,float]] = []
        self.last_blocked = False

        # subs
        self.sub_bubble = self.create_subscription(OccupancyGrid, self.bubble_topic, self.on_bubble, 1)
        self.sub_odom   = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

        # pubs
        self.pub_markers = self.create_publisher(MarkerArray, '/track_obstacles', 1)
        self.pub_blocked = self.create_publisher(Bool, '/lane_blocked', 1) if self.pub_blocked_flag else None
        self.pub_bounds  = self.create_publisher(MarkerArray, '/track_bounds', 1) if self.pub_bounds_flag else None
        self.pub_card    = self.create_publisher(MarkerArray, '/csv_viz_card', 1) if self.enable_card else None

        if self.pub_bounds_flag:
            self.publish_boundaries()

        self.get_logger().info('csv_obstacle up (bubble_grid ∩ track ROI, with boundary clearance & card).')

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x, y, yaw)

    def on_bubble(self, grid: OccupancyGrid):
        info = grid.info
        W, H = info.width, info.height
        res  = info.resolution
        orgx = info.origin.position.x
        orgy = info.origin.position.y

        def cell_center_world(ix, iy):
            return (orgx + (ix + 0.5) * res,
                    orgy + (iy + 0.5) * res)

        in_track_obstacles = []
        has_inner = len(self.inner_poly) >= 3
        bc = max(0.0, self.boundary_clearance_m)

        for iy in range(0, H, self.stride):
            base = iy * W
            for ix in range(0, W, self.stride):
                occ = grid.data[base + ix]
                if occ < self.occ_th:
                    continue
                wx, wy = cell_center_world(ix, iy)

                # 트랙 ROI
                if not point_in_polygon(wx, wy, self.outer_poly):
                    continue
                if has_inner and point_in_polygon(wx, wy, self.inner_poly):
                    continue

                # 경계 여유띠
                d_outer = point_to_polyline_distance(wx, wy, self.outer_poly)
                d_inner = point_to_polyline_distance(wx, wy, self.inner_poly) if has_inner else float('inf')
                if min(d_outer, d_inner) < bc:
                    continue

                in_track_obstacles.append((wx, wy))

        self.latest_obstacles = in_track_obstacles
        self.publish_obstacle_markers(in_track_obstacles)
        self.maybe_publish_blocked(in_track_obstacles)
        if self.pub_bounds_flag:
            self.publish_boundaries()
        if self.enable_card:
            self.publish_card(in_track_obstacles)

    # ------------ publishers ------------
    def publish_obstacle_markers(self, pts: List[Tuple[float,float]]):
        arr = MarkerArray()
        m0 = Marker(); m0.action = Marker.DELETEALL
        arr.markers.append(m0)

        now = self.get_clock().now().to_msg()
        ns = 'csv_track_obs'
        for k, (x, y) in enumerate(pts):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = ns
            m.id = k
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.9
            arr.markers.append(m)

        self.pub_markers.publish(arr)

    def maybe_publish_blocked(self, pts: List[Tuple[float,float]]):
        if self.pub_blocked is None:
            return
        px, py, psi = self.pose
        blocked = False
        for (ox, oy) in pts:
            dx = ox - px; dy = oy - py
            lx =  math.cos(psi)*dx + math.sin(psi)*dy
            ly = -math.sin(psi)*dx + math.cos(psi)*dy
            if lx > 0.0 and abs(ly) < self.lane_half_width and math.hypot(dx, dy) < self.block_dist:
                blocked = True
                break
        self.last_blocked = blocked
        msg = Bool(); msg.data = blocked
        self.pub_blocked.publish(msg)

    def publish_boundaries(self):
        if self.pub_bounds is None:
            return
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        m0 = Marker(); m0.action = Marker.DELETEALL
        arr.markers.append(m0)

        def make_line_strip(points, mid, rgba):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'track_bounds'
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            for (x, y) in points:
                p = Point(x=x, y=y, z=0.0)
                m.points.append(p)
            if len(points) > 2:
                m.points.append(Point(x=points[0][0], y=points[0][1], z=0.0))
            return m

        if len(self.outer_poly) >= 2:
            arr.markers.append(make_line_strip(self.outer_poly, 1, (0.1, 0.6, 1.0, 0.9)))
        if len(self.inner_poly) >= 2:
            arr.markers.append(make_line_strip(self.inner_poly, 2, (0.1, 1.0, 0.4, 0.9)))

        self.pub_bounds.publish(arr)

    def publish_card(self, pts: List[Tuple[float,float]]):
        if self.pub_card is None:
            return
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # DELETEALL
        m0 = Marker(); m0.action = Marker.DELETEALL
        arr.markers.append(m0)

        # 카드 위치: 차량 포즈 기준 오프셋 적용
        px, py, psi = self.pose
        ox = px + math.cos(psi)*self.card_offset_x - math.sin(psi)*self.card_offset_y
        oy = py + math.sin(psi)*self.card_offset_x + math.cos(psi)*self.card_offset_y

        # 배경 카드 (반투명 박스)
        bg = Marker()
        bg.header.frame_id = 'map'
        bg.header.stamp = now
        bg.ns = 'csv_card'
        bg.id = 1000
        bg.type = Marker.CUBE
        bg.action = Marker.ADD
        bg.pose.position.x = ox
        bg.pose.position.y = oy
        bg.pose.position.z = 0.05
        bg.scale.x = 0.9   # 가로
        bg.scale.y = 0.01  # 두께(얇게)
        bg.scale.z = 0.6   # 세로
        bg.color.r = 0.05; bg.color.g = 0.05; bg.color.b = 0.05; bg.color.a = max(0.1, min(1.0, self.card_bg_alpha))
        arr.markers.append(bg)

        # 텍스트
        txt = Marker()
        txt.header.frame_id = 'map'
        txt.header.stamp = now
        txt.ns = 'csv_card'
        txt.id = 1001
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = ox
        txt.pose.position.y = oy
        txt.pose.position.z = 0.30
        txt.scale.z = max(0.10, self.card_scale)  # 폰트 크기
        txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.95

        lines = []
        lines.append("CSV Track Obstacles")
        lines.append(f"obs: {len(pts)}   blocked: {self.last_blocked}")
        lines.append(f"occ_th: {self.occ_th}  stride: {self.stride}")
        lines.append(f"clearance: {self.boundary_clearance_m:.2f} m")
        lines.append(f"lane_hw: {self.lane_half_width:.2f}  block_dist: {self.block_dist:.1f} m")
        txt.text = "\n".join(lines)

        arr.markers.append(txt)
        self.pub_card.publish(arr)

def main():
    rclpy.init()
    node = CsvObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
