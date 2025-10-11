#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BoundObstacleDetector (CORE/RING 파이프라인 완성본)

핵심 아이디어
- outer/inner 폴리곤을 맵 해상도에 맞춰 '내부마스크'로 래스터화
- 내부마스크를 침식(erode)해 CORE(핵심 내부) 생성, RING=내부-CORE
- 스캔 포인트가 CORE면 마진 없이 채택, RING이면 소(小)마진/보수 클러스터 기준으로 채택
- 최종 군집 중심만 차로 차단 여부에 사용, RViz 마커로 디버깅

필수 파라미터(런치에서 지정 권장)
- outer_csv, inner_csvs(여러 파일/글롭/콤마 세퍼레이터 지원)
- mask_width/height, mask_resolution, mask_origin([x,y,yaw])  ← 맵 제작 시 값과 동일!
"""

import math, csv, os, re, glob
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import cv2
import numpy as np

# -------------------- utils: csv I/O --------------------
# csv 로드하여 월드 좌표계에서 다각형 꼭짓점을 읽어 폐곡선화(첫점=끝점)
def load_world_csv(path: str) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    with open(path, 'r') as f:
        rd = csv.reader(f)
        for row in rd:
            if not row:
                continue
            pts.append((float(row[0]), float(row[1])))
    if len(pts) >= 1 and pts[0] != pts[-1]:
        pts.append(pts[0])  # 닫기
    return pts

# inner 파일은 한 개라 아래 함수는 사용안할 듯
def load_multi_inner(spec: str) -> List[List[Tuple[float,float]]]:
    """
    spec 예:
      - "inner_bound_world.csv"
      - "bounds_out/inner_*.csv"
      - "a.csv,b.csv; c.csv"
    """
    paths: List[str] = []
    if any(ch in spec for ch in [',',';',' ']):
        parts = re.split(r'[,\s;]+', spec)
        for p in parts:
            if not p: continue
            paths.extend(glob.glob(p))
    else:
        paths = glob.glob(spec) if any(ch in spec for ch in ['*','?','[']) else [spec]

    inners: List[List[Tuple[float,float]]] = []
    for p in paths:
        if not os.path.exists(p):
            continue
        poly = load_world_csv(p)
        if len(poly) >= 3:
            inners.append(poly)
    return inners

# -------------------- utils: geometry --------------------
# 포인트를 다각형 내부에 포함하는지 판정 (Ray casting)
def point_in_polygon(x: float, y: float, poly: List[Tuple[float,float]], include_boundary: bool=True) -> bool:
    """Ray casting. 경계 포함 여부 토글 가능."""
    inside = False
    n = len(poly)
    if n < 3: return False
    x0, y0 = poly[-1]
    for i in range(n):
        x1, y1 = poly[i]
        if (y1 > y) != (y0 > y):
            xin = (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0
            if (x <= xin) if include_boundary else (x < xin):
                inside = not inside
        x0, y0 = x1, y1
    return inside

def dist_point_to_segment(px, py, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx*vx + vy*vy
    t = 0.0 if vv == 0.0 else max(0.0, min(1.0, (wx*vx + wy*vy)/vv))
    cx, cy = ax + t*vx, ay + t*vy
    return math.hypot(px - cx, py - cy)

def min_dist_to_poly(px, py, poly: List[Tuple[float,float]]) -> float:
    if len(poly) < 2:
        return float('inf')
    dmin = float('inf')
    x0, y0 = poly[-1]
    for (x1, y1) in poly:
        d = dist_point_to_segment(px, py, x0, y0, x1, y1)
        if d < dmin: dmin = d
        x0, y0 = x1, y1
    return dmin

# -------------------- utils: coords --------------------
def world_to_pixel_int(wx, wy, img_h, res, origin):
    ox, oy, oyaw = origin
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    x = wx - ox;  y = wy - oy
    xr = c*x - s*y;  yr = s*x + c*y
    px = int(round(xr / res))
    py = int(round(img_h - (yr / res)))
    return px, py

# -------------------- utils: polygon -> CORE/RING masks --------------------
def polys_to_masks(outer_poly, inner_polys, res, origin, img_w, img_h, core_shrink_m):
    """
    반환: inner_full(255=내부 전체), core(255=핵심 내부), ring(255=경계 주변 띠)
    """
    m = np.zeros((img_h, img_w), np.uint8)

    # outer 채우기
    outer_pts = [world_to_pixel_int(x, y, img_h, res, origin) for (x,y) in outer_poly]
    if len(outer_pts) >= 3:
        cv2.fillPoly(m, [np.array(outer_pts, np.int32)], 255)

    # inner 구멍
    for ip in inner_polys:
        if len(ip) < 3: continue
        pts = [world_to_pixel_int(x, y, img_h, res, origin) for (x,y) in ip]
        cv2.fillPoly(m, [np.array(pts, np.int32)], 0)

    inner_full = m.copy()

    # CORE = 내부를 core_shrink_m 만큼 침식
    px = max(1, int(round(core_shrink_m / max(1e-9, res))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*px+1, 2*px+1))
    core = cv2.erode(inner_full, kernel, iterations=1)

    # RING = 내부 전체 - CORE
    ring = cv2.subtract(inner_full, core)
    return inner_full, core, ring

# -------------------- node --------------------
class BoundObstacleDetector(Node):
    def __init__(self):
        super().__init__('bound_obstacle_detector')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')

        # Bounds (CSV in world coords)
        self.declare_parameter('outer_csv',  'outer_bound_world.csv')
        self.declare_parameter('inner_csvs', 'inner_bound_world.csv')  # 여러 개/글롭/콤마 OK

        # Include boundary?
        self.declare_parameter('include_boundary', True)

        # 기본 필터
        self.declare_parameter('roi_deg', 170.0)
        self.declare_parameter('range_max', 12.0)

        # 라이다 장착 각도 보정(deg)
        self.declare_parameter('laser_yaw_offset_deg', 0.0)

        # 클러스터 파라미터
        self.declare_parameter('euclid_eps', 0.28)     # 코너에서 잘 붙게
        self.declare_parameter('euclid_min_pts', 1)    # CORE는 1~2, RING은 2~3로 따로 적용할 것

        # 차로 차단 판정
        self.declare_parameter('lane_block_half_width', 0.5)
        self.declare_parameter('lane_block_dist', 8.0)

        # 마스크 생성 파라미터(맵과 일치 필수!)
        self.declare_parameter('mask_width',  2048)
        self.declare_parameter('mask_height', 2048)
        self.declare_parameter('mask_resolution', 0.05)        # m/px
        self.declare_parameter('mask_origin', [0.0, 0.0, 0.0]) # [x,y,yaw]

        # CORE/RING 로직 파라미터
        self.declare_parameter('core_shrink_m', 0.20)      # CORE 만들기 위한 침식 량
        self.declare_parameter('ring_small_margin', 0.03)  # RING 전용 소(小)마진

        # 파라미터 fetch
        self.scan_topic  = self.get_parameter('scan_topic').value
        self.odom_topic  = self.get_parameter('odom_topic').value

        self.outer_csv   = self.get_parameter('outer_csv').value
        self.inner_spec  = self.get_parameter('inner_csvs').value

        self.include_boundary = bool(self.get_parameter('include_boundary').value)

        self.roi_rad     = math.radians(float(self.get_parameter('roi_deg').value))
        self.range_max   = float(self.get_parameter('range_max').value)

        self.laser_yaw   = math.radians(float(self.get_parameter('laser_yaw_offset_deg').value))

        self.eps         = float(self.get_parameter('euclid_eps').value)
        self.min_pts     = int(self.get_parameter('euclid_min_pts').value)

        self.block_hw    = float(self.get_parameter('lane_block_half_width').value)
        self.block_dist  = float(self.get_parameter('lane_block_dist').value)

        self.mask_W      = int(self.get_parameter('mask_width').value)
        self.mask_H      = int(self.get_parameter('mask_height').value)
        self.mask_res    = float(self.get_parameter('mask_resolution').value)
        self.mask_origin = list(self.get_parameter('mask_origin').value)

        self.core_shrink_m     = float(self.get_parameter('core_shrink_m').value)
        self.ring_small_margin = float(self.get_parameter('ring_small_margin').value)

        # 경계 로드
        self.outer  = load_world_csv(self.outer_csv)
        if len(self.outer) < 3:
            raise RuntimeError(f"outer CSV 비정상: {self.outer_csv}")
        self.inners = load_multi_inner(self.inner_spec)

        self.get_logger().info(
            f"Bounds loaded: outer={len(self.outer)} pts, inners={len(self.inners)} polygon(s)"
        )

        # CORE/RING 마스크 생성
        self.mask_inner_full, self.mask_core, self.mask_ring = polys_to_masks(
            self.outer, self.inners, self.mask_res, self.mask_origin,
            self.mask_W, self.mask_H, self.core_shrink_m
        )
        self.get_logger().info(
            f"Mask built: size={self.mask_W}x{self.mask_H}, res={self.mask_res}, "
            f"core_shrink={self.core_shrink_m}m"
        )

        # 상태
        self.pose = (0.0, 0.0, 0.0)  # map 기준이라고 가정(TF 필요시 확장)

        # pubs/subs
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 30)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/bound_obstacles', 1)
        self.pub_blocked = self.create_publisher(Bool, '/lane_blocked', 1)
        self.pub_dbg     = self.create_publisher(MarkerArray, '/bound_obstacles_debug', 1)

        self.get_logger().info("BoundObstacleDetector ready (CORE/RING).")

    # -------------------- callbacks --------------------
    def on_odom(self, msg: Odometry):
        # NOTE: 이 Odometry가 'map' 프레임 기준이라고 가정
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x, y, yaw)

    # -------------------- mask helper --------------------
    def _mask_kind(self, wx, wy):
        """0=outside, 1=RING, 2=CORE"""
        px, py = world_to_pixel_int(wx, wy, self.mask_H, self.mask_res, self.mask_origin)
        if px < 0 or py < 0 or px >= self.mask_W or py >= self.mask_H:
            return 0
        if self.mask_inner_full[py, px] == 0:
            return 0
        if self.mask_core[py, px] != 0:
            return 2
        if self.mask_ring[py, px] != 0:
            return 1
        return 0

    # -------------------- scan processing --------------------
    def on_scan(self, scan: LaserScan):
        px, py, psi = self.pose

        ang0 = scan.angle_min
        inc  = scan.angle_increment
        n    = len(scan.ranges)
        if n == 0 or inc == 0.0:
            return

        # ROI
        roi_min = -self.roi_rad
        roi_max = +self.roi_rad
        i0 = max(0, int(math.ceil((roi_min - ang0)/inc)))
        i1 = min(n-1, int(math.floor((roi_max - ang0)/inc)))
        if i0 > i1:
            return

        angles = [ang0 + i*inc for i in range(n)]
        rng_max = min(self.range_max, scan.range_max)

        core_pts: List[Tuple[float,float]] = []
        ring_pts: List[Tuple[float,float]] = []

        # 1) 내부 포인트 수집: CORE / RING 분리
        for i in range(i0, i1+1):
            r = scan.ranges[i]
            if r is None or math.isnan(r) or math.isinf(r) or r < scan.range_min or r > rng_max:
                continue
            th = angles[i]
            wx = px + r * math.cos(psi + self.laser_yaw + th)
            wy = py + r * math.sin(psi + self.laser_yaw + th)

            mk = self._mask_kind(wx, wy)
            if mk == 2:      # CORE
                core_pts.append((wx, wy))
            elif mk == 1:    # RING
                ring_pts.append((wx, wy))
            else:
                continue

        # 2) 군집
        centers_core  = self._euclid_cluster_centers(core_pts, eps=self.eps, min_pts=max(1, self.min_pts))          # CORE: 느슨
        centers_ring0 = self._euclid_cluster_centers(ring_pts, eps=max(self.eps, 0.25), min_pts=max(2, self.min_pts)) # RING: 보수

        # 3) RING 전용 소마진(outer/inners와의 거리) 필터
        centers_ring: List[Tuple[float,float]] = []
        for (cx, cy) in centers_ring0:
            near_outer = (min_dist_to_poly(cx, cy, self.outer) < self.ring_small_margin)
            near_inner = any(len(ip)>=3 and (min_dist_to_poly(cx, cy, ip) < self.ring_small_margin) for ip in self.inners)
            if not (near_outer or near_inner):
                centers_ring.append((cx, cy))

        # 4) 최종 합치기 (CORE는 마진 없이 통과)
        final_centers = centers_core + centers_ring

        # 5) 차로 차단 여부
        blocked = False
        for (ox, oy) in final_centers:
            dx, dy = ox - px, oy - py
            lx =  math.cos(psi)*dx + math.sin(psi)*dy
            ly = -math.sin(psi)*dx + math.cos(psi)*dy
            if lx > 0.0 and abs(ly) < self.block_hw and math.hypot(dx, dy) < self.block_dist:
                blocked = True
                break

        # 6) publish
        self._pub_obstacle_markers(final_centers)
        self._pub_debug_masks(core_pts, ring_pts, final_centers)

        msg = Bool(); msg.data = blocked
        self.pub_blocked.publish(msg)

    # -------------------- clustering --------------------
    def _euclid_cluster_centers(self, pts: List[Tuple[float,float]], eps: float, min_pts: int):
        n = len(pts)
        if n == 0: return []
        parent = list(range(n))
        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        def union(a,b):
            ra, rb = find(a), find(b)
            if ra != rb: parent[ra] = rb

        eps2 = eps*eps
        for i in range(n):
            xi, yi = pts[i]
            for j in range(i+1, n):
                xj, yj = pts[j]
                dx, dy = xi-xj, yi-yj
                if dx*dx + dy*dy <= eps2:
                    union(i, j)

        buckets = {}
        for i in range(n):
            r = find(i)
            buckets.setdefault(r, []).append(i)

        centers = []
        for idxs in buckets.values():
            if len(idxs) < min_pts:
                continue
            sx = sy = 0.0
            for k in idxs:
                x,y = pts[k]
                sx += x; sy += y
            centers.append((sx/len(idxs), sy/len(idxs)))
        return centers

    # -------------------- markers --------------------
    def _pub_obstacle_markers(self, centers: List[Tuple[float, float]]):
        arr = MarkerArray()
        # 전체 삭제
        m0 = Marker(); m0.action = Marker.DELETEALL
        arr.markers.append(m0)

        now = self.get_clock().now().to_msg()

        # centers as SPHERE_LIST
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'bo_centers'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.32
        m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.2; m.color.a = 0.95
        for (x,y) in centers:
            p = Point(); p.x = x; p.y = y; p.z = 0.0
            m.points.append(p)
        arr.markers.append(m)

        self.pub_markers.publish(arr)

    def _pub_debug_masks(self, core_pts, ring_pts, final_centers):
        arr = MarkerArray()
        m0 = Marker(); m0.action = Marker.DELETEALL
        arr.markers.append(m0)
        now = self.get_clock().now().to_msg()

        # CORE 포인트(파랑)
        m_core = Marker()
        m_core.header.frame_id = 'map'
        m_core.header.stamp = now
        m_core.ns = 'bo_dbg'
        m_core.id = 0
        m_core.type = Marker.POINTS
        m_core.action = Marker.ADD
        m_core.scale.x = m_core.scale.y = 0.05
        m_core.color.r = 0.2; m_core.color.g = 0.45; m_core.color.b = 1.0; m_core.color.a = 0.9
        for (x,y) in core_pts:
            m_core.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(m_core)

        # RING 포인트(초록)
        m_ring = Marker()
        m_ring.header.frame_id = 'map'
        m_ring.header.stamp = now
        m_ring.ns = 'bo_dbg'
        m_ring.id = 1
        m_ring.type = Marker.POINTS
        m_ring.action = Marker.ADD
        m_ring.scale.x = m_ring.scale.y = 0.05
        m_ring.color.r = 0.2; m_ring.color.g = 1.0; m_ring.color.b = 0.2; m_ring.color.a = 0.9
        for (x,y) in ring_pts:
            m_ring.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(m_ring)

        # 최종 센터(빨강)
        m_fin = Marker()
        m_fin.header.frame_id = 'map'
        m_fin.header.stamp = now
        m_fin.ns = 'bo_dbg'
        m_fin.id = 2
        m_fin.type = Marker.SPHERE_LIST
        m_fin.action = Marker.ADD
        m_fin.scale.x = m_fin.scale.y = m_fin.scale.z = 0.32
        m_fin.color.r = 1.0; m_fin.color.g = 0.2; m_fin.color.b = 0.2; m_fin.color.a = 0.95
        for (x,y) in final_centers:
            m_fin.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(m_fin)

        self.pub_dbg.publish(arr)

# -------------------- main --------------------
def main(args=None):
    rclpy.init(args=args)
    node = BoundObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
