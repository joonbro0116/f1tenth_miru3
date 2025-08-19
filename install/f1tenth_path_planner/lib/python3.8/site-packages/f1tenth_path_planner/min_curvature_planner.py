#!/usr/bin/env python3

"""
F1TENTH 최소 곡률 경로 플래너
Minimum Curvature Path Planner for F1TENTH
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
import math

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header


class MinCurvaturePlanner(Node):
    def __init__(self):
        super().__init__('min_curvature_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.racing_line_pub = self.create_publisher(Path, '/racing_line', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Parameters
        self.declare_parameter('track_width', 2.0)  # 트랙 너비 (m)
        self.declare_parameter('safety_margin', 0.3)  # 안전 마진 (m)
        self.declare_parameter('num_waypoints', 100)  # 웨이포인트 수
        self.declare_parameter('optimization_weight', 1.0)  # 최적화 가중치
        
        self.track_width = self.get_parameter('track_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.opt_weight = self.get_parameter('optimization_weight').value
        
        # 내부 변수
        self.map_data = None
        self.track_centerline = None
        self.track_boundaries = None
        
        self.get_logger().info('최소 곡률 경로 플래너가 시작되었습니다.')

    def map_callback(self, msg):
        """맵 데이터 수신 및 트랙 중심선 추출"""
        self.map_data = msg
        self.extract_track_centerline()
        
    def extract_track_centerline(self):
        """맵에서 트랙 중심선 추출"""
        if self.map_data is None:
            return
            
        # 맵 데이터를 numpy 배열로 변환
        map_array = np.array(self.map_data.data).reshape(
            self.map_data.info.height, self.map_data.info.width)
        
        # 자유 공간 (값이 0인 셀) 찾기
        free_cells = np.where(map_array == 0)
        
        if len(free_cells[0]) == 0:
            self.get_logger().warn('자유 공간을 찾을 수 없습니다.')
            return
        
        # 간단한 중심선 추출 (실제로는 더 복잡한 알고리즘 필요)
        centerline_points = []
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        # 예시: 맵 중앙을 따라 중심선 생성
        height, width = map_array.shape
        for i in range(0, width, 10):  # 10픽셀마다 샘플링
            y_center = height // 2
            if map_array[y_center, i] == 0:  # 자유 공간인 경우
                x = origin_x + i * resolution
                y = origin_y + y_center * resolution
                centerline_points.append([x, y])
        
        if len(centerline_points) > 3:
            self.track_centerline = np.array(centerline_points)
            self.get_logger().info(f'중심선 추출 완료: {len(centerline_points)}개 포인트')
        
    def goal_callback(self, msg):
        """목표 지점 수신 및 경로 계획"""
        if self.track_centerline is None:
            self.get_logger().warn('트랙 중심선이 없습니다. 맵을 먼저 로드하세요.')
            return
            
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.get_logger().info(f'목표 지점 수신: ({goal_x:.2f}, {goal_y:.2f})')
        
        # 최소 곡률 경로 생성
        optimal_path = self.plan_min_curvature_path(goal_x, goal_y)
        
        if optimal_path is not None:
            self.publish_path(optimal_path)
            
    def plan_min_curvature_path(self, goal_x, goal_y):
        """최소 곡률 경로 계획"""
        if len(self.track_centerline) < 3:
            return None
            
        # 시작점: 중심선의 첫 번째 점
        start_point = self.track_centerline[0]
        goal_point = np.array([goal_x, goal_y])
        
        # 중심선을 따라 웨이포인트 생성
        waypoints = self.generate_waypoints(start_point, goal_point)
        
        # 곡률 최소화 최적화
        optimized_path = self.optimize_curvature(waypoints)
        
        return optimized_path
        
    def generate_waypoints(self, start, goal):
        """시작점과 목표점 사이의 웨이포인트 생성"""
        # 중심선에서 가장 가까운 점들 찾기
        start_idx = self.find_closest_point_index(start)
        goal_idx = self.find_closest_point_index(goal)
        
        if start_idx == goal_idx:
            return np.array([start, goal])
            
        # 중심선을 따라 웨이포인트 선택
        if start_idx < goal_idx:
            centerline_segment = self.track_centerline[start_idx:goal_idx+1]
        else:
            centerline_segment = self.track_centerline[goal_idx:start_idx+1]
            centerline_segment = centerline_segment[::-1]  # 순서 뒤집기
            
        # 균등하게 웨이포인트 샘플링
        if len(centerline_segment) > self.num_waypoints:
            indices = np.linspace(0, len(centerline_segment)-1, self.num_waypoints, dtype=int)
            waypoints = centerline_segment[indices]
        else:
            waypoints = centerline_segment
            
        return waypoints
        
    def find_closest_point_index(self, point):
        """주어진 점에 가장 가까운 중심선 점의 인덱스 찾기"""
        distances = np.linalg.norm(self.track_centerline - point, axis=1)
        return np.argmin(distances)
        
    def optimize_curvature(self, waypoints):
        """곡률 최소화 최적화"""
        if len(waypoints) < 3:
            return waypoints
            
        # 초기 추정값: 기존 웨이포인트
        x0 = waypoints.flatten()
        
        # 첫 번째와 마지막 점은 고정
        bounds = []
        for i in range(len(waypoints)):
            if i == 0 or i == len(waypoints) - 1:
                # 시작점과 끝점 고정
                bounds.append((waypoints[i, 0], waypoints[i, 0]))  # x 고정
                bounds.append((waypoints[i, 1], waypoints[i, 1]))  # y 고정
            else:
                # 중간점들은 트랙 경계 내에서 이동 가능
                margin = self.track_width / 2 - self.safety_margin
                bounds.append((waypoints[i, 0] - margin, waypoints[i, 0] + margin))  # x 범위
                bounds.append((waypoints[i, 1] - margin, waypoints[i, 1] + margin))  # y 범위
        
        # 최적화 실행
        try:
            result = minimize(
                self.curvature_objective,
                x0,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 1000}
            )
            
            if result.success:
                optimized_waypoints = result.x.reshape(-1, 2)
                self.get_logger().info('곡률 최적화 성공')
                return optimized_waypoints
            else:
                self.get_logger().warn('곡률 최적화 실패, 원본 경로 사용')
                return waypoints
                
        except Exception as e:
            self.get_logger().error(f'최적화 중 오류: {str(e)}')
            return waypoints
            
    def curvature_objective(self, x):
        """곡률 최소화 목적 함수"""
        waypoints = x.reshape(-1, 2)
        
        if len(waypoints) < 3:
            return 0.0
            
        total_curvature = 0.0
        
        for i in range(1, len(waypoints) - 1):
            # 세 연속 점으로 곡률 계산
            p1 = waypoints[i-1]
            p2 = waypoints[i]
            p3 = waypoints[i+1]
            
            curvature = self.calculate_curvature(p1, p2, p3)
            total_curvature += curvature ** 2
            
        return total_curvature
        
    def calculate_curvature(self, p1, p2, p3):
        """세 점으로 곡률 계산"""
        # 벡터 계산
        v1 = p2 - p1
        v2 = p3 - p2
        
        # 길이 계산
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
            
        # 각도 변화량으로 곡률 근사
        cos_angle = np.dot(v1, v2) / (len1 * len2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_change = np.arccos(cos_angle)
        
        # 곡률 = 각도 변화 / 호장 길이
        arc_length = (len1 + len2) / 2
        curvature = angle_change / arc_length if arc_length > 1e-6 else 0.0
        
        return curvature
        
    def publish_path(self, waypoints):
        """경로 퍼블리시"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            
            # 방향 계산 (다음 점을 향하는 방향)
            if len(path_msg.poses) > 0:
                prev_point = path_msg.poses[-1].pose.position
                dx = point[0] - prev_point.x
                dy = point[1] - prev_point.y
                yaw = math.atan2(dy, dx)
                
                pose_stamped.pose.orientation.z = math.sin(yaw / 2)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose_stamped.pose.orientation.w = 1.0
                
            path_msg.poses.append(pose_stamped)
            
        self.path_pub.publish(path_msg)
        self.racing_line_pub.publish(path_msg)
        self.get_logger().info(f'경로 퍼블리시 완료: {len(waypoints)}개 포인트')


def main(args=None):
    rclpy.init(args=args)
    
    planner = MinCurvaturePlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()