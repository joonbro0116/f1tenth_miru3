#!/usr/bin/env python3

"""
로컬라이제이션 성능 평가 테스트 코드

업데이트 속도, 위치 분산, 추정 정확도 등을 실시간으로 모니터링하고 분석

사용법:
ros2 run f1tenth_slam_nav localization_performance_test.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
import time
import math
from collections import deque

class LocalizationPerformanceTest(Node):
    def __init__(self):
        super().__init__('localization_performance_test')

        # 구독자 설정
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # TF 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 성능 데이터 저장
        self.pose_timestamps = deque(maxlen=1000)
        self.poses = deque(maxlen=1000)
        self.covariances = deque(maxlen=1000)

        # 통계 변수
        self.last_pose_time = None
        self.update_intervals = deque(maxlen=100)

        # 이동 거리 추적
        self.last_position = None
        self.total_distance = 0.0
        self.position_history = deque(maxlen=500)

        # 타이머 설정 (매 5초마다 통계 출력)
        self.create_timer(5.0, self.print_statistics)

        self.get_logger().info("로컬라이제이션 성능 테스트 시작...")
        self.get_logger().info("AMCL pose 데이터를 수집 중...")

    def amcl_pose_callback(self, msg):
        current_time = time.time()

        # 업데이트 주기 계산
        if self.last_pose_time is not None:
            interval = current_time - self.last_pose_time
            self.update_intervals.append(interval)

        self.last_pose_time = current_time

        # 포즈 데이터 저장
        pose = msg.pose.pose
        self.pose_timestamps.append(current_time)
        self.poses.append(pose)

        # 공분산 데이터 저장
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        self.covariances.append(cov)

        # 위치 변화 추적
        current_pos = [pose.position.x, pose.position.y]
        self.position_history.append(current_pos)

        if self.last_position is not None:
            distance = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 +
                (current_pos[1] - self.last_position[1])**2
            )
            self.total_distance += distance

        self.last_position = current_pos

    def calculate_position_variance(self, window_size=50):
        """최근 N개 위치의 분산 계산"""
        if len(self.position_history) < window_size:
            return None, None

        recent_positions = list(self.position_history)[-window_size:]
        positions = np.array(recent_positions)

        var_x = np.var(positions[:, 0])
        var_y = np.var(positions[:, 1])

        return var_x, var_y

    def calculate_update_rate(self):
        """업데이트 주기 통계 계산"""
        if len(self.update_intervals) == 0:
            return None, None, None

        intervals = np.array(self.update_intervals)
        mean_interval = np.mean(intervals)
        std_interval = np.std(intervals)
        frequency = 1.0 / mean_interval if mean_interval > 0 else 0

        return frequency, mean_interval, std_interval

    def calculate_covariance_stats(self):
        """공분산 매트릭스 통계"""
        if len(self.covariances) == 0:
            return None, None, None, None

        recent_cov = self.covariances[-1]

        # 위치 불확실성 (x, y)
        pos_uncertainty_x = math.sqrt(recent_cov[0, 0])
        pos_uncertainty_y = math.sqrt(recent_cov[1, 1])

        # 방향 불확실성
        orientation_uncertainty = math.sqrt(recent_cov[5, 5])

        # 전체 위치 불확실성
        total_pos_uncertainty = math.sqrt(pos_uncertainty_x**2 + pos_uncertainty_y**2)

        return pos_uncertainty_x, pos_uncertainty_y, orientation_uncertainty, total_pos_uncertainty

    def calculate_movement_stats(self):
        """이동 통계 계산"""
        if len(self.position_history) < 2:
            return 0.0, 0.0

        # 현재 속도 추정 (최근 5개 포인트 기반)
        if len(self.position_history) >= 5 and len(self.pose_timestamps) >= 5:
            recent_positions = list(self.position_history)[-5:]
            recent_times = list(self.pose_timestamps)[-5:]

            time_diff = recent_times[-1] - recent_times[0]
            if time_diff > 0:
                distance_moved = math.sqrt(
                    (recent_positions[-1][0] - recent_positions[0][0])**2 +
                    (recent_positions[-1][1] - recent_positions[0][1])**2
                )
                current_velocity = distance_moved / time_diff
            else:
                current_velocity = 0.0
        else:
            current_velocity = 0.0

        return self.total_distance, current_velocity

    def print_statistics(self):
        """주기적으로 통계 출력"""
        if len(self.poses) < 5:
            self.get_logger().info("데이터 수집 중... (충분한 데이터 필요)")
            return

        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("로컬라이제이션 성능 통계")
        self.get_logger().info("="*80)

        # 1. 업데이트 성능
        freq, mean_interval, std_interval = self.calculate_update_rate()
        if freq is not None:
            self.get_logger().info(f"📊 업데이트 성능:")
            self.get_logger().info(f"   - 주파수: {freq:.2f} Hz")
            self.get_logger().info(f"   - 평균 간격: {mean_interval*1000:.1f} ms")
            self.get_logger().info(f"   - 간격 편차: ±{std_interval*1000:.1f} ms")

            if freq < 20:
                self.get_logger().warn("⚠️  업데이트 주파수가 낮습니다 (20Hz 이상 권장)")
            else:
                self.get_logger().info("✅ 업데이트 주파수 양호")

        # 2. 위치 분산 (안정성)
        var_x, var_y = self.calculate_position_variance()
        if var_x is not None:
            self.get_logger().info(f"\n📍 위치 안정성 (최근 50개 샘플):")
            self.get_logger().info(f"   - X축 분산: {var_x:.6f} m²")
            self.get_logger().info(f"   - Y축 분산: {var_y:.6f} m²")
            self.get_logger().info(f"   - 총 분산: {var_x + var_y:.6f} m²")

            if var_x + var_y > 0.001:  # 1mm²
                self.get_logger().warn("⚠️  위치 분산이 높습니다 (불안정)")
            else:
                self.get_logger().info("✅ 위치 추정 안정")

        # 3. 불확실성 (공분산)
        unc_x, unc_y, unc_theta, total_unc = self.calculate_covariance_stats()
        if unc_x is not None:
            self.get_logger().info(f"\n🎯 추정 불확실성:")
            self.get_logger().info(f"   - X축 불확실성: ±{unc_x:.3f} m")
            self.get_logger().info(f"   - Y축 불확실성: ±{unc_y:.3f} m")
            self.get_logger().info(f"   - 방향 불확실성: ±{math.degrees(unc_theta):.1f}°")
            self.get_logger().info(f"   - 총 위치 불확실성: ±{total_unc:.3f} m")

            if total_unc > 0.1:  # 10cm
                self.get_logger().warn("⚠️  위치 불확실성이 높습니다")
            else:
                self.get_logger().info("✅ 위치 불확실성 양호")

        # 4. 이동 통계
        total_dist, velocity = self.calculate_movement_stats()
        self.get_logger().info(f"\n🚗 이동 통계:")
        self.get_logger().info(f"   - 총 이동거리: {total_dist:.2f} m")
        self.get_logger().info(f"   - 현재 속도: {velocity:.2f} m/s ({velocity*3.6:.1f} km/h)")

        # 5. 데이터 수집 정보
        self.get_logger().info(f"\n📈 데이터 수집:")
        self.get_logger().info(f"   - 수집된 포즈: {len(self.poses)}개")
        self.get_logger().info(f"   - 수집 시간: {(self.pose_timestamps[-1] - self.pose_timestamps[0]):.1f}초")

def main():
    rclpy.init()

    print("\n" + "="*80)
    print("F1TENTH 로컬라이제이션 성능 테스트")
    print("="*80)
    print("이 도구는 AMCL 로컬라이제이션의 성능을 실시간으로 분석합니다.")
    print("\n측정 항목:")
    print("- 업데이트 주파수 및 안정성")
    print("- 위치 추정 분산 (안정성)")
    print("- 불확실성 (공분산 매트릭스)")
    print("- 이동 통계")
    print("\n로봇을 이동시키면서 결과를 관찰하세요.")
    print("Ctrl+C로 종료합니다.")
    print("="*80)

    node = LocalizationPerformanceTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n테스트가 종료되었습니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()