#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F1 RE 트랙을 이용한 원본 vs 최적화 경로 비교 시각화
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate, optimize
import os
import time

class F1REOptimizer:
    def __init__(self, track_file):
        self.track_file = track_file
        self.vehicle_params = {
            'width': 0.15,  # F1tenth 차량 폭 [m]
            'length': 0.33,  # 휠베이스 [m]
            'mass': 4.0,  # 차량 질량 [kg]
            'v_max': 20.0,  # 최대 속도 [m/s]
            'width_opt': 0.2  # 최적화용 안전폭 [m]
        }
        self.original_track = None
        self.interpolated_track = None
        self.optimized_path = None
        
    def load_and_convert_track(self):
        """F1 RE 트랙 로드 및 형식 변환"""
        # CSV 파일 읽기
        data = np.loadtxt(self.track_file, comments='#', delimiter=',', skiprows=1)
        print(f"F1 RE 트랙 데이터: {data.shape[0]}개 포인트")
        
        # 컬럼 순서: center_x, center_y, left_distance, right_distance
        # 변환: x, y, w_right, w_left
        converted_track = np.column_stack([
            data[:, 0],  # center_x
            data[:, 1],  # center_y  
            data[:, 3],  # right_distance
            data[:, 2]   # left_distance
        ])
        
        self.original_track = converted_track
        
        # 닫힌 루프 확인
        start_point = converted_track[0, :2]
        end_point = converted_track[-1, :2]
        distance = np.linalg.norm(start_point - end_point)
        
        print(f"닫힌 루프 거리: {distance:.3f}m")
        if distance > 0.5:
            # 닫힌 루프로 만들기
            self.original_track = np.vstack([converted_track, converted_track[0]])
            print("닫힌 루프로 변환 완료")
            
        return self.original_track
    
    def interpolate_track(self, stepsize=0.1):
        """트랙 보간"""
        centerline = self.original_track[:, :2]
        
        # 누적 거리 계산
        distances = np.zeros(len(centerline))
        for i in range(1, len(centerline)):
            distances[i] = distances[i-1] + np.linalg.norm(centerline[i] - centerline[i-1])
        
        total_length = distances[-1]
        num_points = int(total_length / stepsize)
        
        # 스플라인 보간
        try:
            tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], 
                                       s=10.0, per=1, k=3)
            u_new = np.linspace(0, 1, num_points)
            spline_points = interpolate.splev(u_new, tck)
            x_new = spline_points[0]
            y_new = spline_points[1]
            print("스플라인 보간 성공")
        except:
            # 선형 보간 사용
            new_distances = np.linspace(0, total_length, num_points)
            f_x = interpolate.interp1d(distances, centerline[:, 0], kind='linear')
            f_y = interpolate.interp1d(distances, centerline[:, 1], kind='linear')
            x_new = f_x(new_distances)
            y_new = f_y(new_distances)
            print("선형 보간 사용")
        
        # 트랙 폭 보간
        f_w_right = interpolate.interp1d(distances, self.original_track[:, 2], kind='linear')
        f_w_left = interpolate.interp1d(distances, self.original_track[:, 3], kind='linear')
        
        new_distances = np.linspace(0, total_length, num_points)
        w_right_new = f_w_right(new_distances)
        w_left_new = f_w_left(new_distances)
        
        self.interpolated_track = np.column_stack([x_new, y_new, w_right_new, w_left_new])
        
        print(f"트랙 보간 완료: {num_points}개 포인트")
        return self.interpolated_track
    
    def calculate_normal_vectors(self):
        """법선 벡터 계산"""
        points = self.interpolated_track[:, :2]
        n_points = len(points)
        
        # 접선 벡터 계산
        tangent_vectors = np.zeros_like(points)
        for i in range(n_points):
            if i == 0:
                tangent_vectors[i] = points[1] - points[-1]
            elif i == n_points - 1:
                tangent_vectors[i] = points[0] - points[i-1]
            else:
                tangent_vectors[i] = points[i+1] - points[i-1]
        
        # 정규화
        tangent_lengths = np.linalg.norm(tangent_vectors, axis=1)
        tangent_vectors = tangent_vectors / tangent_lengths.reshape(-1, 1)
        
        # 법선 벡터 (90도 회전)
        normal_vectors = np.zeros_like(tangent_vectors)
        normal_vectors[:, 0] = -tangent_vectors[:, 1]
        normal_vectors[:, 1] = tangent_vectors[:, 0]
        
        self.normal_vectors = normal_vectors
        return normal_vectors
    
    def calculate_curvature(self, path_points):
        """곡률 계산"""
        n_points = len(path_points)
        curvatures = np.zeros(n_points)
        
        for i in range(n_points):
            # 3점 기반 곡률 계산
            if i == 0:
                p1, p2, p3 = path_points[-1], path_points[i], path_points[i+1]
            elif i == n_points - 1:
                p1, p2, p3 = path_points[i-1], path_points[i], path_points[0]
            else:
                p1, p2, p3 = path_points[i-1], path_points[i], path_points[i+1]
            
            try:
                a = np.linalg.norm(p2 - p1)
                b = np.linalg.norm(p3 - p2)
                c = np.linalg.norm(p3 - p1)
                
                area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
                
                if area > 1e-10 and a > 1e-10 and b > 1e-10 and c > 1e-10:
                    curvatures[i] = 4 * area / (a * b * c)
                else:
                    curvatures[i] = 0.0
            except:
                curvatures[i] = 0.0
                
        return curvatures
    
    def optimize_minimum_curvature(self):
        """최소곡률 최적화"""
        points = self.interpolated_track[:, :2]
        normals = self.normal_vectors
        w_right = self.interpolated_track[:, 2]
        w_left = self.interpolated_track[:, 3]
        
        n_points = len(points)
        print("최소곡률 최적화 시작...")
        
        def objective_function(alpha):
            opt_points = points + alpha.reshape(-1, 1) * normals
            curvatures = self.calculate_curvature(opt_points)
            return np.sum(curvatures**2)
        
        # 초기값: 센터라인
        alpha0 = np.zeros(n_points)
        
        # 경계 조건 (더 여유있게 설정)
        bounds = []
        safety = self.vehicle_params['width_opt']
        
        for i in range(n_points):
            # 트랙 폭 확인
            total_width = w_right[i] + w_left[i]
            if total_width < safety:
                # 트랙이 너무 좁으면 센터라인 고정
                bounds.append((-0.001, 0.001))
            else:
                # 여유있는 경계 설정
                margin = 0.05  # 5cm 여유
                lb = max(-(w_left[i] - safety/2 - margin), -w_left[i] + 0.01)
                ub = min(w_right[i] - safety/2 - margin, w_right[i] - 0.01)
                bounds.append((lb, ub))
        
        try:
            # 간단한 최적화 (제약 조건 없이)
            result = optimize.minimize(
                objective_function,
                alpha0,
                method='L-BFGS-B',  # 더 안정적인 방법
                bounds=bounds,
                options={'disp': False, 'maxiter': 100}
            )
            
            if result.success:
                print("최적화 성공!")
                self.alpha_opt = result.x
                self.optimized_path = points + self.alpha_opt.reshape(-1, 1) * normals
            else:
                print(f"최적화 실패, 센터라인 사용: {result.message}")
                self.alpha_opt = alpha0
                self.optimized_path = points
                
        except Exception as e:
            print(f"최적화 오류: {e}")
            self.alpha_opt = alpha0
            self.optimized_path = points
        
        return self.optimized_path
    
    def plot_comparison(self):
        """원본 vs 최적화 경로 비교 플롯"""
        if self.original_track is None or self.optimized_path is None:
            print("데이터가 없습니다. 먼저 최적화를 실행하세요.")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        
        # 1. 메인 비교 플롯
        ax1 = axes[0, 0]
        
        # 원본 트랙 경계
        original_points = self.original_track[:, :2]
        w_right_orig = self.original_track[:, 2]
        w_left_orig = self.original_track[:, 3]
        
        # 간단한 법선 벡터 계산 (원본용)
        n_orig = len(original_points)
        normals_orig = np.zeros_like(original_points)
        
        for i in range(n_orig):
            if i == 0:
                tangent = original_points[1] - original_points[-1]
            elif i == n_orig - 1:
                tangent = original_points[0] - original_points[i-1]
            else:
                tangent = original_points[i+1] - original_points[i-1]
            
            # 정규화
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
                normals_orig[i] = [-tangent[1], tangent[0]]
        
        right_boundary = original_points + normals_orig * w_right_orig.reshape(-1, 1)
        left_boundary = original_points - normals_orig * w_left_orig.reshape(-1, 1)
        
        # 플롯
        ax1.plot(right_boundary[:, 0], right_boundary[:, 1], 'k--', linewidth=1, alpha=0.7, label='트랙 경계')
        ax1.plot(left_boundary[:, 0], left_boundary[:, 1], 'k--', linewidth=1, alpha=0.7)
        ax1.plot(original_points[:, 0], original_points[:, 1], 'b-', linewidth=2, label='원본 센터라인')
        ax1.plot(self.optimized_path[:, 0], self.optimized_path[:, 1], 'r-', linewidth=3, label='최적화된 레이싱라인')
        
        # 시작점 표시
        ax1.plot(original_points[0, 0], original_points[0, 1], 'go', markersize=8, label='시작점')
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('F1 RE: 원본 vs 최적화된 레이싱라인')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 곡률 비교
        ax2 = axes[0, 1]
        
        original_curvature = self.calculate_curvature(original_points)
        optimized_curvature = self.calculate_curvature(self.optimized_path)
        
        distances = np.arange(len(original_curvature))
        opt_distances = np.arange(len(optimized_curvature))
        
        ax2.plot(distances, original_curvature, 'b-', linewidth=2, label='원본 곡률')
        ax2.plot(opt_distances, optimized_curvature, 'r-', linewidth=2, label='최적화된 곡률')
        ax2.set_xlabel('포인트 인덱스')
        ax2.set_ylabel('곡률 [rad/m]')
        ax2.set_title('곡률 비교')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 트랙 폭 분포
        ax3 = axes[1, 0]
        
        track_widths = self.original_track[:, 2] + self.original_track[:, 3]
        ax3.plot(track_widths, 'g-', linewidth=2, label='전체 트랙 폭')
        ax3.axhline(y=self.vehicle_params['width_opt'], color='r', linestyle='--', label=f'차량 안전폭 ({self.vehicle_params["width_opt"]}m)')
        ax3.set_xlabel('포인트 인덱스')
        ax3.set_ylabel('폭 [m]')
        ax3.set_title('트랙 폭 분포')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 최적화 변위 (Alpha)
        ax4 = axes[1, 1]
        
        if hasattr(self, 'alpha_opt'):
            ax4.plot(self.alpha_opt, 'orange', linewidth=2, label='경로 오프셋')
            ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax4.set_xlabel('포인트 인덱스')
            ax4.set_ylabel('오프셋 [m]')
            ax4.set_title('최적화 경로 오프셋 (양수: 오른쪽, 음수: 왼쪽)')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, '최적화 데이터 없음', ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('최적화 오프셋')
        
        plt.tight_layout()
        
        # 저장
        os.makedirs("outputs", exist_ok=True)
        plt.savefig("outputs/f1_re_comparison.png", dpi=300, bbox_inches='tight')
        print("비교 결과가 outputs/f1_re_comparison.png에 저장되었습니다.")
        
        plt.show()
        
        # 통계 출력
        print(f"\n=== F1 RE 트랙 분석 결과 ===")
        print(f"원본 포인트 수: {len(original_points)}")
        print(f"최적화 포인트 수: {len(self.optimized_path)}")
        print(f"원본 최대 곡률: {np.max(original_curvature):.4f} rad/m")
        print(f"최적화 최대 곡률: {np.max(optimized_curvature):.4f} rad/m")
        print(f"원본 평균 곡률: {np.mean(np.abs(original_curvature)):.4f} rad/m")
        print(f"최적화 평균 곡률: {np.mean(np.abs(optimized_curvature)):.4f} rad/m")
        
        if hasattr(self, 'alpha_opt'):
            print(f"최대 오프셋: {np.max(np.abs(self.alpha_opt)):.3f} m")

def main():
    print("=== F1 RE 트랙 최소곡률 최적화 비교 ===")
    
    track_file = "inputs/tracks/f1_re"
    optimizer = F1REOptimizer(track_file)
    
    start_time = time.time()
    
    try:
        # 1. 트랙 로드
        print("\n1. F1 RE 트랙 로드 중...")
        optimizer.load_and_convert_track()
        
        # 2. 트랙 보간
        print("\n2. 트랙 보간 중...")
        optimizer.interpolate_track(stepsize=0.1)
        
        # 3. 법선 벡터 계산
        print("\n3. 법선 벡터 계산 중...")
        optimizer.calculate_normal_vectors()
        
        # 4. 최소곡률 최적화
        print("\n4. 최소곡률 최적화 중...")
        optimizer.optimize_minimum_curvature()
        
        # 5. 비교 플롯
        print("\n5. 결과 비교 시각화 중...")
        optimizer.plot_comparison()
        
        end_time = time.time()
        print(f"\n완료! 총 실행시간: {end_time - start_time:.2f}초")
        
    except Exception as e:
        print(f"오류 발생: {e}")
        return False
    
    return True

if __name__ == "__main__":
    main()