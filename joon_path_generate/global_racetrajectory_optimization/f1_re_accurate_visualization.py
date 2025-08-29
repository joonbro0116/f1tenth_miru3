#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F1 RE 트랙 정확한 시각화 - 스플라인 기반 법선 벡터 계산
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate, optimize
import os
import time

class AccurateF1REOptimizer:
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
    
    def calculate_accurate_normals(self, centerline):
        """스플라인 기반 정확한 법선 벡터 계산"""
        n_points = len(centerline)
        
        try:
            # 닫힌 루프 스플라인 피팅
            tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], 
                                       s=0, per=1, k=3)
            
            # 각 점에서의 매개변수 값 계산
            u_points = np.linspace(0, 1, n_points)
            
            # 1차 미분으로 접선 벡터 구하기
            dx_du, dy_du = interpolate.splev(u_points, tck, der=1)
            
            # 접선 벡터 정규화
            tangent_lengths = np.sqrt(dx_du**2 + dy_du**2)
            tangent_vectors = np.column_stack([dx_du / tangent_lengths, dy_du / tangent_lengths])
            
            # 법선 벡터 (접선을 90도 반시계방향 회전)
            normal_vectors = np.column_stack([-tangent_vectors[:, 1], tangent_vectors[:, 0]])
            
            print("스플라인 기반 정확한 법선 벡터 계산 완료")
            return normal_vectors
            
        except Exception as e:
            print(f"스플라인 법선 계산 실패: {e}, 간단한 방법 사용")
            # 백업: 간단한 중앙차분
            normal_vectors = np.zeros_like(centerline)
            
            for i in range(n_points):
                if i == 0:
                    tangent = centerline[1] - centerline[-1]
                elif i == n_points - 1:
                    tangent = centerline[0] - centerline[i-1]
                else:
                    tangent = centerline[i+1] - centerline[i-1]
                
                # 정규화
                tangent_len = np.linalg.norm(tangent)
                if tangent_len > 0:
                    tangent = tangent / tangent_len
                    normal_vectors[i] = [-tangent[1], tangent[0]]
            
            return normal_vectors
    
    def interpolate_track(self, stepsize=0.05):
        """더 조밀한 트랙 보간"""
        centerline = self.original_track[:, :2]
        
        # 누적 거리 계산
        distances = np.zeros(len(centerline))
        for i in range(1, len(centerline)):
            distances[i] = distances[i-1] + np.linalg.norm(centerline[i] - centerline[i-1])
        
        total_length = distances[-1]
        num_points = int(total_length / stepsize)
        print(f"보간 후 예상 포인트 수: {num_points}")
        
        # 스플라인 보간
        try:
            tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], 
                                       s=5.0, per=1, k=3)  # 약간의 스무딩
            u_new = np.linspace(0, 1, num_points)
            spline_points = interpolate.splev(u_new, tck)
            x_new = spline_points[0]
            y_new = spline_points[1]
            print("스플라인 보간 성공")
        except Exception as e:
            print(f"스플라인 보간 실패: {e}")
            # 선형 보간 사용
            new_distances = np.linspace(0, total_length, num_points)
            f_x = interpolate.interp1d(distances, centerline[:, 0], kind='cubic')
            f_y = interpolate.interp1d(distances, centerline[:, 1], kind='cubic')
            x_new = f_x(new_distances)
            y_new = f_y(new_distances)
            print("큐빅 보간 사용")
        
        # 트랙 폭 보간
        f_w_right = interpolate.interp1d(distances, self.original_track[:, 2], kind='linear')
        f_w_left = interpolate.interp1d(distances, self.original_track[:, 3], kind='linear')
        
        new_distances = np.linspace(0, total_length, num_points)
        w_right_new = f_w_right(new_distances)
        w_left_new = f_w_left(new_distances)
        
        self.interpolated_track = np.column_stack([x_new, y_new, w_right_new, w_left_new])
        
        # 정확한 법선 벡터 계산
        self.normal_vectors = self.calculate_accurate_normals(self.interpolated_track[:, :2])
        
        print(f"트랙 보간 완료: {num_points}개 포인트")
        return self.interpolated_track
    
    def calculate_curvature(self, path_points):
        """스플라인 기반 곡률 계산"""
        try:
            # 스플라인 피팅
            tck, u = interpolate.splprep([path_points[:, 0], path_points[:, 1]], 
                                       s=0, per=1, k=3)
            
            n_points = len(path_points)
            u_points = np.linspace(0, 1, n_points)
            
            # 1차, 2차 미분
            dx_du, dy_du = interpolate.splev(u_points, tck, der=1)
            d2x_du2, d2y_du2 = interpolate.splev(u_points, tck, der=2)
            
            # 곡률 공식: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            numerator = np.abs(dx_du * d2y_du2 - dy_du * d2x_du2)
            denominator = np.power(dx_du**2 + dy_du**2, 1.5)
            
            curvatures = np.where(denominator > 1e-10, numerator / denominator, 0.0)
            return curvatures
            
        except:
            # 백업: 3점 기반
            n_points = len(path_points)
            curvatures = np.zeros(n_points)
            
            for i in range(n_points):
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
        
        # 더 보수적인 경계 조건
        bounds = []
        safety = self.vehicle_params['width_opt']
        
        for i in range(n_points):
            total_width = w_right[i] + w_left[i]
            if total_width < safety * 1.2:  # 안전 마진
                bounds.append((-0.001, 0.001))  # 거의 고정
            else:
                margin = 0.1  # 10cm 여유
                lb = max(-(w_left[i] - safety/2 - margin), -w_left[i] * 0.8)
                ub = min(w_right[i] - safety/2 - margin, w_right[i] * 0.8)
                bounds.append((lb, ub))
        
        try:
            result = optimize.minimize(
                objective_function,
                alpha0,
                method='L-BFGS-B',
                bounds=bounds,
                options={'disp': False, 'maxiter': 200}
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
    
    def plot_detailed_comparison(self):
        """상세한 비교 플롯"""
        if self.original_track is None or self.optimized_path is None:
            print("데이터가 없습니다. 먼저 최적화를 실행하세요.")
            return
        
        plt.rcParams['font.family'] = 'DejaVu Sans'  # 영어 폰트 강제 사용
        
        fig, axes = plt.subplots(2, 2, figsize=(18, 14))
        
        # 1. 정확한 트랙 경계를 가진 메인 플롯
        ax1 = axes[0, 0]
        
        # 원본 트랙
        original_points = self.original_track[:, :2]
        w_right_orig = self.original_track[:, 2]
        w_left_orig = self.original_track[:, 3]
        
        # 원본 트랙의 정확한 법선 벡터 계산
        original_normals = self.calculate_accurate_normals(original_points)
        
        # 정확한 경계 계산
        right_boundary = original_points + original_normals * w_right_orig.reshape(-1, 1)
        left_boundary = original_points - original_normals * w_left_orig.reshape(-1, 1)
        
        # 보간된 트랙 경계
        interp_points = self.interpolated_track[:, :2]
        w_right_interp = self.interpolated_track[:, 2]
        w_left_interp = self.interpolated_track[:, 3]
        
        interp_right_boundary = interp_points + self.normal_vectors * w_right_interp.reshape(-1, 1)
        interp_left_boundary = interp_points - self.normal_vectors * w_left_interp.reshape(-1, 1)
        
        # 플롯
        ax1.plot(right_boundary[:, 0], right_boundary[:, 1], 'k--', linewidth=2, alpha=0.7, label='Original Track Boundary')
        ax1.plot(left_boundary[:, 0], left_boundary[:, 1], 'k--', linewidth=2, alpha=0.7)
        ax1.plot(interp_right_boundary[:, 0], interp_right_boundary[:, 1], 'gray', linewidth=1, alpha=0.5, label='Interpolated Boundary')
        ax1.plot(interp_left_boundary[:, 0], interp_left_boundary[:, 1], 'gray', linewidth=1, alpha=0.5)
        
        ax1.plot(original_points[:, 0], original_points[:, 1], 'b-', linewidth=2, label='Original Centerline', alpha=0.7)
        ax1.plot(self.optimized_path[:, 0], self.optimized_path[:, 1], 'r-', linewidth=3, label='Optimized Racing Line')
        
        # 시작점
        ax1.plot(original_points[0, 0], original_points[0, 1], 'go', markersize=8, label='Start Point')
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('F1 RE Track: Original vs Optimized Racing Line')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 곡률 비교
        ax2 = axes[0, 1]
        
        original_curvature = self.calculate_curvature(original_points)
        optimized_curvature = self.calculate_curvature(self.optimized_path)
        
        distances_orig = np.arange(len(original_curvature))
        distances_opt = np.arange(len(optimized_curvature))
        
        ax2.plot(distances_orig, original_curvature, 'b-', linewidth=2, label='Original Curvature', alpha=0.7)
        ax2.plot(distances_opt, optimized_curvature, 'r-', linewidth=2, label='Optimized Curvature')
        ax2.set_xlabel('Point Index')
        ax2.set_ylabel('Curvature [rad/m]')
        ax2.set_title('Curvature Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 트랙 폭과 차량 폭 비교
        ax3 = axes[1, 0]
        
        track_widths = self.original_track[:, 2] + self.original_track[:, 3]
        ax3.plot(track_widths, 'g-', linewidth=2, label='Total Track Width')
        ax3.axhline(y=self.vehicle_params['width_opt'], color='r', linestyle='--', label=f'Vehicle Safety Width ({self.vehicle_params["width_opt"]}m)')
        ax3.axhline(y=self.vehicle_params['width'], color='orange', linestyle=':', label=f'Vehicle Width ({self.vehicle_params["width"]}m)')
        ax3.set_xlabel('Point Index')
        ax3.set_ylabel('Width [m]')
        ax3.set_title('Track Width Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 최적화 변위와 제약 조건
        ax4 = axes[1, 1]
        
        if hasattr(self, 'alpha_opt'):
            ax4.plot(self.alpha_opt, 'orange', linewidth=2, label='Path Offset')
            ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Centerline')
            
            # 제약 조건 표시
            w_right_interp = self.interpolated_track[:, 2]
            w_left_interp = self.interpolated_track[:, 3]
            
            upper_bound = w_right_interp - self.vehicle_params['width_opt']/2
            lower_bound = -(w_left_interp - self.vehicle_params['width_opt']/2)
            
            ax4.fill_between(range(len(upper_bound)), lower_bound, upper_bound, 
                           alpha=0.2, color='gray', label='Feasible Region')
            
            ax4.set_xlabel('Point Index')
            ax4.set_ylabel('Offset [m]')
            ax4.set_title('Optimization Path Offset (Positive: Right, Negative: Left)')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'No Optimization Data', ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('Optimization Offset')
        
        plt.tight_layout()
        
        # 저장
        os.makedirs("outputs", exist_ok=True)
        plt.savefig("outputs/f1_re_accurate_comparison.png", dpi=300, bbox_inches='tight')
        print("정확한 비교 결과가 outputs/f1_re_accurate_comparison.png에 저장되었습니다.")
        
        plt.show()
        
        # 통계 출력
        print(f"\n=== F1 RE 정확한 트랙 분석 결과 ===")
        print(f"원본 포인트 수: {len(original_points)}")
        print(f"보간 포인트 수: {len(self.interpolated_track)}")
        print(f"최적화 포인트 수: {len(self.optimized_path)}")
        print(f"원본 최대 곡률: {np.max(original_curvature):.4f} rad/m")
        print(f"최적화 최대 곡률: {np.max(optimized_curvature):.4f} rad/m")
        print(f"원본 평균 곡률: {np.mean(np.abs(original_curvature)):.4f} rad/m")
        print(f"최적화 평균 곡률: {np.mean(np.abs(optimized_curvature)):.4f} rad/m")
        
        if hasattr(self, 'alpha_opt'):
            print(f"최대 오프셋: {np.max(np.abs(self.alpha_opt)):.3f} m")
            print(f"평균 절대 오프셋: {np.mean(np.abs(self.alpha_opt)):.3f} m")

def main():
    print("=== F1 RE 트랙 정확한 시각화 및 최적화 ===")
    
    track_file = "inputs/tracks/0826-2.csv"
    optimizer = AccurateF1REOptimizer(track_file)
    
    start_time = time.time()
    
    try:
        # 1. 트랙 로드
        print("\n1. F1 RE 트랙 로드 중...")
        optimizer.load_and_convert_track()
        
        # 2. 정밀한 트랙 보간
        print("\n2. 정밀한 트랙 보간 중...")
        optimizer.interpolate_track(stepsize=0.05)  # 더 조밀하게
        
        # 3. 최소곡률 최적화
        print("\n3. 최소곡률 최적화 중...")
        optimizer.optimize_minimum_curvature()
        
        # 4. 정확한 비교 플롯
        print("\n4. 정확한 결과 비교 시각화 중...")
        optimizer.plot_detailed_comparison()
        
        end_time = time.time()
        print(f"\n완료! 총 실행시간: {end_time - start_time:.2f}초")
        
    except Exception as e:
        print(f"오류 발생: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    main()
