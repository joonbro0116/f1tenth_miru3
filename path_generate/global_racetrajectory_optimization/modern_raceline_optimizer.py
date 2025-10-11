#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
최신 라이브러리를 사용한 최소곡률 기반 글로벌 레이싱라인 최적화
Original TUMFTM code를 분석하여 현대적인 라이브러리로 재작성

Created by: Claude (Based on TUMFTM/global_racetrajectory_optimization)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate, optimize, spatial
from scipy.sparse import diags
# import pandas as pd  # pandas 불필요하므로 제거
import time
import os
import warnings
warnings.filterwarnings('ignore')

class ModernRacelineOptimizer:
    """최소곡률 기반 레이싱라인 최적화 클래스"""
    
    def __init__(self, track_file: str, vehicle_params: dict = None):
        """
        초기화
        
        Args:
            track_file: 트랙 CSV 파일 경로
            vehicle_params: 차량 파라미터 딕셔너리
        """
        self.track_file = track_file
        self.vehicle_params = vehicle_params or self._get_default_vehicle_params()
        self.reftrack = None
        self.optimized_path = None
        
    def _get_default_vehicle_params(self):
        """기본 차량 파라미터"""
        return {
            'width': 0.15,  # 차량 폭 [m] (F1tenth)
            'length': 0.33,  # 차량 길이 (휠베이스) [m]
            'mass': 4.0,  # 차량 질량 [kg] (F1tenth 추정)
            'v_max': 20.0,  # 최대 속도 [m/s] (소형차 적정)
            'curvlim': 2.0,  # 곡률 제한 [rad/m] (소형차는 더 민첩)
            'width_opt': 0.25  # 최적화용 차량 폭 (안전 거리 포함) [m]
        }
    
    def load_track(self):
        """트랙 데이터 로드"""
        try:
            # CSV 파일 읽기 (주석 제외)
            data = np.loadtxt(self.track_file, comments='#', delimiter=',')
            print(f"트랙 데이터 로드 완료: {data.shape[0]}개 포인트")
            
            if data.shape[1] == 4:
                # [x, y, w_right, w_left] 형식
                self.reftrack = data
            else:
                raise ValueError(f"트랙 파일 형식이 올바르지 않습니다. 4개 열이 필요하지만 {data.shape[1]}개 열이 있습니다.")
            
            # 닫힌 루프 확인
            start_point = self.reftrack[0, :2]
            end_point = self.reftrack[-1, :2]
            distance = np.linalg.norm(start_point - end_point)
            
            if distance > 0.1:  # 10cm 이상 차이나면
                print(f"경고: 트랙이 닫힌 루프가 아닙니다. 거리: {distance:.3f}m")
            
            # 최소 트랙 폭 확인
            min_width = np.min(self.reftrack[:, 2] + self.reftrack[:, 3])
            if min_width < self.vehicle_params['width'] + 0.5:
                print(f"경고: 최소 트랙 폭 {min_width:.2f}m가 차량 폭보다 작습니다!")
                
        except Exception as e:
            raise Exception(f"트랙 로드 실패: {e}")
    
    def interpolate_track(self, stepsize: float = 0.1):
        """트랙 보간"""
        if self.reftrack is None:
            raise Exception("먼저 트랙을 로드해주세요.")
        
        # 트랙 centerline
        centerline = self.reftrack[:, :2]
        
        # 닫힌 루프를 위해 첫 점을 마지막에 추가
        centerline_closed = np.vstack([centerline, centerline[0]])
        
        # 누적 거리 계산
        distances = np.zeros(centerline_closed.shape[0])
        for i in range(1, centerline_closed.shape[0]):
            distances[i] = distances[i-1] + np.linalg.norm(centerline_closed[i] - centerline_closed[i-1])
        
        total_length = distances[-1]
        
        # 균등한 간격으로 재샘플링
        num_points = int(total_length / stepsize)
        new_distances = np.linspace(0, total_length, num_points)
        
        # 스플라인 보간 (더 정교하게)
        try:
            # 2D 주기적 스플라인 사용 (닫힌 루프용)
            tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], 
                                       s=10.0,  # 스무딩 팩터 (0=보간, 큰값=스무딩)
                                       per=1,   # 주기적
                                       k=3)     # 3차 스플라인
            
            # 새로운 매개변수 값들
            u_new = np.linspace(0, 1, num_points)
            
            # 새로운 점들 계산
            spline_points = interpolate.splev(u_new, tck)
            x_new = spline_points[0]
            y_new = spline_points[1]
            
            print("주기적 스플라인 보간 성공")
            
        except Exception as e:
            print(f"스플라인 보간 실패 ({e}), 큐빅 보간 사용")
            # 큐빅 보간 시도
            try:
                f_x = interpolate.interp1d(distances[:-1], centerline[:, 0], kind='cubic', 
                                         fill_value='extrapolate')
                f_y = interpolate.interp1d(distances[:-1], centerline[:, 1], kind='cubic', 
                                         fill_value='extrapolate')
                
                x_new = f_x(new_distances)
                y_new = f_y(new_distances)
                print("큐빅 보간 성공")
            except:
                # 마지막 수단: 선형 보간
                print("큐빅 보간도 실패, 선형 보간 사용")
                f_x = interpolate.interp1d(distances[:-1], centerline[:, 0], kind='linear', 
                                         fill_value='extrapolate')
                f_y = interpolate.interp1d(distances[:-1], centerline[:, 1], kind='linear', 
                                         fill_value='extrapolate')
                
                x_new = f_x(new_distances)
                y_new = f_y(new_distances)
        
        # 트랙 폭 보간
        f_w_right = interpolate.interp1d(distances[:-1], self.reftrack[:, 2], kind='linear', 
                                        fill_value='extrapolate')
        f_w_left = interpolate.interp1d(distances[:-1], self.reftrack[:, 3], kind='linear', 
                                       fill_value='extrapolate')
        
        w_right_new = f_w_right(new_distances)
        w_left_new = f_w_left(new_distances)
        
        # 보간된 트랙 생성
        self.reftrack_interp = np.column_stack([x_new, y_new, w_right_new, w_left_new])
        
        print(f"트랙 보간 완료: {self.reftrack_interp.shape[0]}개 포인트")
        
        return self.reftrack_interp
    
    def calculate_normal_vectors(self):
        """법선 벡터 계산"""
        if not hasattr(self, 'reftrack_interp'):
            raise Exception("먼저 트랙을 보간해주세요.")
        
        points = self.reftrack_interp[:, :2]
        n_points = points.shape[0]
        
        # 접선 벡터 계산 (중앙 차분)
        tangent_vectors = np.zeros_like(points)
        
        for i in range(n_points):
            if i == 0:
                # 첫 번째 점
                tangent_vectors[i] = points[1] - points[-1]  # 닫힌 루프
            elif i == n_points - 1:
                # 마지막 점
                tangent_vectors[i] = points[0] - points[i-1]  # 닫힌 루프
            else:
                # 중간 점들
                tangent_vectors[i] = points[i+1] - points[i-1]
        
        # 접선 벡터 정규화
        tangent_lengths = np.linalg.norm(tangent_vectors, axis=1)
        tangent_vectors = tangent_vectors / tangent_lengths.reshape(-1, 1)
        
        # 법선 벡터 계산 (접선 벡터를 90도 회전)
        normal_vectors = np.zeros_like(tangent_vectors)
        normal_vectors[:, 0] = -tangent_vectors[:, 1]  # -sin
        normal_vectors[:, 1] = tangent_vectors[:, 0]   # cos
        
        self.normal_vectors = normal_vectors
        
        return normal_vectors
    
    def optimize_minimum_curvature(self):
        """최소곡률 최적화"""
        if not hasattr(self, 'normal_vectors'):
            raise Exception("먼저 법선 벡터를 계산해주세요.")
        
        points = self.reftrack_interp[:, :2]
        normals = self.normal_vectors
        w_right = self.reftrack_interp[:, 2]
        w_left = self.reftrack_interp[:, 3]
        
        n_points = points.shape[0]
        
        print("최소곡률 최적화 시작...")
        
        # 최적화 변수: alpha (각 점에서 법선 방향으로의 이동거리)
        # 양수: 오른쪽으로 이동, 음수: 왼쪽으로 이동
        
        def objective_function(alpha):
            """목적 함수: 곡률의 제곱합 최소화"""
            # 최적화된 경로 점들
            opt_points = points + alpha.reshape(-1, 1) * normals
            
            # 곡률 계산
            curvatures = self._calculate_curvature(opt_points)
            
            # 곡률의 제곱합 반환
            return np.sum(curvatures**2)
        
        def constraint_function(alpha):
            """제약 조건: 트랙 경계 내부에 있어야 함"""
            constraints = []
            
            # 오른쪽 경계 제약
            constraints.extend(w_right - alpha - self.vehicle_params['width_opt']/2)
            
            # 왼쪽 경계 제약  
            constraints.extend(w_left + alpha - self.vehicle_params['width_opt']/2)
            
            return np.array(constraints)
        
        # 초기값: 모든 점에서 중앙선
        alpha0 = np.zeros(n_points)
        
        # 경계 조건
        bounds = []
        for i in range(n_points):
            # alpha의 범위: [-w_left + safety, w_right - safety]
            safety = self.vehicle_params['width_opt'] / 2
            lb = -(w_left[i] - safety)
            ub = w_right[i] - safety
            bounds.append((lb, ub))
        
        # 제약 조건
        constraints = {'type': 'ineq', 'fun': constraint_function}
        
        # 최적화 실행
        try:
            result = optimize.minimize(
                objective_function,
                alpha0,
                method='SLSQP',
                bounds=bounds,
                constraints=constraints,
                options={'disp': True, 'maxiter': 1000}
            )
            
            if result.success:
                print("최적화 성공!")
                self.alpha_opt = result.x
                
                # 최적화된 경로 계산
                self.optimized_path = points + self.alpha_opt.reshape(-1, 1) * normals
                
                return self.optimized_path
            else:
                print(f"최적화 실패: {result.message}")
                # 실패시 중앙선 반환
                self.alpha_opt = alpha0
                self.optimized_path = points
                return self.optimized_path
                
        except Exception as e:
            print(f"최적화 중 오류 발생: {e}")
            # 오류시 중앙선 반환
            self.alpha_opt = alpha0
            self.optimized_path = points
            return self.optimized_path
    
    def _calculate_curvature(self, path_points):
        """경로 점들의 곡률 계산 (스플라인 기반)"""
        n_points = path_points.shape[0]
        
        # 닫힌 루프를 위해 첫 점을 마지막에 추가
        path_closed = np.vstack([path_points, path_points[0]])
        
        try:
            # 스플라인 피팅
            tck, u = interpolate.splprep([path_closed[:, 0], path_closed[:, 1]], 
                                       s=0, per=1, k=3)
            
            # 매개변수 값들
            u_points = np.linspace(0, 1, n_points)
            
            # 1차 미분 (속도 벡터)
            dx_du, dy_du = interpolate.splev(u_points, tck, der=1)
            
            # 2차 미분 (가속도 벡터)  
            d2x_du2, d2y_du2 = interpolate.splev(u_points, tck, der=2)
            
            # 곡률 계산: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            numerator = np.abs(dx_du * d2y_du2 - dy_du * d2x_du2)
            denominator = np.power(dx_du**2 + dy_du**2, 1.5)
            
            # 분모가 0인 경우 처리
            curvatures = np.where(denominator > 1e-10, numerator / denominator, 0.0)
            
        except:
            # 스플라인 실패시 3점 기반 계산으로 대체
            print("스플라인 곡률 계산 실패, 3점 기반 계산 사용")
            curvatures = np.zeros(n_points)
            
            for i in range(n_points):
                # 3점을 이용한 곡률 계산
                if i == 0:
                    p1, p2, p3 = path_points[-1], path_points[i], path_points[i+1]
                elif i == n_points - 1:
                    p1, p2, p3 = path_points[i-1], path_points[i], path_points[0]
                else:
                    p1, p2, p3 = path_points[i-1], path_points[i], path_points[i+1]
                
                # 곡률 계산 (3점을 지나는 원의 곡률)
                try:
                    a = np.linalg.norm(p2 - p1)
                    b = np.linalg.norm(p3 - p2)
                    c = np.linalg.norm(p3 - p1)
                    
                    # 면적 계산 (외적)
                    area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
                    
                    if area > 1e-10 and a > 1e-10 and b > 1e-10 and c > 1e-10:
                        curvatures[i] = 4 * area / (a * b * c)
                    else:
                        curvatures[i] = 0.0
                        
                except:
                    curvatures[i] = 0.0
        
        return curvatures
    
    def calculate_velocity_profile(self):
        """속도 프로파일 계산 (간단한 모델)"""
        if self.optimized_path is None:
            raise Exception("먼저 경로를 최적화해주세요.")
        
        # 곡률 계산
        curvatures = self._calculate_curvature(self.optimized_path)
        
        # 최대 허용 속도 (곡률 기반)
        # v_max = sqrt(a_y_max / curvature)
        a_y_max = 9.81 * 0.8  # 최대 횡가속도 (0.8g)
        
        velocities = np.zeros(len(curvatures))
        for i, kappa in enumerate(curvatures):
            if kappa > 1e-6:
                v_curve = np.sqrt(a_y_max / kappa)
                velocities[i] = min(v_curve, self.vehicle_params['v_max'])
            else:
                velocities[i] = self.vehicle_params['v_max']
        
        # 속도 스무딩 (이동평균)
        window_size = 5
        velocities_smooth = np.convolve(velocities, np.ones(window_size)/window_size, mode='same')
        
        self.velocities = velocities_smooth
        
        return velocities_smooth
    
    def export_trajectory(self, output_file: str):
        """최적화된 궤적을 파일로 내보내기"""
        if self.optimized_path is None:
            raise Exception("먼저 경로를 최적화해주세요.")
        
        # 거리 계산
        distances = np.zeros(len(self.optimized_path))
        for i in range(1, len(self.optimized_path)):
            distances[i] = distances[i-1] + np.linalg.norm(self.optimized_path[i] - self.optimized_path[i-1])
        
        # 헤딩 계산
        headings = np.zeros(len(self.optimized_path))
        for i in range(len(self.optimized_path)):
            if i == len(self.optimized_path) - 1:
                heading_vec = self.optimized_path[0] - self.optimized_path[i]
            else:
                heading_vec = self.optimized_path[i+1] - self.optimized_path[i]
            headings[i] = np.arctan2(heading_vec[1], heading_vec[0])
        
        # 곡률 계산
        curvatures = self._calculate_curvature(self.optimized_path)
        
        # 속도 계산 (없으면 기본값)
        if not hasattr(self, 'velocities'):
            self.calculate_velocity_profile()
        
        # 가속도 계산 (간단한 모델)
        accelerations = np.zeros(len(self.velocities))
        
        # 궤적 데이터 구성
        trajectory = np.column_stack([
            distances,  # s_m
            self.optimized_path[:, 0],  # x_m
            self.optimized_path[:, 1],  # y_m
            headings,  # psi_rad
            curvatures,  # kappa_radpm
            self.velocities,  # vx_mps
            accelerations  # ax_mps2
        ])
        
        # 파일 저장
        header = "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2"
        np.savetxt(output_file, trajectory, delimiter=',', header=header, comments='')
        
        print(f"궤적이 {output_file}에 저장되었습니다.")
        
        return trajectory
    
    def plot_results(self, save_figure: bool = True, output_dir: str = "outputs"):
        """결과 시각화"""
        if self.optimized_path is None:
            raise Exception("먼저 경로를 최적화해주세요.")
        
        # 출력 디렉토리 생성
        os.makedirs(output_dir, exist_ok=True)
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # 1. 트랙과 최적화된 경로
        ax1 = axes[0, 0]
        
        # 트랙 경계
        track_points = self.reftrack_interp[:, :2]
        normals = self.normal_vectors
        w_right = self.reftrack_interp[:, 2]
        w_left = self.reftrack_interp[:, 3]
        
        right_boundary = track_points + normals * w_right.reshape(-1, 1)
        left_boundary = track_points - normals * w_left.reshape(-1, 1)
        
        ax1.plot(right_boundary[:, 0], right_boundary[:, 1], 'k--', linewidth=1, label='트랙 경계')
        ax1.plot(left_boundary[:, 0], left_boundary[:, 1], 'k--', linewidth=1)
        ax1.plot(track_points[:, 0], track_points[:, 1], 'b-', linewidth=2, label='센터라인')
        ax1.plot(self.optimized_path[:, 0], self.optimized_path[:, 1], 'r-', linewidth=3, label='최적화된 경로')
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('최소곡률 기반 레이싱라인')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # 2. 곡률 분포
        ax2 = axes[0, 1]
        curvatures = self._calculate_curvature(self.optimized_path)
        distances = np.arange(len(curvatures))
        
        ax2.plot(distances, curvatures, 'g-', linewidth=2)
        ax2.set_xlabel('거리 [포인트]')
        ax2.set_ylabel('곡률 [rad/m]')
        ax2.set_title('곡률 프로파일')
        ax2.grid(True)
        
        # 3. 속도 프로파일
        ax3 = axes[1, 0]
        if hasattr(self, 'velocities'):
            ax3.plot(distances, self.velocities * 3.6, 'purple', linewidth=2)  # km/h로 변환
        else:
            self.calculate_velocity_profile()
            ax3.plot(distances, self.velocities * 3.6, 'purple', linewidth=2)
            
        ax3.set_xlabel('거리 [포인트]')
        ax3.set_ylabel('속도 [km/h]')
        ax3.set_title('속도 프로파일')
        ax3.grid(True)
        
        # 4. 최적화 변수 (alpha)
        ax4 = axes[1, 1]
        if hasattr(self, 'alpha_opt'):
            ax4.plot(distances, self.alpha_opt, 'orange', linewidth=2)
            ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax4.set_xlabel('거리 [포인트]')
            ax4.set_ylabel('Alpha [m]')
            ax4.set_title('경로 오프셋 (양수: 오른쪽, 음수: 왼쪽)')
            ax4.grid(True)
        
        plt.tight_layout()
        
        if save_figure:
            plt.savefig(os.path.join(output_dir, 'raceline_optimization_results.png'), dpi=300, bbox_inches='tight')
            print(f"결과 그림이 {output_dir}/raceline_optimization_results.png에 저장되었습니다.")
        
        plt.show()

def main():
    """메인 함수"""
    print("=== 최신 라이브러리 기반 최소곡률 레이싱라인 최적화 ===")
    
    # 트랙 파일 경로
    track_file = "inputs/tracks/test_track_reorder.csv"
    
    # 최적화 객체 생성
    optimizer = ModernRacelineOptimizer(track_file)
    
    # 시작 시간
    start_time = time.time()
    
    try:
        # 1. 트랙 로드
        print("\n1. 트랙 로드 중...")
        optimizer.load_track()
        
        # 2. 트랙 보간
        print("\n2. 트랙 보간 중...")
        optimizer.interpolate_track(stepsize=0.1)
        
        # 3. 법선 벡터 계산
        print("\n3. 법선 벡터 계산 중...")
        optimizer.calculate_normal_vectors()
        
        # 4. 최소곡률 최적화
        print("\n4. 최소곡률 최적화 중...")
        optimized_path = optimizer.optimize_minimum_curvature()
        
        # 5. 속도 프로파일 계산
        print("\n5. 속도 프로파일 계산 중...")
        optimizer.calculate_velocity_profile()
        
        # 6. 결과 저장
        print("\n6. 결과 저장 중...")
        os.makedirs("outputs", exist_ok=True)
        trajectory = optimizer.export_trajectory("outputs/optimized_raceline.csv")
        
        # 7. 결과 시각화
        print("\n7. 결과 시각화 중...")
        optimizer.plot_results()
        
        # 완료 시간
        end_time = time.time()
        print(f"\n완료! 총 실행시간: {end_time - start_time:.2f}초")
        print(f"최적화된 궤적이 outputs/optimized_raceline.csv에 저장되었습니다.")
        
        # 통계 출력
        curvatures = optimizer._calculate_curvature(optimized_path)
        print(f"\n=== 결과 통계 ===")
        print(f"포인트 수: {len(optimized_path)}")
        print(f"최대 곡률: {np.max(curvatures):.4f} rad/m")
        print(f"평균 곡률: {np.mean(np.abs(curvatures)):.4f} rad/m")
        print(f"최대 속도: {np.max(optimizer.velocities)*3.6:.1f} km/h")
        print(f"평균 속도: {np.mean(optimizer.velocities)*3.6:.1f} km/h")
        
    except Exception as e:
        print(f"오류 발생: {e}")
        return False
    
    return True

if __name__ == "__main__":
    main()