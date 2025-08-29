#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F1 RE 최적화된 궤적을 CSV 파일로 저장
"""

import numpy as np
import os
from f1_re_accurate_visualization import AccurateF1REOptimizer

def export_f1_re_trajectory():
    """F1 RE 최적화된 궤적 저장"""
    
    print("=== F1 RE 최적화 궤적 저장 ===")
    
    track_file = "inputs/tracks/0826-2.csv"
    optimizer = AccurateF1REOptimizer(track_file)
    
    # 1. 트랙 처리
    print("1. 트랙 로드 및 처리 중...")
    optimizer.load_and_convert_track()
    optimizer.interpolate_track(stepsize=0.05)
    optimizer.optimize_minimum_curvature()
    
    # 2. 궤적 데이터 생성
    print("2. 궤적 데이터 생성 중...")
    
    # 거리 계산
    distances = np.zeros(len(optimizer.optimized_path))
    for i in range(1, len(optimizer.optimized_path)):
        distances[i] = distances[i-1] + np.linalg.norm(
            optimizer.optimized_path[i] - optimizer.optimized_path[i-1]
        )
    
    # 헤딩 계산
    headings = np.zeros(len(optimizer.optimized_path))
    for i in range(len(optimizer.optimized_path)):
        if i == len(optimizer.optimized_path) - 1:
            heading_vec = optimizer.optimized_path[0] - optimizer.optimized_path[i]
        else:
            heading_vec = optimizer.optimized_path[i+1] - optimizer.optimized_path[i]
        headings[i] = np.arctan2(heading_vec[1], heading_vec[0])
    
    # 곡률 계산
    curvatures = optimizer.calculate_curvature(optimizer.optimized_path)
    
    # 속도 계산 (곡률 기반)
    a_y_max = 9.81 * 0.6  # F1tenth 적정 횡가속도
    velocities = np.zeros(len(curvatures))
    
    for i, kappa in enumerate(curvatures):
        if kappa > 1e-6:
            v_curve = np.sqrt(a_y_max / kappa)
            velocities[i] = min(v_curve, optimizer.vehicle_params['v_max'])
        else:
            velocities[i] = optimizer.vehicle_params['v_max']
    
    # 속도 스무딩
    window_size = 7
    velocities_smooth = np.convolve(velocities, np.ones(window_size)/window_size, mode='same')
    
    # 가속도 계산 (간단한 모델)
    accelerations = np.zeros(len(velocities_smooth))
    dt = 0.1  # 예상 시간 간격
    
    for i in range(1, len(velocities_smooth)):
        accelerations[i] = (velocities_smooth[i] - velocities_smooth[i-1]) / dt
    
    # 궤적 데이터 구성
    trajectory = np.column_stack([
        distances,  # s_m
        optimizer.optimized_path[:, 0],  # x_m
        optimizer.optimized_path[:, 1],  # y_m
        headings,  # psi_rad
        curvatures,  # kappa_radpm
        velocities_smooth,  # vx_mps
        accelerations  # ax_mps2
    ])
    
    # 3. 파일 저장
    print("3. 파일 저장 중...")
    
    os.makedirs("outputs", exist_ok=True)
    
    # 최적화된 궤적 저장
    header = "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2"
    np.savetxt("outputs/f1_re_optimized_trajectory.csv", trajectory, 
               delimiter=',', header=header, comments='', fmt='%.6f')
    
    # 통계 정보 저장
    with open("outputs/f1_re_trajectory_stats.txt", 'w') as f:
        f.write("=== F1 RE 최적화 궤적 통계 ===\n")
        f.write(f"포인트 수: {len(trajectory)}\n")
        f.write(f"총 거리: {distances[-1]:.2f} m\n")
        f.write(f"최대 곡률: {np.max(curvatures):.4f} rad/m\n")
        f.write(f"평균 곡률: {np.mean(np.abs(curvatures)):.4f} rad/m\n")
        f.write(f"최대 속도: {np.max(velocities_smooth)*3.6:.1f} km/h\n")
        f.write(f"평균 속도: {np.mean(velocities_smooth)*3.6:.1f} km/h\n")
        f.write(f"최대 가속도: {np.max(np.abs(accelerations)):.2f} m/s²\n")
        f.write(f"\n=== 차량 파라미터 ===\n")
        f.write(f"차량 폭: {optimizer.vehicle_params['width']} m\n")
        f.write(f"휠베이스: {optimizer.vehicle_params['length']} m\n")
        f.write(f"차량 질량: {optimizer.vehicle_params['mass']} kg\n")
        f.write(f"최대 속도: {optimizer.vehicle_params['v_max']} m/s\n")
        f.write(f"안전 폭: {optimizer.vehicle_params['width_opt']} m\n")
    
    print(f"\n✅ 저장 완료!")
    print(f"   - 궤적 파일: outputs/f1_re_optimized_trajectory.csv")
    print(f"   - 통계 파일: outputs/f1_re_trajectory_stats.txt")
    print(f"   - 비교 이미지: outputs/f1_re_accurate_comparison.png")
    
    # 4. 간단한 통계 출력
    print(f"\n=== 최종 궤적 통계 ===")
    print(f"포인트 수: {len(trajectory)}")
    print(f"총 거리: {distances[-1]:.2f} m")
    print(f"해상도: {distances[-1]/len(trajectory)*1000:.1f} mm/point")
    print(f"최대 곡률: {np.max(curvatures):.4f} rad/m")
    print(f"평균 곡률: {np.mean(np.abs(curvatures)):.4f} rad/m")
    print(f"최대 속도: {np.max(velocities_smooth)*3.6:.1f} km/h")
    print(f"평균 속도: {np.mean(velocities_smooth)*3.6:.1f} km/h")
    print(f"예상 랩타임: {distances[-1]/np.mean(velocities_smooth):.1f}초")

if __name__ == "__main__":
    export_f1_re_trajectory()
