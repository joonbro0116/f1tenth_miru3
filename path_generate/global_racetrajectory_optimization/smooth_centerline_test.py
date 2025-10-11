#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
centerline을 spline으로 부드럽게 만들고 균등하게 샘플링하는 테스트
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import os

def smooth_and_resample_centerline():
    """centerline을 spline으로 부드럽게 만들고 재샘플링"""
    
    # F1 RE 데이터 로드
    data = np.loadtxt("inputs/tracks/f1_re", comments='#', delimiter=',', skiprows=1)
    print(f"원본 데이터 크기: {data.shape}")
    
    centerline = data[:, :2]  # center_x, center_y
    left_dist = data[:, 2]    # left_distance  
    right_dist = data[:, 3]   # right_distance
    
    # 닫힌 루프로 만들기
    start_point = centerline[0]
    end_point = centerline[-1]
    if np.linalg.norm(start_point - end_point) > 0.5:
        centerline = np.vstack([centerline, centerline[0]])
        left_dist = np.append(left_dist, left_dist[0])
        right_dist = np.append(right_dist, right_dist[0])
    
    print(f"닫힌 루프 데이터 크기: {centerline.shape}")
    
    # 원본 centerline의 누적 거리 계산
    distances = np.zeros(len(centerline))
    for i in range(1, len(centerline)):
        distances[i] = distances[i-1] + np.linalg.norm(centerline[i] - centerline[i-1])
    
    total_distance = distances[-1]
    print(f"총 트랙 길이: {total_distance:.2f} m")
    
    # 스플라인 보간 (periodic spline)
    try:
        # 주기적 스플라인으로 부드러운 centerline 생성
        tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], 
                                   s=0.5,  # smoothing factor (0=no smoothing)
                                   per=1,  # periodic
                                   k=3)    # cubic spline
        
        # 원하는 해상도로 재샘플링 (예: 0.1m 간격)
        sample_distance = 0.1  # m
        num_samples = int(total_distance / sample_distance)
        u_new = np.linspace(0, 1, num_samples)
        
        # 새로운 centerline 점들 생성
        smooth_centerline = np.array(interpolate.splev(u_new, tck)).T
        
        print(f"부드러운 centerline 포인트 수: {len(smooth_centerline)}")
        
        # left_dist와 right_dist도 보간
        # 원본 u 파라미터를 사용하여 거리값들 보간
        u_original = np.linspace(0, 1, len(left_dist))
        left_interp = interpolate.interp1d(u_original, left_dist, kind='cubic', 
                                         fill_value='extrapolate')
        right_interp = interpolate.interp1d(u_original, right_dist, kind='cubic',
                                          fill_value='extrapolate')
        
        smooth_left_dist = left_interp(u_new)
        smooth_right_dist = right_interp(u_new)
        
        # 시각화
        plt.figure(figsize=(15, 10))
        
        # 원본과 부드러운 centerline 비교
        plt.subplot(2, 2, 1)
        plt.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Original Centerline')
        plt.plot(smooth_centerline[:, 0], smooth_centerline[:, 1], 'r-', linewidth=1, label='Smooth Centerline')
        plt.plot(centerline[0, 0], centerline[0, 1], 'go', markersize=8, label='Start')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('원본 vs 부드러운 Centerline')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # 원본 centerline 확대
        plt.subplot(2, 2, 2)
        plt.plot(centerline[:50, 0], centerline[:50, 1], 'b.-', linewidth=2, markersize=4, label='Original')
        plt.plot(smooth_centerline[:500, 0], smooth_centerline[:500, 1], 'r-', linewidth=1, label='Smooth')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('확대된 비교 (일부분)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # 거리 분포 비교
        plt.subplot(2, 2, 3)
        plt.plot(left_dist, 'b-', label='Original Left Distance')
        plt.plot(smooth_left_dist, 'r-', alpha=0.7, label='Smooth Left Distance')
        plt.xlabel('Point Index')
        plt.ylabel('Distance [m]')
        plt.title('Left Distance 비교')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.subplot(2, 2, 4)
        plt.plot(right_dist, 'b-', label='Original Right Distance')
        plt.plot(smooth_right_dist, 'r-', alpha=0.7, label='Smooth Right Distance')
        plt.xlabel('Point Index')
        plt.ylabel('Distance [m]')
        plt.title('Right Distance 비교')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        os.makedirs("outputs", exist_ok=True)
        plt.savefig("outputs/smooth_centerline_test.png", dpi=300, bbox_inches='tight')
        print("결과가 outputs/smooth_centerline_test.png에 저장되었습니다.")
        plt.show()
        
        # 부드러운 데이터를 새 파일로 저장
        smooth_data = np.column_stack([smooth_centerline[:, 0], smooth_centerline[:, 1], 
                                     smooth_left_dist, smooth_right_dist])
        
        np.savetxt('inputs/tracks/f1_re_smooth', smooth_data, delimiter=',', 
                  header='center_x,center_y,left_distance,right_distance', 
                  comments='', fmt='%.6f')
        print("부드러운 데이터가 inputs/tracks/f1_re_smooth에 저장되었습니다.")
        
        return smooth_centerline, smooth_left_dist, smooth_right_dist
        
    except Exception as e:
        print(f"스플라인 처리 실패: {e}")
        return None, None, None

if __name__ == "__main__":
    smooth_and_resample_centerline()