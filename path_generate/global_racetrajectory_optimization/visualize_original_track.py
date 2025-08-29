#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원본 트랙 CSV 파일 시각화
"""

import numpy as np
import matplotlib.pyplot as plt
import os

def visualize_track(track_file):
    """트랙 파일 시각화"""
    
    # CSV 파일 읽기
    data = np.loadtxt(track_file, comments='#', delimiter=',')
    print(f"트랙 데이터: {data.shape[0]}개 포인트, {data.shape[1]}개 열")
    print(f"데이터 범위:")
    print(f"  X: {np.min(data[:, 0]):.2f} ~ {np.max(data[:, 0]):.2f}")
    print(f"  Y: {np.min(data[:, 1]):.2f} ~ {np.max(data[:, 1]):.2f}")
    print(f"  Right width: {np.min(data[:, 2]):.2f} ~ {np.max(data[:, 2]):.2f}")
    print(f"  Left width: {np.min(data[:, 3]):.2f} ~ {np.max(data[:, 3]):.2f}")
    
    # 트랙 centerline
    centerline = data[:, :2]
    w_right = data[:, 2]
    w_left = data[:, 3]
    
    # 법선 벡터 간단 계산
    n_points = centerline.shape[0]
    tangent_vectors = np.zeros_like(centerline)
    
    for i in range(n_points):
        if i == 0:
            tangent_vectors[i] = centerline[1] - centerline[-1]  # 닫힌 루프
        elif i == n_points - 1:
            tangent_vectors[i] = centerline[0] - centerline[i-1]  # 닫힌 루프  
        else:
            tangent_vectors[i] = centerline[i+1] - centerline[i-1]
    
    # 접선 벡터 정규화
    tangent_lengths = np.linalg.norm(tangent_vectors, axis=1)
    tangent_vectors = tangent_vectors / tangent_lengths.reshape(-1, 1)
    
    # 법선 벡터 계산 (접선 벡터를 90도 회전)
    normal_vectors = np.zeros_like(tangent_vectors)
    normal_vectors[:, 0] = -tangent_vectors[:, 1]  # -sin
    normal_vectors[:, 1] = tangent_vectors[:, 0]   # cos
    
    # 트랙 경계 계산
    right_boundary = centerline + normal_vectors * w_right.reshape(-1, 1)
    left_boundary = centerline - normal_vectors * w_left.reshape(-1, 1)
    
    # 시각화
    plt.figure(figsize=(15, 10))
    
    # 메인 플롯
    plt.subplot(2, 2, 1)
    plt.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Centerline', marker='o', markersize=1)
    plt.plot(right_boundary[:, 0], right_boundary[:, 1], 'r--', linewidth=1, label='Right boundary')
    plt.plot(left_boundary[:, 0], left_boundary[:, 1], 'g--', linewidth=1, label='Left boundary')
    
    # 시작점 표시
    plt.plot(centerline[0, 0], centerline[0, 1], 'ro', markersize=8, label='Start point')
    plt.plot(centerline[-1, 0], centerline[-1, 1], 'bo', markersize=8, label='End point')
    
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Original Track Layout')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # 트랙 폭 분포
    plt.subplot(2, 2, 2)
    plt.plot(w_right, 'r-', label='Right width')
    plt.plot(w_left, 'g-', label='Left width')
    plt.plot(w_right + w_left, 'b-', label='Total width')
    plt.xlabel('Point index')
    plt.ylabel('Width [m]')
    plt.title('Track Width Distribution')
    plt.legend()
    plt.grid(True)
    
    # X, Y 좌표 분포
    plt.subplot(2, 2, 3)
    plt.plot(centerline[:, 0], 'b-', label='X coordinate')
    plt.xlabel('Point index')
    plt.ylabel('X [m]')
    plt.title('X Coordinate')
    plt.grid(True)
    
    plt.subplot(2, 2, 4)
    plt.plot(centerline[:, 1], 'r-', label='Y coordinate')
    plt.xlabel('Point index')
    plt.ylabel('Y [m]')
    plt.title('Y Coordinate')
    plt.grid(True)
    
    plt.tight_layout()
    
    # 저장
    os.makedirs("outputs", exist_ok=True)
    plt.savefig("outputs/original_track_analysis.png", dpi=300, bbox_inches='tight')
    print("원본 트랙 분석 결과가 outputs/original_track_analysis.png에 저장되었습니다.")
    
    plt.show()
    
    # 닫힌 루프 확인
    start_point = centerline[0]
    end_point = centerline[-1]
    distance = np.linalg.norm(start_point - end_point)
    print(f"\n닫힌 루프 확인:")
    print(f"  시작점: ({start_point[0]:.3f}, {start_point[1]:.3f})")
    print(f"  끝점: ({end_point[0]:.3f}, {end_point[1]:.3f})")
    print(f"  거리: {distance:.3f}m")
    
    if distance < 0.5:
        print("  → 닫힌 루프입니다!")
    else:
        print("  → 열린 루프입니다.")
    
    # 연속성 확인
    distances_between_points = []
    for i in range(1, len(centerline)):
        dist = np.linalg.norm(centerline[i] - centerline[i-1])
        distances_between_points.append(dist)
    
    distances_between_points = np.array(distances_between_points)
    print(f"\n포인트 간 거리:")
    print(f"  최소: {np.min(distances_between_points):.3f}m")
    print(f"  최대: {np.max(distances_between_points):.3f}m")
    print(f"  평균: {np.mean(distances_between_points):.3f}m")
    print(f"  표준편차: {np.std(distances_between_points):.3f}m")

if __name__ == "__main__":
    track_file = "inputs/tracks/test_track_reorder.csv"
    print(f"트랙 파일 분석: {track_file}")
    visualize_track(track_file)