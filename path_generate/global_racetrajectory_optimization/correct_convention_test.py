#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tumftm 컨벤션에 맞는 올바른 좌우 거리 매핑 테스트
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import os

def test_convention_mapping():
    """tumftm 컨벤션 테스트"""
    
    # F1 RE 데이터 로드
    data = np.loadtxt("inputs/tracks/f1_re", comments='#', delimiter=',', skiprows=1)
    print(f"원본 데이터 형식: center_x, center_y, left_distance, right_distance")
    print(f"데이터 크기: {data.shape}")
    
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
    
    # 스플라인 기반 정확한 법선 벡터 계산
    try:
        tck, u = interpolate.splprep([centerline[:, 0], centerline[:, 1]], s=0, per=1, k=3)
        u_points = np.linspace(0, 1, len(centerline))
        dx_du, dy_du = interpolate.splev(u_points, tck, der=1)
        
        # 접선 벡터 정규화
        tangent_lengths = np.sqrt(dx_du**2 + dy_du**2)
        tangent_vectors = np.column_stack([dx_du / tangent_lengths, dy_du / tangent_lengths])
        
        # 법선 벡터 (오른쪽 방향으로, tumftm 컨벤션에 따라)
        normal_vectors = np.column_stack([tangent_vectors[:, 1], -tangent_vectors[:, 0]])
        
        print("스플라인 기반 법선 벡터 계산 완료")
        
    except Exception as e:
        print(f"스플라인 실패: {e}, 간단한 방법 사용")
        # 백업 방법
        normal_vectors = np.zeros_like(centerline)
        for i in range(len(centerline)):
            if i == 0:
                tangent = centerline[1] - centerline[-1]
            elif i == len(centerline) - 1:
                tangent = centerline[0] - centerline[i-1]
            else:
                tangent = centerline[i+1] - centerline[i-1]
            
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
                # 오른쪽을 가리키는 법선 벡터
                normal_vectors[i] = [tangent[1], -tangent[0]]
    
    # 여러 가지 매핑 방법 테스트
    mappings = {
        "방법1 (현재)": {
            "w_tr_right": right_dist,
            "w_tr_left": left_dist
        },
        "방법2 (반대)": {
            "w_tr_right": left_dist, 
            "w_tr_left": right_dist
        }
    }
    
    plt.figure(figsize=(20, 10))
    
    for idx, (method_name, mapping) in enumerate(mappings.items()):
        ax = plt.subplot(1, 2, idx+1)
        
        w_tr_right = mapping["w_tr_right"]
        w_tr_left = mapping["w_tr_left"]
        
        # tumftm 컨벤션에 따른 경계 계산
        right_boundary = centerline + normal_vectors * w_tr_right.reshape(-1, 1)
        left_boundary = centerline - normal_vectors * w_tr_left.reshape(-1, 1)
        
        # 플롯 (경계는 점으로만 표시)
        ax.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=3, label='Centerline')
        ax.scatter(right_boundary[:, 0], right_boundary[:, 1], c='red', s=2, alpha=0.7, label='Right Boundary')
        ax.scatter(left_boundary[:, 0], left_boundary[:, 1], c='green', s=2, alpha=0.7, label='Left Boundary')
        
        # centerline에서 boundary points로 quiver 표시 (모든 row)
        for i in range(len(centerline)):
            center_x, center_y = centerline[i]
            right_x, right_y = right_boundary[i]
            left_x, left_y = left_boundary[i]
            
            # centerline에서 right boundary로
            ax.quiver(center_x, center_y, right_x - center_x, right_y - center_y, 
                     color='red', alpha=0.8, scale=1, scale_units='xy', angles='xy', width=0.003)
            
            # centerline에서 left boundary로  
            ax.quiver(center_x, center_y, left_x - center_x, left_y - center_y, 
                     color='green', alpha=0.8, scale=1, scale_units='xy', angles='xy', width=0.003)
        
        # 시작점 표시
        ax.plot(centerline[0, 0], centerline[0, 1], 'ko', markersize=8, label='Start')
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]') 
        ax.set_title(f'{method_name}\nTUMFTM 컨벤션 테스트')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    plt.tight_layout()
    os.makedirs("outputs", exist_ok=True)
    plt.savefig("outputs/convention_test.png", dpi=300, bbox_inches='tight')
    print("컨벤션 테스트 결과가 outputs/convention_test.png에 저장되었습니다.")
    plt.show()
    
    # CSV 원본과 비교
    print(f"\n=== CSV 원본 데이터 샘플 ===")
    print("center_x, center_y, left_distance, right_distance")
    for i in range(min(5, len(data))):
        print(f"{data[i, 0]:8.3f}, {data[i, 1]:8.3f}, {data[i, 2]:12.3f}, {data[i, 3]:13.3f}")
    
    return centerline, normal_vectors, left_dist, right_dist

if __name__ == "__main__":
    test_convention_mapping()