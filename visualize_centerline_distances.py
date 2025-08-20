#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_centerline_with_distances():
    """
    centerline_with_distances.csv 파일을 시각화하는 함수
    """
    
    # CSV 파일 경로
    csv_path = "/home/f1/f1tenth_ws/maps/track_edges_revised.csv"
    
    # 파일 존재 확인
    if not os.path.exists(csv_path):
        print(f"파일을 찾을 수 없습니다: {csv_path}")
        print("먼저 path planner를 실행해서 track_edges_revised.csv 파일을 생성하세요.")
        return
    
    # CSV 파일 읽기
    try:
        df = pd.read_csv(csv_path)
        print(f"데이터 로드 완료: {len(df)}개 포인트")
        print(f"컬럼: {list(df.columns)}")
    except Exception as e:
        print(f"CSV 파일 읽기 오류: {e}")
        return
    
    # 데이터 확인
    if len(df) == 0:
        print("데이터가 비어있습니다.")
        return
    
    # 필요한 컬럼 확인
    required_columns = ['center_x', 'center_y', 'left_distance', 'right_distance']
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        print(f"필수 컬럼이 없습니다: {missing_columns}")
        return
    
    # 데이터 통계 출력
    print(f"\n=== 데이터 통계 ===")
    print(f"X 범위: {df['center_x'].min():.2f} ~ {df['center_x'].max():.2f}")
    print(f"Y 범위: {df['center_y'].min():.2f} ~ {df['center_y'].max():.2f}")
    print(f"왼쪽 거리 범위: {df['left_distance'].min():.2f} ~ {df['left_distance'].max():.2f}")
    print(f"오른쪽 거리 범위: {df['right_distance'].min():.2f} ~ {df['right_distance'].max():.2f}")
    print(f"평균 왼쪽 거리: {df['left_distance'].mean():.2f}m")
    print(f"평균 오른쪽 거리: {df['right_distance'].mean():.2f}m")
    
    # 시각화
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Centerline with Track Boundary Distances', fontsize=16)
    
    # 1. 중심선과 좌우 경계 시각화
    ax1 = axes[0, 0]
    
    # 중심선 그리기
    ax1.plot(df['center_x'], df['center_y'], 'b-', linewidth=2, label='Centerline', alpha=0.8)
    ax1.scatter(df['center_x'], df['center_y'], c='blue', s=10, alpha=0.6)
    
    # 왼쪽과 오른쪽 경계 추정 포인트 계산 및 그리기
    for i in range(0, len(df), max(1, len(df)//170)):  # 샘플링해서 표시
        x, y = df.iloc[i]['center_x'], df.iloc[i]['center_y']
        left_dist = df.iloc[i]['left_distance']
        right_dist = df.iloc[i]['right_distance']
        
        # 방향 벡터 계산 (간단하게 다음 점으로의 방향)
        if i < len(df) - 1:
            dx = df.iloc[i+1]['center_x'] - x
            dy = df.iloc[i+1]['center_y'] - y
        else:
            dx = x - df.iloc[i-1]['center_x']
            dy = y - df.iloc[i-1]['center_y']
        
        # 정규화
        norm = np.sqrt(dx*dx + dy*dy)
        if norm > 0:
            dx, dy = dx/norm, dy/norm
        
        # 수직 벡터 계산
        left_x = x + (-dy) * left_dist
        left_y = y + dx * left_dist
        right_x = x + dy * right_dist
        right_y = y + (-dx) * right_dist
        
        # 경계 포인트 표시
        ax1.plot([x, left_x], [y, left_y], 'r-', alpha=0.3, linewidth=0.5)
        ax1.plot([x, right_x], [y, right_y], 'g-', alpha=0.3, linewidth=0.5)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Track Layout with Boundary Distances')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    # 2. 거리 히트맵 (왼쪽 거리)
    ax2 = axes[0, 1]
    scatter = ax2.scatter(df['center_x'], df['center_y'], c=df['left_distance'], 
                         cmap='viridis', s=20, alpha=0.8)
    plt.colorbar(scatter, ax=ax2, label='Left Distance (m)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Left Boundary Distance Heatmap')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # 3. 거리 히트맵 (오른쪽 거리)
    ax3 = axes[1, 0]
    scatter = ax3.scatter(df['center_x'], df['center_y'], c=df['right_distance'], 
                         cmap='plasma', s=20, alpha=0.8)
    plt.colorbar(scatter, ax=ax3, label='Right Distance (m)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Right Boundary Distance Heatmap')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # 4. 거리 변화 그래프
    ax4 = axes[1, 1]
    point_indices = range(len(df))
    ax4.plot(point_indices, df['left_distance'], 'r-', label='Left Distance', linewidth=1.5, alpha=0.8)
    ax4.plot(point_indices, df['right_distance'], 'g-', label='Right Distance', linewidth=1.5, alpha=0.8)
    ax4.plot(point_indices, df['left_distance'] + df['right_distance'], 'b--', 
             label='Total Track Width', linewidth=1, alpha=0.6)
    ax4.set_xlabel('Centerline Point Index')
    ax4.set_ylabel('Distance (m)')
    ax4.set_title('Distance Variation Along Centerline')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    plt.tight_layout()
    
    # 이미지 저장
    output_path = "/home/f1/f1tenth_ws/centerline_distances_visualization.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n시각화 이미지가 저장되었습니다: {output_path}")
    
    # 이미지 표시
    plt.show()

def create_simple_track_visualization():
    """
    간단한 트랙 레이아웃 시각화
    """
    csv_path = "/home/f1/f1tenth_ws/maps/track_edges_revised.csv"
    
    if not os.path.exists(csv_path):
        print(f"파일을 찾을 수 없습니다: {csv_path}")
        return
    
    df = pd.read_csv(csv_path)
    
    plt.figure(figsize=(12, 8))
    
    # 중심선 그리기
    plt.plot(df['center_x'], df['center_y'], 'b-', linewidth=3, label='Centerline')
    
    # 시작점 표시
    plt.scatter(df.iloc[0]['center_x'], df.iloc[0]['center_y'], c='red', s=100, 
                marker='o', label='Start Point', zorder=5)
    
    # 몇 개 포인트에서 트랙 폭 표시
    for i in range(0, len(df), len(df)//20):
        x, y = df.iloc[i]['center_x'], df.iloc[i]['center_y']
        left_dist = df.iloc[i]['left_distance']
        right_dist = df.iloc[i]['right_distance']
        
        # 방향 계산
        if i < len(df) - 1:
            dx = df.iloc[i+1]['center_x'] - x
            dy = df.iloc[i+1]['center_y'] - y
        else:
            dx = x - df.iloc[i-1]['center_x']
            dy = y - df.iloc[i-1]['center_y']
        
        norm = np.sqrt(dx*dx + dy*dy)
        if norm > 0:
            dx, dy = dx/norm, dy/norm
        
        # 트랙 경계 표시
        left_x = x + (-dy) * left_dist
        left_y = y + dx * left_dist
        right_x = x + dy * right_dist
        right_y = y + (-dx) * right_dist
        
        plt.plot([left_x, right_x], [left_y, right_y], 'k-', alpha=0.4, linewidth=1)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('F1TENTH Track Layout')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    # 저장
    output_path = "/home/f1/f1tenth_ws/simple_track_layout.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"간단한 트랙 레이아웃이 저장되었습니다: {output_path}")
    
    plt.show()

if __name__ == "__main__":
    print("=== Centerline with Distances Visualization ===")
    
    # 상세 시각화
    visualize_centerline_with_distances()
    
    # 간단한 트랙 레이아웃
    print("\n=== Simple Track Layout ===")
    create_simple_track_visualization()
