import cv2
import numpy as np
import yaml
import argparse
import os
import sys
from pathlib import Path
from tkinter import Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename

def select_centerline_file(default_path=None):
    """팝업으로 센터라인 CSV 파일을 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = askopenfilename(
            title='센터라인 CSV 파일 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/maps'
        )
        if not file_path:
            sys.exit("센터라인 파일이 선택되지 않았습니다.")
        return file_path
    return default_path

def select_pgm_file(default_path=None):
    """팝업으로 PGM 맵 파일을 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = askopenfilename(
            title='PGM 맵 파일 선택',
            filetypes=[('PGM Files', '*.pgm'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/maps'
        )
        if not file_path:
            sys.exit("PGM 파일이 선택되지 않았습니다.")
        return file_path
    return default_path

def select_yaml_file(default_path=None):
    """팝업으로 YAML 맵 정보 파일을 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = askopenfilename(
            title='YAML 맵 정보 파일 선택',
            filetypes=[('YAML Files', '*.yaml *.yml'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/maps'
        )
        if not file_path:
            sys.exit("YAML 파일이 선택되지 않았습니다.")
        return file_path
    return default_path

def select_output_file(default_path=None):
    """팝업으로 출력 파일 경로를 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = asksaveasfilename(
            title='출력 CSV 파일 저장 위치 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/maps',
            defaultextension='.csv'
        )
        if not file_path:
            sys.exit("출력 파일 경로가 선택되지 않았습니다.")
        return file_path
    return default_path

def load_centerline(csv_path):
    """센터라인 CSV 파일을 로드합니다."""
    centerline = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    return centerline

def load_map(pgm_path, yaml_path):
    """PGM과 YAML 파일에서 맵 정보를 로드합니다."""
    # PGM 파일 로드
    map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    
    # YAML 파일 로드
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    
    resolution = map_info['resolution']
    origin = map_info['origin']
    
    return map_img, resolution, origin

def world_to_pixel(world_point, resolution, origin, map_height):
    """월드 좌표를 픽셀 좌표로 변환합니다."""
    # fit_centerline.py와 동일한 좌표계 사용: world = pixel * resolution - origin
    # 따라서 역변환: pixel = (world + origin) / resolution
    pixel_x = int((world_point[0] + origin[0]) / resolution)
    pixel_y = int((world_point[1] + origin[1]) / resolution)
    return pixel_x, pixel_y

def calculate_direction_vector(centerline, i):
    """센터라인에서 주어진 인덱스의 방향 벡터를 계산합니다."""
    if len(centerline) < 2:
        return np.array([1.0, 0.0])
    
    if i == 0:
        # 첫 번째 점: 다음 점 방향
        direction = np.array([centerline[1, 0] - centerline[0, 0], 
                            centerline[1, 1] - centerline[0, 1]])
    elif i == len(centerline) - 1:
        # 마지막 점: 이전 점에서의 방향
        direction = np.array([centerline[i, 0] - centerline[i-1, 0], 
                            centerline[i, 1] - centerline[i-1, 1]])
    else:
        # 중간 점: 이전과 다음 점 사이의 방향
        direction = np.array([centerline[i+1, 0] - centerline[i-1, 0], 
                            centerline[i+1, 1] - centerline[i-1, 1]])
    
    # 정규화
    norm = np.linalg.norm(direction)
    if norm > 0:
        direction = direction / norm
    else:
        direction = np.array([1.0, 0.0])
    
    return direction

def calculate_perpendicular_vectors(direction):
    """방향 벡터에 수직인 왼쪽과 오른쪽 벡터를 계산합니다."""
    left_perp = np.array([-direction[1], direction[0]])
    right_perp = np.array([direction[1], -direction[0]])
    return left_perp, right_perp

def find_boundary_simple(map_img, center_pixel, direction_vector, resolution, max_distance=3.0):
    """
    간단하고 효과적인 경계 감지 - 개선된 좌표계 및 거리 제한.
    픽셀 값이 0인 첫 번째 지점을 경계로 판단.
    """
    h, w = map_img.shape
    max_pixels = int(max_distance / resolution)
    
    # 시작점 체크
    if (center_pixel[0] < 0 or center_pixel[0] >= w or 
        center_pixel[1] < 0 or center_pixel[1] >= h):
        return 0.0
    
    # 시작점이 이미 경계(검은 픽셀)인 경우
    if map_img[center_pixel[1], center_pixel[0]] == 0:
        return 0.0
    
    # 방향을 따라 픽셀 단위로 검사 (더 세밀한 단계)
    for step in range(1, max_pixels * 2 + 1):
        distance = step * 0.5  # 0.5 픽셀 단위로 검사
        
        # 현재 검사할 픽셀 위치 (Y축 반전 없이)
        check_x = int(round(center_pixel[0] + direction_vector[0] * distance))
        check_y = int(round(center_pixel[1] + direction_vector[1] * distance))
        
        # 이미지 경계 확인
        if check_x < 0 or check_x >= w or check_y < 0 or check_y >= h:
            return distance * resolution
        
        # 검은 픽셀(값 0) 확인 - 이것이 경계!
        if map_img[check_y, check_x] == 0:
            return distance * resolution
    
    # 최대 거리에 도달 (더 보수적인 값)
    return max_distance

def extract_track_edges_revised(centerline_path, pgm_path, yaml_path, output_path):
    """개선된 트랙 경계 추출 - 센터라인 피팅 코드의 접근법 사용."""
    
    # 데이터 로드
    centerline = load_centerline(centerline_path)
    map_img, resolution, origin = load_map(pgm_path, yaml_path)
    
    print(f"센터라인 포인트 수: {len(centerline)}")
    print(f"맵 크기: {map_img.shape}")
    print(f"해상도: {resolution}m/pixel")
    print(f"Origin: {origin}")
    
    # 맵의 픽셀 값 분포 확인
    unique_values, counts = np.unique(map_img, return_counts=True)
    print(f"맵 픽셀 값 분포:")
    for val, count in zip(unique_values, counts):
        percentage = (count / map_img.size) * 100
        print(f"  값 {val}: {count}개 ({percentage:.1f}%)")
    
    # 센터라인 좌표 범위 확인
    print(f"센터라인 X 범위: {np.min(centerline[:, 0]):.3f} ~ {np.max(centerline[:, 0]):.3f}")
    print(f"센터라인 Y 범위: {np.min(centerline[:, 1]):.3f} ~ {np.max(centerline[:, 1]):.3f}")
    
    # 결과 저장용 리스트
    results = []
    left_distances = []
    right_distances = []
    
    h, w = map_img.shape
    
    print("간단한 Ray Casting 방식으로 경계 감지 중...")
    
    for i, center_point in enumerate(centerline):
        # 월드 좌표를 픽셀 좌표로 변환
        center_pixel = world_to_pixel(center_point, resolution, origin, h)
        
        # 픽셀 좌표 유효성 검사
        if (center_pixel[0] < 0 or center_pixel[0] >= w or 
            center_pixel[1] < 0 or center_pixel[1] >= h):
            print(f"Warning: 센터라인 포인트 {i}가 맵 범위를 벗어남: pixel={center_pixel}, world={center_point}")
            left_distances.append(0.0)
            right_distances.append(0.0)
            continue
        
        # 해당 픽셀이 검은색(경계)인지 확인
        pixel_value = map_img[center_pixel[1], center_pixel[0]]
        if pixel_value == 0:
            print(f"Warning: 센터라인 포인트 {i}가 경계(검은 픽셀)에 있음: pixel_value={pixel_value}")
            left_distances.append(0.0)
            right_distances.append(0.0)
            continue
        
        # 방향 벡터 계산
        direction = calculate_direction_vector(centerline, i)
        
        # 수직 방향 벡터 계산
        left_perp, right_perp = calculate_perpendicular_vectors(direction)
        
        # 왼쪽과 오른쪽 경계까지의 거리 계산 (더 보수적인 최대 거리)
        left_dist = find_boundary_simple(map_img, center_pixel, left_perp, resolution, max_distance=3.0)
        right_dist = find_boundary_simple(map_img, center_pixel, right_perp, resolution, max_distance=3.0)
        
        left_distances.append(left_dist)
        right_distances.append(right_dist)
        
        # 결과에 추가
        results.append([center_point[0], center_point[1], left_dist, right_dist])
        
        if i % 50 == 0:
            print(f"진행률: {i}/{len(centerline)} 포인트")
    
    # 결과를 numpy 배열로 변환
    results = np.array(results)
    
    # CSV 파일로 저장
    header = "center_x,center_y,left_distance,right_distance"
    np.savetxt(output_path, results, delimiter=',', header=header, comments='')
    
    # 통계 출력
    left_distances = np.array(left_distances)
    right_distances = np.array(right_distances)
    
    print(f"\n결과 통계:")
    print(f"평균 왼쪽 거리: {np.mean(left_distances):.3f}m")
    print(f"평균 오른쪽 거리: {np.mean(right_distances):.3f}m")
    print(f"왼쪽 거리 0인 포인트: {np.sum(left_distances == 0)}/{len(left_distances)}")
    print(f"오른쪽 거리 0인 포인트: {np.sum(right_distances == 0)}/{len(right_distances)}")
    print(f"왼쪽 거리 최대값인 포인트: {np.sum(left_distances >= 7.5)}/{len(left_distances)}")
    print(f"오른쪽 거리 최대값인 포인트: {np.sum(right_distances >= 7.5)}/{len(right_distances)}")
    print(f"결과 저장 완료: {output_path}")
    
    return results

def visualize_track_edges_revised(results, save_path=None, show_plot=False):
    """트랙 경계점들을 시각화합니다."""
    import matplotlib.pyplot as plt
    plt.ioff()  # Turn off interactive mode to prevent blocking
    
    results = np.array(results)
    
    # 배열 차원 검증
    if results.ndim == 1 or results.size == 0:
        print("Error: Results array is empty or 1-dimensional. Cannot visualize.")
        return
    
    if results.shape[1] < 4:
        print(f"Error: Results array has {results.shape[1]} columns, expected at least 4 (x, y, left_dist, right_dist).")
        return
    
    # 센터라인 그리기
    plt.figure(figsize=(12, 8))
    plt.plot(results[:, 0], results[:, 1], 'b-', linewidth=2, label='Centerline')
    
    # 경계점 계산 및 그리기
    step_size = 1  # 시각화를 위해 일부만 표시
    left_boundary_points = []
    right_boundary_points = []
    valid_left_count = 0
    valid_right_count = 0
    
    for i in range(0, len(results), step_size):
        center_x, center_y = results[i, 0], results[i, 1]
        left_dist, right_dist = results[i, 2], results[i, 3]
        
        # 방향 벡터 계산
        if i == 0:
            if len(results) > 1:
                direction = np.array([results[1, 0] - results[0, 0], results[1, 1] - results[0, 1]])
            else:
                direction = np.array([1.0, 0.0])
        elif i == len(results) - 1:
            direction = np.array([results[i, 0] - results[i-1, 0], results[i, 1] - results[i-1, 1]])
        else:
            direction = np.array([results[i+1, 0] - results[i-1, 0], results[i+1, 1] - results[i-1, 1]])
        
        # 정규화
        norm = np.linalg.norm(direction)
        if norm > 0:
            direction = direction / norm
            
            # 수직 벡터 계산
            left_perp = np.array([-direction[1], direction[0]])
            right_perp = np.array([direction[1], -direction[0]])
            
            # 경계점 계산 및 표시 (유효한 거리만)
            if 0.1 < left_dist < 7.5:
                left_point = np.array([center_x, center_y]) + left_perp * left_dist
                left_boundary_points.append(left_point)
                plt.plot([center_x, left_point[0]], [center_y, left_point[1]], 'r-', alpha=0.6, linewidth=1)
                valid_left_count += 1
            
            if 0.1 < right_dist < 7.5:
                right_point = np.array([center_x, center_y]) + right_perp * right_dist
                right_boundary_points.append(right_point)
                plt.plot([center_x, right_point[0]], [center_y, right_point[1]], 'g-', alpha=0.6, linewidth=1)
                valid_right_count += 1
    
    # 경계점들을 연결선으로 표시
    if left_boundary_points:
        left_boundary_points = np.array(left_boundary_points)
        plt.plot(left_boundary_points[:, 0], left_boundary_points[:, 1], 'r.', alpha=0.8, markersize=2, label='Left Boundary')
    
    if right_boundary_points:
        right_boundary_points = np.array(right_boundary_points)
        plt.plot(right_boundary_points[:, 0], right_boundary_points[:, 1], 'g.', alpha=0.8, markersize=2, label='Right Boundary')
    
    print(f"시각화된 경계선 수: 왼쪽 {valid_left_count}, 오른쪽 {valid_right_count} (전체 {len(range(0, len(results), step_size))}개 중)")
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('F1tenth Track Centerline and Boundaries (Revised Method)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"시각화 저장 완료: {save_path}")
    
    if show_plot:
        plt.show()
    else:
        plt.close()  # Close the figure to free memory

def main():
    parser = argparse.ArgumentParser(description='개선된 트랙 경계 추출기')
    parser.add_argument('--centerline', type=str, help='센터라인 CSV 파일 경로')
    parser.add_argument('--pgm', type=str, help='PGM 맵 파일 경로')
    parser.add_argument('--yaml', type=str, help='YAML 맵 정보 파일 경로')
    parser.add_argument('--output', type=str, help='출력 CSV 파일 경로')
    parser.add_argument('--save_plot', type=str, help='시각화 이미지 저장 경로')
    parser.add_argument('--show_plot', action='store_true', help='시각화 창 표시')
    parser.add_argument('--use_popup', action='store_true', help='팝업 다이얼로그로 파일 선택')
    
    args = parser.parse_args()
    
    # 파일 경로 설정 - 기본적으로 팝업 사용
    if any([args.centerline, args.pgm, args.yaml, args.output]):
        # 명령행 인수가 제공된 경우 그것을 사용
        base_dir = Path(__file__).parent.parent
        centerline_dir = base_dir / "centerline"
        centerline_with_boundary_dir = base_dir / "centerline_with_boundary"
        maps_dir = base_dir / "maps"
        
        # 기본적으로 centerline 폴더에서 센터라인을 찾고, centerline_with_boundary에 저장
        centerline_path = args.centerline or str(centerline_dir / "f1tenth_korea_3rd_reexported_centerline.csv")
        pgm_path = args.pgm or str(maps_dir / "f1tenth_korea_3rd_reexported.pgm")
        yaml_path = args.yaml or str(maps_dir / "f1tenth_korea_3rd_reexported.yaml")
        output_path = args.output or str(centerline_with_boundary_dir / "f1tenth_korea_3rd_reexported_with_boundary.csv")
    else:
        # 기본적으로 팝업 다이얼로그 사용
        print("팝업 다이얼로그로 파일을 선택하세요...")
        
        # Hide the main tkinter window
        root = Tk()
        root.withdraw()
        
        centerline_path = select_centerline_file(None)
        pgm_path = select_pgm_file(None)
        yaml_path = select_yaml_file(None)
        
        # 기본 저장 위치를 centerline_with_boundary 폴더로 설정
        output_path = asksaveasfilename(
            title='boundary CSV 파일 저장 위치 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/path_generate/centerline_with_boundary',
            defaultextension='.csv'
        )
        if not output_path:
            sys.exit("출력 파일 경로가 선택되지 않았습니다.")
        
        root.destroy()
    
    print("선택된 파일들:")
    print(f"  센터라인: {centerline_path}")
    print(f"  PGM: {pgm_path}")
    print(f"  YAML: {yaml_path}")
    print(f"  출력: {output_path}")
    print(f"  방법: simplified boundary detection")
    print()
    
    # 트랙 경계 추출
    results = extract_track_edges_revised(centerline_path, pgm_path, yaml_path, output_path)
    
    # 자동으로 시각화 PNG 파일 생성
    if args.save_plot:
        plot_path = args.save_plot
    else:
        # centerline_with_boundary 폴더에 PNG 파일 생성
        output_dir = os.path.dirname(output_path)
        os.makedirs(output_dir, exist_ok=True)  # 폴더가 없으면 생성
        output_name = os.path.splitext(os.path.basename(output_path))[0]
        plot_path = os.path.join(output_dir, f"{output_name}_visualization.png")
    
    # 시각화 수행 (기본적으로 활성화)
    visualize_track_edges_revised(results, plot_path, args.show_plot)

if __name__ == "__main__":
    main()