import time
import sys
import os
import numpy as np
import yaml
import math
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d

from sklearn.cluster import DBSCAN
from skimage.feature import canny
from skimage.morphology import skeletonize
from graph_util import FindCycle
from pixel2point import pixels2points, points2pixels
from matplotlib import pyplot as plt

def imageFromFile(pgm_path):
    """PGM 파일에서 이미지 로드"""
    with open(pgm_path, 'rb') as pgmf:
        header = []
        while len(header) < 4:
            line = pgmf.readline()
            words = line.split()
            if len(words) > 0 and words[0] != b'#':
                header.extend(words)
        
        if len(header) != 4 or header[0] != b'P5' or not header[1].isdigit() or not header[2].isdigit() or not header[3].isdigit():
            raise ValueError("Error Reading PGM File")
        
        width = int(header[1])
        height = int(header[2])
        depth = int(header[3])

        image = []
        while len(image) < height:
            row = []
            while len(row) < width:
                word = ord(pgmf.read(1))
                row.append(word)
            image.append(row)

        return np.array(image), depth, pgm_path

def infoFromFile(yaml_path):
    """YAML 파일에서 맵 정보 로드"""
    with open(yaml_path, 'r') as yamlf:
        data = yaml.safe_load(yamlf)
        resolution = None
        origin = None
        try:
            resolution = data['resolution']
            origin = data['origin']
        except KeyError:
            raise ValueError("Error Reading YAML File")

        return resolution, origin

def pixelsInCircle(radius):
    """원 안의 픽셀 개수 계산"""
    num = 0
    for i in range(100000):
        num += math.floor(radius**2. / (4.*i + 1.)) - math.floor(radius**2. / (4.*i + 3.))
    num *= 4
    num += 1
    return num

def denoise_with_params(image, depth, params):
    """파라미터화된 디노이즈 함수"""
    radius = params['denoise_radius']
    min_samples_ratio = params['denoise_min_samples_ratio']
    min_cluster_ratio = params['denoise_min_cluster_ratio']
    min_cluster_min = params['denoise_min_cluster_min']
    
    # DBSCAN 설정
    min_samples = max(1, int(pixelsInCircle(radius) // min_samples_ratio))
    db_scan = DBSCAN(eps=radius, min_samples=min_samples)

    points = pixels2points(image, False)
    clusters = db_scan.fit(points).labels_
    cluster_sizes = np.bincount(clusters + 1)
    
    if len(cluster_sizes) < 2:
        raise RuntimeError('No clusters found')
    
    cluster_sizes = cluster_sizes[1:]  # 노이즈 제거
    
    # 최소 클러스터 크기 계산
    min_cluster_size = max(min_cluster_min, int(len(points) * min_cluster_ratio))
    valid_clusters = [i for i, size in enumerate(cluster_sizes) if size >= min_cluster_size]
    
    points = [x for i, x in enumerate(points) if clusters[i] in valid_clusters]

    height, width = image.shape
    image = points2pixels(points, width, height, depth, False)

    negative_points = pixels2points(image, True)

    # 두 번째 DBSCAN
    db_scan = DBSCAN(eps=1.5, min_samples=1)
    clusters = db_scan.fit(negative_points).labels_
    cluster_sizes = np.bincount(clusters + 1)
    
    if len(cluster_sizes) < 3:
        raise RuntimeError('Not enough clusters found')
    
    cluster_sizes = cluster_sizes[1:]
    main_clusters = np.argsort(cluster_sizes)[-3:]

    negative_points = [x for i, x in enumerate(negative_points) if clusters[i] in main_clusters]

    return points2pixels(negative_points, width, height, depth, True)

def prune(image, plot_mode):
    """그래프 기반 중심선 추출"""
    height, width = image.shape
    g = FindCycle()
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if pixel > 0:
                for j in range(y-1, y+2):
                    for i in range(x-1, x+2):
                        if i >= 0 and i < width and j >= 0 and j < height and image[j][i] > 0:
                            g.addEdge((x, y), (i, j))
    
    g.findCycle()

    if plot_mode < 2:
        return image, g.cycle

    image = np.zeros((height, width))
    for i, (x, y) in enumerate(g.cycle):
        image[y, x] = i / len(g.cycle) * (155) + 100
    return image, g.cycle

def subsample(image, cycle, period, depth):
    """서브샘플링"""
    subsampled_cycle = []
    for i, point in enumerate(cycle):
        if i % period == 0:
            subsampled_cycle.append(point)
    
    # 닫힌 루프 보장
    if len(subsampled_cycle) > 1 and subsampled_cycle[-1] != cycle[-1]:
        subsampled_cycle.append(cycle[-1])
    
    height, width = image.shape
    image = points2pixels(subsampled_cycle, width, height, depth, False)
    
    return image, subsampled_cycle

def calculate_curvature(points):
    """각 포인트에서의 곡률을 계산"""
    points = np.array(points)
    n = len(points)
    curvatures = np.zeros(n)
    
    for i in range(n):
        p1 = points[(i-1) % n]
        p2 = points[i]
        p3 = points[(i+1) % n]
        
        v1 = p2 - p1
        v2 = p3 - p2
        
        if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_angle = np.clip(cos_angle, -1, 1)
            angle_change = np.arccos(cos_angle)
            curvatures[i] = angle_change
    
    return curvatures

def smooth_high_curvature_areas(points, params):
    """곡률이 높은 부분만 선택적으로 스무딩"""
    curvature_threshold = params['curvature_threshold']
    window_size = params['smoothing_window_size']
    iterations = params['smoothing_iterations']
    blend_factor = params['smoothing_blend_factor']
    
    points = np.array(points)
    curvatures = calculate_curvature(points)
    
    print(f"곡률 분석: 평균={curvatures.mean():.3f}, 최대={curvatures.max():.3f}")
    print(f"곡률 임계값 {curvature_threshold} 이상인 포인트: {np.sum(curvatures > curvature_threshold)}개")
    
    high_curvature_mask = curvatures > curvature_threshold
    smooth_mask = np.zeros(len(points), dtype=bool)
    
    for i, is_high_curv in enumerate(high_curvature_mask):
        if is_high_curv:
            for j in range(-window_size//2, window_size//2 + 1):
                idx = (i + j) % len(points)
                smooth_mask[idx] = True
    
    print(f"Smoothing applied to {np.sum(smooth_mask)} points ({np.sum(smooth_mask)/len(points)*100:.1f}% of total)")
    
    smoothed_points = points.copy()
    
    for iteration in range(iterations):
        new_points = smoothed_points.copy()
        
        for i, should_smooth in enumerate(smooth_mask):
            if should_smooth:
                neighbor_indices = []
                for j in range(-window_size//2, window_size//2 + 1):
                    neighbor_idx = (i + j) % len(points)
                    neighbor_indices.append(neighbor_idx)
                
                weights = []
                for neighbor_idx in neighbor_indices:
                    distance = min(abs(neighbor_idx - i), len(points) - abs(neighbor_idx - i))
                    weight = np.exp(-0.5 * (distance / (window_size // 4))**2)
                    weights.append(weight)
                
                weights = np.array(weights)
                weights /= weights.sum()
                
                smooth_x = np.sum([smoothed_points[idx][0] * w for idx, w in zip(neighbor_indices, weights)])
                smooth_y = np.sum([smoothed_points[idx][1] * w for idx, w in zip(neighbor_indices, weights)])
                
                new_points[i][0] = (1 - blend_factor) * smoothed_points[i][0] + blend_factor * smooth_x
                new_points[i][1] = (1 - blend_factor) * smoothed_points[i][1] + blend_factor * smooth_y
        
        smoothed_points = new_points
        print(f"Smoothing iteration {iteration + 1}/{iterations} completed")
    
    return smoothed_points

def smooth_centerline_with_params(points, params):
    """파라미터화된 centerline 스무딩"""
    smoothing_method = params['smoothing_method']
    sigma = params['gaussian_sigma']
    spline_points_ratio = params['spline_points_ratio']
    
    if len(points) < 3:
        print(f"포인트가 너무 적습니다: {len(points)}개")
        return np.array(points)
    
    points = np.array(points)
    print(f"원본 포인트 수: {len(points)}")
    
    if smoothing_method == 'curvature':
        print("곡률 기반 선택적 스무딩 적용 중...")
        return smooth_high_curvature_areas(points, params)
    
    print(f"포인트 범위: x({points[:, 0].min():.2f}~{points[:, 0].max():.2f}), y({points[:, 1].min():.2f}~{points[:, 1].max():.2f})")
    
    if smoothing_method == 'gaussian' or smoothing_method == 'both':
        print("가우시안 스무딩 적용 중...")
        extended_points = np.vstack([points[-3:], points, points[:3]])
        smooth_x = gaussian_filter1d(extended_points[:, 0], sigma=sigma)
        smooth_y = gaussian_filter1d(extended_points[:, 1], sigma=sigma)
        points = np.column_stack([smooth_x[3:-3], smooth_y[3:-3]])
        print(f"가우시안 스무딩 후 포인트 수: {len(points)}")
    
    if smoothing_method == 'spline' or smoothing_method == 'both':
        print("B-spline 보간 적용 중...")
        try:
            spline_points = int(len(points) * spline_points_ratio)
            print(f"목표 spline 포인트 수: {spline_points}")
            
            unique_points = []
            for i, point in enumerate(points):
                if i == 0 or not np.allclose(point, points[i-1], atol=1e-6):
                    unique_points.append(point)
            
            points = np.array(unique_points)
            print(f"중복 제거 후 포인트 수: {len(points)}")
            
            if len(points) < 4:
                print("spline 보간을 위한 포인트가 부족합니다.")
                return points
            
            tck, u = interpolate.splprep([points[:, 0], points[:, 1]], 
                                       s=len(points), k=3, per=True)
            
            u_new = np.linspace(0, 1, spline_points)
            spline_points_arr = interpolate.splev(u_new, tck)
            
            points = np.column_stack([spline_points_arr[0], spline_points_arr[1]])
            print(f"spline 보간 후 포인트 수: {len(points)}")
            
        except Exception as e:
            print(f"spline 보간 실패: {e}")
            if smoothing_method != 'both':
                extended_points = np.vstack([points[-3:], points, points[:3]])
                smooth_x = gaussian_filter1d(extended_points[:, 0], sigma=sigma)
                smooth_y = gaussian_filter1d(extended_points[:, 1], sigma=sigma)
                points = np.column_stack([smooth_x[3:-3], smooth_y[3:-3]])
    
    return points

def calculate_track_widths(centerline_points, pgm_image, resolution, origin):
    """개선된 트랙 경계 추출 - 센터라인 피팅 코드의 접근법 사용."""
    print("트랙 폭 계산 중... (개선된 경계 감지)")
    
    if len(centerline_points) < 3:
        print("포인트가 너무 적어 트랙 폭을 계산할 수 없습니다.")
        return None, None
    
    print(f"센터라인 포인트 수: {len(centerline_points)}")
    print(f"맵 크기: {pgm_image.shape}")
    print(f"해상도: {resolution}m/pixel")
    print(f"Origin: {origin}")
    
    # 맵의 픽셀 값 분포 확인
    unique_values, counts = np.unique(pgm_image, return_counts=True)
    print(f"맵 픽셀 값 분포:")
    for val, count in zip(unique_values, counts):
        percentage = (count / pgm_image.size) * 100
        print(f"  값 {val}: {count}개 ({percentage:.1f}%)")
    
    # 센터라인 좌표 범위 확인
    print(f"센터라인 X 범위: {np.min(centerline_points[:, 0]):.3f} ~ {np.max(centerline_points[:, 0]):.3f}")
    print(f"센터라인 Y 범위: {np.min(centerline_points[:, 1]):.3f} ~ {np.max(centerline_points[:, 1]):.3f}")
    
    left_distances = []
    right_distances = []
    h, w = pgm_image.shape
    
    print("간단한 Ray Casting 방식으로 경계 감지 중...")
    
    for i, center_point in enumerate(centerline_points):
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
        pixel_value = pgm_image[center_pixel[1], center_pixel[0]]
        if pixel_value == 0:
            print(f"Warning: 센터라인 포인트 {i}가 경계(검은 픽셀)에 있음: pixel_value={pixel_value}")
            left_distances.append(0.0)
            right_distances.append(0.0)
            continue
        
        # 방향 벡터 계산 (Raceline-Optimization 방식)
        direction = calculate_direction_vector(centerline_points, i)
        
        # 수직 방향 벡터 계산
        left_perp, right_perp = calculate_perpendicular_vectors(direction)
        
        # 왼쪽과 오른쪽 경계까지의 거리 계산
        left_dist = find_boundary_simple(pgm_image, center_pixel, left_perp, resolution)
        right_dist = find_boundary_simple(pgm_image, center_pixel, right_perp, resolution)
        
        left_distances.append(left_dist)
        right_distances.append(right_dist)
        
        if i % 50 == 0:
            print(f"진행률: {i}/{len(centerline_points)} 포인트")
    
    # 통계 출력
    left_distances = np.array(left_distances)
    right_distances = np.array(right_distances)
    
    print(f"\n결과 통계:")
    print(f"평균 왼쪽 거리: {np.mean(left_distances):.3f}m")
    print(f"평균 오른쪽 거리: {np.mean(right_distances):.3f}m")
    print(f"왼쪽 거리 0인 포인트: {np.sum(left_distances == 0)}/{len(left_distances)}")
    print(f"오른쪽 거리 0인 포인트: {np.sum(right_distances == 0)}/{len(right_distances)}")
    print(f"왼쪽 거리 최대값: {np.max(left_distances):.3f}m")
    print(f"오른쪽 거리 최대값: {np.max(right_distances):.3f}m")
    
    return left_distances, right_distances

def world_to_pixel(world_point, resolution, origin, map_height):
    """월드 좌표를 픽셀 좌표로 변환합니다."""
    # fit_centerline.py와 동일한 좌표계 사용: world = pixel * resolution - origin
    # 따라서 역변환: pixel = (world + origin) / resolution
    pixel_x = int((world_point[0] + origin[0]) / resolution)
    pixel_y = int((world_point[1] + origin[1]) / resolution)
    return pixel_x, pixel_y

def calculate_direction_vector(centerline, i):
    """Raceline-Optimization 방식: 인접점 기반 방향 벡터 계산"""
    if len(centerline) < 2:
        return np.array([1.0, 0.0])
    
    if i == 0:
        # 첫 번째 점: 현재점에서 다음점으로의 방향
        direction = np.array([centerline[1, 0] - centerline[0, 0], 
                            centerline[1, 1] - centerline[0, 1]])
    elif i == len(centerline) - 1:
        # 마지막 점: 이전점에서 현재점으로의 방향
        direction = np.array([centerline[i, 0] - centerline[i-1, 0], 
                            centerline[i, 1] - centerline[i-1, 1]])
    else:
        # 중간 점: 이전점에서 다음점으로의 방향 (Raceline-Optimization 방식)
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

def find_boundary_simple(map_img, center_pixel, direction_vector, resolution):
    """
    간단하고 효과적인 경계 감지 - 픽셀 값이 0인 첫 번째 지점을 경계로 판단.
    """
    h, w = map_img.shape
    max_pixels = max(h, w)  # 맵 전체 크기까지 검사 가능
    
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
        
        # 현재 검사할 픽셀 위치
        check_x = int(round(center_pixel[0] + direction_vector[0] * distance))
        check_y = int(round(center_pixel[1] + direction_vector[1] * distance))
        
        # 이미지 경계 확인
        if check_x < 0 or check_x >= w or check_y < 0 or check_y >= h:
            return distance * resolution
        
        # 검은 픽셀(값 0) 확인 - 이것이 경계!
        if map_img[check_y, check_x] == 0:
            return distance * resolution
    
    # 맵 끝까지 도달 (이론적으로는 발생하지 않음)
    return max_pixels * resolution


def plot(image, title, plot_mode):
    """이미지 플롯"""
    if plot_mode > 0:
        fig = plt.figure()
        fig.suptitle(title)
        plt.imshow(image, interpolation='nearest', cmap='Greys')

def plotExtra(image, title, plot_mode):
    """추가 플롯"""
    if plot_mode > 1:
        plot(image, title, plot_mode)

def process_centerline_with_params(params, progress_callback=None):
    """파라미터를 사용하여 centerline fitting 처리"""
    try:
        start_time = time.time()
        
        if progress_callback:
            progress_callback(0, "파일 로딩 중...")
        
        # 파일 로드
        image, depth, name = imageFromFile(params['pgm_path'])
        resolution, origin = infoFromFile(params['yaml_path'])
        
        print(f"Resolution: {resolution}")
        
        if progress_callback:
            progress_callback(10, "원본 이미지 로드 완료")
        
        plot_mode = params['plot_mode']
        plot(image, "Original Image", plot_mode)
        
        # 디노이징
        if progress_callback:
            progress_callback(20, "디노이징 중...")
        print("Denoising")
        image = denoise_with_params(image, depth, params)
        denoised = image
        plotExtra(image, "Denoised", plot_mode)
        
        # 스켈레톤화
        if progress_callback:
            progress_callback(40, "스켈레톤화 중...")
        print("Skeletonizing")
        skeleton_input = np.where(image > 0, 1, 0)
        skeleton_pixels_before = np.sum(skeleton_input)
        print(f"스켈레톤화 전: {skeleton_pixels_before}개 픽셀")
        
        image = skeletonize(skeleton_input)
        skeleton_pixels_after = np.sum(image)
        print(f"스켈레톤화 후: {skeleton_pixels_after}개 픽셀 ({skeleton_pixels_after/skeleton_pixels_before*100:.1f}% 보존)")
        
        skeleton_overlay = np.where(image == 1, 0, denoised)
        image = image * depth
        
        plotExtra(image, "Skeletonized", plot_mode)
        plotExtra(skeleton_overlay, "Skeletonized Overlayed on Denoised", plot_mode)
        
        # 프루닝
        if progress_callback:
            progress_callback(60, "중심선 추출 중...")
        print("Pruning")
        image, track_cycle = prune(image, plot_mode)
        overlayed = np.where(image > 0, 0, denoised)
        
        plotExtra(image, "Pruned", plot_mode)
        plotExtra(overlayed, "Pruned Overlayed on Denoised", plot_mode)
        
        # 서브샘플링
        if progress_callback:
            progress_callback(70, "서브샘플링 중...")
        print("Subsampling")
        subsample_period = params['subsample_period']
        image, track_cycle = subsample(image, track_cycle, subsample_period, depth)
        
        plot(image, "Output", plot_mode)
        
        # 실제 좌표 변환
        if progress_callback:
            progress_callback(80, "좌표 변환 중...")
        print("Transforming to Real-World Coordinates")
        track_cycle = np.array(track_cycle)
        track_cycle = track_cycle * resolution - origin[0:2]
        
        # 스무딩
        if progress_callback:
            progress_callback(85, "스무딩 중...")
        smooth_method = params['smoothing_method']
        if smooth_method != 'none':
            print(f"Smoothing centerline using {smooth_method} method")
            original_cycle = track_cycle.copy()
            track_cycle = smooth_centerline_with_params(track_cycle, params)
            
            # 비교 플롯 표시
            if plot_mode > 0:
                plt.figure(figsize=(15, 6))
                
                plt.subplot(1, 2, 1)
                plt.plot(original_cycle[:, 0], original_cycle[:, 1], 'ro-', 
                        label='Original centerline', markersize=2, alpha=0.7)
                plt.plot(track_cycle[:, 0], track_cycle[:, 1], 'b-', 
                        label=f'Smoothed centerline ({smooth_method})', linewidth=2)
                plt.title('Centerline Smoothing Comparison')
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                plt.legend()
                plt.grid(True, alpha=0.3)
                plt.axis('equal')
                
                plt.subplot(1, 2, 2)
                plt.plot(track_cycle[:, 0], track_cycle[:, 1], 
                        linewidth=2, color='blue')
                plt.scatter(track_cycle[::50, 0], track_cycle[::50, 1], 
                           c='red', s=20, zorder=5, alpha=0.8)
                plt.title(f'Smoothed Centerline ({smooth_method})')
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                plt.grid(True, alpha=0.3)
                plt.axis('equal')
                
                plt.tight_layout()
                plt.show()
        
        if progress_callback:
            progress_callback(85, "트랙 폭 계산 중...")
        
        # 트랙 폭 계산 (옵션)
        left_distances = None
        right_distances = None
        if params.get('calculate_track_widths', False):
            # 원본 PGM 이미지 로드 (트랙 경계 감지용)
            original_image, _, _ = imageFromFile(params['pgm_path'])
            left_distances, right_distances = calculate_track_widths(
                track_cycle, original_image, resolution, origin
            )
            
            # Track width 시각화 (제공된 코드 방식)
            if plot_mode > 0:
                print("트랙 폭 시각화 중...")
                plt.figure(figsize=(12, 8))
                
                # 센터라인 그리기
                plt.plot(track_cycle[:, 0], track_cycle[:, 1], 'b-', linewidth=2, label='Centerline')
                
                # 경계점 계산 및 그리기 (모든 포인트 처리)
                left_boundary_points = []
                right_boundary_points = []
                valid_left_count = 0
                valid_right_count = 0
                
                for i in range(len(track_cycle)):
                    center_x, center_y = track_cycle[i, 0], track_cycle[i, 1]
                    left_dist, right_dist = left_distances[i], right_distances[i]
                    
                    # 방향 벡터 계산 (Raceline-Optimization 방식)
                    direction = calculate_direction_vector(track_cycle, i)
                    
                    # 수직 벡터 계산
                    left_perp, right_perp = calculate_perpendicular_vectors(direction)
                    
                    # 경계점 계산 및 표시 (최소 거리만 체크)
                    if left_dist > 0.1:
                        left_point = np.array([center_x, center_y]) + left_perp * left_dist
                        left_boundary_points.append(left_point)
                        plt.plot([center_x, left_point[0]], [center_y, left_point[1]], 'r-', alpha=0.6, linewidth=1)
                        valid_left_count += 1
                    
                    if right_dist > 0.1:
                        right_point = np.array([center_x, center_y]) + right_perp * right_dist
                        right_boundary_points.append(right_point)
                        plt.plot([center_x, right_point[0]], [center_y, right_point[1]], 'g-', alpha=0.6, linewidth=1)
                        valid_right_count += 1
                
                # 경계점들을 점으로 표시
                if left_boundary_points:
                    left_boundary_points = np.array(left_boundary_points)
                    plt.plot(left_boundary_points[:, 0], left_boundary_points[:, 1], 'r.', alpha=0.8, markersize=2, label='Left Boundary')
                
                if right_boundary_points:
                    right_boundary_points = np.array(right_boundary_points)
                    plt.plot(right_boundary_points[:, 0], right_boundary_points[:, 1], 'g.', alpha=0.8, markersize=2, label='Right Boundary')
                
                print(f"시각화된 경계선 수: 왼쪽 {valid_left_count}, 오른쪽 {valid_right_count} (전체 {len(track_cycle)}개 중)")
                
                plt.xlabel('X (meters)')
                plt.ylabel('Y (meters)')
                plt.title(f'F1tenth Track Centerline and Boundaries\nAvg Left: {np.mean(left_distances):.3f}m, Avg Right: {np.mean(right_distances):.3f}m')
                plt.legend()
                plt.axis('equal')
                plt.grid(True, alpha=0.3)
                plt.show()
        
        if progress_callback:
            progress_callback(90, "파일 저장 중...")
        
        # CSV 파일 저장
        print("Writing Output CSV")
        output_dir = params['output_dir']
        os.makedirs(output_dir, exist_ok=True)
        
        base_name = os.path.splitext(os.path.basename(name))[0]
        
        if left_distances is not None and right_distances is not None:
            # 확장된 CSV (center_x, center_y, left_distance, right_distance)
            output_path = os.path.join(output_dir, f"{base_name}_centerline_with_widths.csv")
            output_data = np.column_stack([track_cycle, left_distances, right_distances])
            header = "center_x,center_y,left_distance,right_distance"
            np.savetxt(output_path, output_data, delimiter=",", header=header, comments='')
            print(f"확장된 centerline 저장: {len(track_cycle)}개 포인트 (좌우 거리 포함)")
            
            # 통계 출력 (제공된 코드 방식)
            print(f"\n결과 통계:")
            print(f"평균 왼쪽 거리: {np.mean(left_distances):.3f}m")
            print(f"평균 오른쪽 거리: {np.mean(right_distances):.3f}m")
            print(f"왼쪽 거리 0인 포인트: {np.sum(left_distances == 0)}/{len(left_distances)}")
            print(f"오른쪽 거리 0인 포인트: {np.sum(right_distances == 0)}/{len(right_distances)}")
            print(f"왼쪽 거리 최대값: {np.max(left_distances):.3f}m")
            print(f"오른쪽 거리 최대값: {np.max(right_distances):.3f}m")
        else:
            # 기본 CSV (x, y)
            output_path = os.path.join(output_dir, f"{base_name}_centerline.csv")
            header = "x,y"
            np.savetxt(output_path, track_cycle, delimiter=",", header=header, comments='')
            print(f"기본 centerline 저장: {len(track_cycle)}개 포인트")
        
        print("Processing Time:", time.time() - start_time, "seconds")
        print("Finished. Output saved to", output_path)
        
        if progress_callback:
            progress_callback(100, f"완료: {output_path}")
        
        # 플롯 표시
        if plot_mode > 0:
            plt.show()
        
        return output_path
        
    except Exception as e:
        print(f"Error during processing: {e}")
        if progress_callback:
            progress_callback(0, f"오류: {e}")
        return None