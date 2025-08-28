import time
import argparse
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
from tkinter import Tk
from tkinter.filedialog import askopenfile, askdirectory

def imageFromFile(pgmf):

    if pgmf is None:
        pgmf = askopenfile(mode='rb', filetypes=[('PGM Files', '*.pgm')], title='Select PGM Track Image File')

    if pgmf is None:
        sys.exit("Invalid PGM File")

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

    return np.array(image), depth, pgmf.name

def infoFromFile(yamlf):

    if yamlf is None:
        yamlf = askopenfile(mode='r', filetypes=[('YAML Files', '*.yaml *.yml')], title='Select Track Info YAML File')

    if yamlf is None:
        sys.exit("Invalid YAML File")

    data = yaml.safe_load(yamlf)
    resolution = None
    origin = None
    try:
        resolution = data['resolution']
        origin = data['origin']
    except KeyError:
        sys.exit("Error Reading YAML File")

    return resolution, origin

def outputDirFromFile(output_dir):

    if output_dir is None:
        output_dir = askdirectory(title='Select Output Directory')

    if output_dir is None:
        sys.exit("Invalid Output Directory")

    return output_dir

def plot(image, title, plot_mode):
    if plot_mode > 0:
        fig = plt.figure()
        fig.canvas.set_window_title(title)
        plt.imshow(image, interpolation='nearest', cmap='Greys')

def plotExtra(image, title, plot_mode):
    if plot_mode > 1:
        plot(image, title, plot_mode)

def pixelsInCircle(radius):
    num = 0
    for i in range(100000):
        num += math.floor(radius**2. / (4.*i + 1.)) - math.floor(radius**2. / (4.*i + 3.))
    num *= 4
    num += 1
    return num

def denoise(image, depth, plot_mode):

    # 더 보수적인 디노이즈: 작은 radius와 더 적은 min_samples
    radius = 2
    db_scan = DBSCAN(eps=radius, min_samples=max(1, pixelsInCircle(radius)//8))

    points = pixels2points(image, False)

    clusters = db_scan.fit(points).labels_
    cluster_sizes = np.bincount(clusters + 1)
    if len(cluster_sizes) < 2:
        raise RuntimeError('No clusters found')
    cluster_sizes = cluster_sizes[1:] # ignore noise at first index
    
    # 더 보수적 접근: 최소 크기 이상의 모든 클러스터 유지
    min_cluster_size = max(10, len(points) // 100)  # 전체 포인트의 1% 이상 또는 최소 10개
    valid_clusters = [i for i, size in enumerate(cluster_sizes) if size >= min_cluster_size]
    
    points = [x for i, x in enumerate(points) if clusters[i] in valid_clusters]

    height, width = image.shape
    image = points2pixels(points, width, height, depth, False)

    negative_points = pixels2points(image, True)

    # 더 보수적인 두 번째 DBSCAN
    db_scan = DBSCAN(eps=1.5, min_samples=1)

    clusters = db_scan.fit(negative_points).labels_
    cluster_sizes = np.bincount(clusters + 1)
    if len(cluster_sizes) < 3:
        raise RuntimeError('Not enough clusters found')
    cluster_sizes = cluster_sizes[1:] # ignore noise at first index
    main_clusters = np.argsort(cluster_sizes)[-3:]

    negative_points = [x for i, x in enumerate(negative_points) if clusters[i] in main_clusters]

    return points2pixels(negative_points, width, height, depth, True)

def prune(image, plot_mode):
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

def subsample(image, cycle, period):
    subsampled_cycle = []
    for i, point in enumerate(cycle):
        if i % period == 0 and len(cycle) - i >= period / 2:
            subsampled_cycle.append(point)

    height, width = image.shape
    image = points2pixels(subsampled_cycle, width, height, depth, False)
    
    return image, subsampled_cycle

def calculate_curvature(points):
    """각 포인트에서의 곡률을 계산합니다"""
    points = np.array(points)
    n = len(points)
    curvatures = np.zeros(n)
    
    for i in range(n):
        # 이전, 현재, 다음 포인트 (순환)
        p1 = points[(i-1) % n]
        p2 = points[i]
        p3 = points[(i+1) % n]
        
        # 벡터 계산
        v1 = p2 - p1
        v2 = p3 - p2
        
        # 각도 변화 계산
        if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_angle = np.clip(cos_angle, -1, 1)  # 수치 안정성
            angle_change = np.arccos(cos_angle)
            curvatures[i] = angle_change
    
    return curvatures

def smooth_high_curvature_areas(points, curvature_threshold=0.4, window_size=60, iterations=5):
    """곡률이 높은 부분만 선택적으로 스무딩합니다"""
    points = np.array(points)
    curvatures = calculate_curvature(points)
    
    print(f"곡률 분석: 평균={curvatures.mean():.3f}, 최대={curvatures.max():.3f}")
    print(f"곡률 임계값 {curvature_threshold} 이상인 포인트: {np.sum(curvatures > curvature_threshold)}개")
    
    # 곡률이 높은 부분 주변을 확장해서 스무딩 영역 결정
    high_curvature_mask = curvatures > curvature_threshold
    smooth_mask = np.zeros(len(points), dtype=bool)
    
    # 곡률이 높은 포인트 주변으로 마스크 확장
    for i, is_high_curv in enumerate(high_curvature_mask):
        if is_high_curv:
            for j in range(-window_size//2, window_size//2 + 1):
                idx = (i + j) % len(points)
                smooth_mask[idx] = True
    
    print(f"Smoothing applied to {np.sum(smooth_mask)} points ({np.sum(smooth_mask)/len(points)*100:.1f}% of total)")
    
    smoothed_points = points.copy()
    
    # 여러 번 반복해서 더 부드럽게
    for iteration in range(iterations):
        new_points = smoothed_points.copy()
        
        for i, should_smooth in enumerate(smooth_mask):
            if should_smooth:
                # 주변 포인트들로 가중 평균
                neighbor_indices = []
                for j in range(-window_size//2, window_size//2 + 1):
                    neighbor_idx = (i + j) % len(points)
                    neighbor_indices.append(neighbor_idx)
                
                # 거리 기반 가중치 (가까울수록 높은 가중치)
                weights = []
                for neighbor_idx in neighbor_indices:
                    distance = min(abs(neighbor_idx - i), len(points) - abs(neighbor_idx - i))  # 순환 거리
                    weight = np.exp(-0.5 * (distance / (window_size // 4))**2)
                    weights.append(weight)
                
                weights = np.array(weights)
                weights /= weights.sum()
                
                # 가중 평균으로 새 위치 계산
                smooth_x = np.sum([smoothed_points[idx][0] * w for idx, w in zip(neighbor_indices, weights)])
                smooth_y = np.sum([smoothed_points[idx][1] * w for idx, w in zip(neighbor_indices, weights)])
                
                # 원본과 스무딩 결과를 적절히 섞어서 너무 급격한 변화 방지
                blend_factor = 0.3  # 30%만 스무딩 적용
                new_points[i][0] = (1 - blend_factor) * smoothed_points[i][0] + blend_factor * smooth_x
                new_points[i][1] = (1 - blend_factor) * smoothed_points[i][1] + blend_factor * smooth_y
        
        smoothed_points = new_points
        print(f"Smoothing iteration {iteration + 1}/{iterations} completed")
    
    return smoothed_points

def smooth_centerline(points, smoothing_method='curvature', spline_points=None, sigma=2.0, curvature_threshold=0.3):
    """
    centerline 포인트들을 부드럽고 미분가능한 곡선으로 변환
    
    Parameters:
    - points: centerline 포인트들 [(x, y), ...]
    - smoothing_method: 'curvature', 'spline', 'gaussian', 'both'
    - curvature_threshold: 곡률 임계값 (높을수록 더 급한 코너만 스무딩)
    - spline_points: spline 보간할 포인트 수 (None이면 원래 포인트 수의 2배)
    - sigma: 가우시안 필터 시그마 값
    """
    if len(points) < 3:
        print(f"포인트가 너무 적습니다: {len(points)}개")
        return np.array(points)
    
    points = np.array(points)
    print(f"원본 포인트 수: {len(points)}")
    
    if smoothing_method == 'curvature':
        print("곡률 기반 선택적 스무딩 적용 중...")
        return smooth_high_curvature_areas(points, curvature_threshold)
    
    # 기존 방법들
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
            if spline_points is None:
                spline_points = len(points) * 2
            
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

def world_to_pixel_coords(world_points, resolution, origin):
    """실제 좌표를 픽셀 좌표로 변환"""
    pixel_points = []
    for point in world_points:
        # 실제 좌표를 픽셀 좌표로 변환
        pixel_x = int((point[0] + origin[0]) / resolution)
        pixel_y = int((point[1] + origin[1]) / resolution)
        pixel_points.append([pixel_x, pixel_y])
    return np.array(pixel_points)

def bilinear_interpolation(image, x, y):
    """bilinear 보간으로 서브픽셀 값 계산"""
    height, width = image.shape
    
    x1 = int(np.floor(x))
    x2 = min(x1 + 1, width - 1)
    y1 = int(np.floor(y))
    y2 = min(y1 + 1, height - 1)
    
    if x1 < 0 or y1 < 0 or x2 >= width or y2 >= height:
        return 0
    
    # 가중치 계산
    wx2 = x - x1
    wx1 = 1 - wx2
    wy2 = y - y1
    wy1 = 1 - wy2
    
    # 보간
    return (image[y1, x1] * wx1 * wy1 +
            image[y1, x2] * wx2 * wy1 +
            image[y2, x1] * wx1 * wy2 +
            image[y2, x2] * wx2 * wy2)

def ray_cast_to_boundary(start_point, direction, pgm_image):
    """시작점에서 특정 방향으로 ray casting하여 트랙 경계까지의 거리 계산"""
    height, width = pgm_image.shape
    max_distance = min(width, height)
    
    current_x, current_y = start_point
    step_size = 0.5  # 서브픽셀 정확도
    
    for distance in np.arange(step_size, max_distance, step_size):
        # 현재 위치 계산
        test_x = current_x + direction[0] * distance
        test_y = current_y + direction[1] * distance
        
        # 이미지 경계 확인
        if test_x < 0 or test_x >= width or test_y < 0 or test_y >= height:
            return distance
        
        # 픽셀 값 확인 (보간 사용)
        pixel_value = bilinear_interpolation(pgm_image, test_x, test_y)
        
        # 트랙 경계 감지 (벽 = 검은색, 자유공간 = 흰색)
        if pixel_value < 127:  # 임계값 (트랙 경계)
            return distance
    
    return max_distance

def calculate_track_widths_integrated(centerline_points, pgm_image, resolution, origin):
    """centerline 각 포인트에서 좌우 트랙 폭 계산 (Raceline-Optimization 호환 방식)"""
    print("트랙 폭 계산 중... (Raceline-Optimization 호환 법선벡터 사용)")
    
    if len(centerline_points) < 3:
        print("포인트가 너무 적어 트랙 폭을 계산할 수 없습니다.")
        return None, None
    
    # 실제 좌표를 픽셀 좌표로 변환
    pixel_points = world_to_pixel_coords(centerline_points, resolution, origin)
    
    left_distances = []
    right_distances = []
    
    height, width = pgm_image.shape
    
    for i, pixel_point in enumerate(pixel_points):
        # Raceline-Optimization 방식: 인접점 기반 방향 벡터 계산
        current_point = pixel_point
        
        if i == 0:
            # 첫 번째 점: 다음 점 방향
            if len(pixel_points) > 1:
                direction_vector = np.array([
                    pixel_points[1][0] - pixel_points[0][0], 
                    pixel_points[1][1] - pixel_points[0][1]
                ])
            else:
                direction_vector = np.array([1, 0])
        elif i == len(pixel_points) - 1:
            # 마지막 점: 이전 점에서의 방향
            direction_vector = np.array([
                pixel_points[i][0] - pixel_points[i-1][0], 
                pixel_points[i][1] - pixel_points[i-1][1]
            ])
        else:
            # 중간 점: 이전과 다음 점 사이의 방향 (Raceline-Optimization 방식)
            direction_vector = np.array([
                pixel_points[i+1][0] - pixel_points[i-1][0], 
                pixel_points[i+1][1] - pixel_points[i-1][1]
            ])
        
        # 벡터 정규화
        if np.linalg.norm(direction_vector) > 0:
            direction_vector = direction_vector / np.linalg.norm(direction_vector)
        else:
            direction_vector = np.array([1, 0])  # 기본값
        
        # 수직 벡터 계산 (법선 방향)
        # 좌측: 진행방향에서 시계방향 90도 회전
        left_normal = np.array([-direction_vector[1], direction_vector[0]])
        # 우측: 진행방향에서 반시계방향 90도 회전
        right_normal = np.array([direction_vector[1], -direction_vector[0]])
        
        # 좌측 거리 측정
        left_dist = ray_cast_to_boundary(current_point, left_normal, pgm_image)
        left_distances.append(left_dist * resolution)  # 픽셀을 미터로 변환
        
        # 우측 거리 측정
        right_dist = ray_cast_to_boundary(current_point, right_normal, pgm_image)
        right_distances.append(right_dist * resolution)  # 픽셀을 미터로 변환
    
    print(f"트랙 폭 계산 완료: 평균 좌측 {np.mean(left_distances):.2f}m, 평균 우측 {np.mean(right_distances):.2f}m")
    return np.array(left_distances), np.array(right_distances)

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Fits a center-line to a closed track.')
    parser.add_argument('--pgm_path', help='file path to the pgm track image file', nargs='?', type=argparse.FileType('rb'), default=None)
    parser.add_argument('--yaml_path', help='file path to the yaml track info file', nargs='?', type=argparse.FileType('r'), default=None)
    parser.add_argument('--subsample_period', help='defines the subsampling rate of the center-line pixels for the output points', nargs='?', type=int, default=1)
    parser.add_argument('--plot_mode', help='0: no plots, 1: basic plots, 2: all plots', nargs='?', type=int, default=1)
    parser.add_argument('--smooth_method', help='smoothing method: curvature, spline, gaussian, both, none', nargs='?', type=str, default='curvature')
    parser.add_argument('--smooth_sigma', help='gaussian smoothing sigma value', nargs='?', type=float, default=2.0)
    parser.add_argument('--curvature_threshold', help='curvature threshold for selective smoothing (higher = only sharp corners)', nargs='?', type=float, default=0.3)
    parser.add_argument('--out_dir', help='directory path to write output to', nargs='?', type=argparse.FileType('w'), default=None)
    parser.add_argument('--calculate_track_widths', help='calculate left and right track widths', action='store_true')
    args = parser.parse_args()

    plot_mode = args.plot_mode
    subsample_period = args.subsample_period
    smooth_method = args.smooth_method
    smooth_sigma = args.smooth_sigma
    curvature_threshold = args.curvature_threshold
    calculate_track_widths = args.calculate_track_widths
    
    # Get track image and info from file
    Tk().withdraw()
    image, depth, name = imageFromFile(args.pgm_path)
    resolution, origin = infoFromFile(args.yaml_path)

    print(resolution)

    start_time = time.time()

    plot(image, "Original Image", plot_mode)

    # Denoise image
    print("Denoising")
    image = denoise(image, depth, plot_mode)
    denoised = image

    plotExtra(image, "Denoised", plot_mode)

    # Skeletonize Image
    print("Skeletonizing")
    image = skeletonize(np.where(image > 0, 1, 0)) # convert to binary image and skeletonize
    skeleton_overlay = np.where(image == 1, 0, denoised)
    image = image * depth

    plotExtra(image, "Skeletonized", plot_mode)
    plotExtra(skeleton_overlay, "Skeletonized Overlayed on Denoised", plot_mode)

    # Prune Image
    print("Pruning")
    image, track_cycle = prune(image, plot_mode)
    overlayed = np.where(image > 0, 0, denoised)

    plotExtra(image, "Pruned", plot_mode)
    plotExtra(overlayed, "Pruned Overlayed on Denoised", plot_mode)

    # Subsample
    print("Subsampling")
    image, track_cycle = subsample(image, track_cycle, subsample_period)

    plot(image, "Output", plot_mode)

    # Transform to Real-World Coordinates
    print("Transforming to Real-World Coordinates")
    track_cycle = np.array(track_cycle)
    track_cycle = track_cycle * resolution - origin[0:2]
    
    # Smooth centerline if requested
    if smooth_method != 'none':
        print(f"Smoothing centerline using {smooth_method} method")
        original_cycle = track_cycle.copy()
        track_cycle = smooth_centerline(track_cycle, smooth_method, sigma=smooth_sigma, curvature_threshold=curvature_threshold)
        
        # 원본과 스무딩된 결과 비교 플롯
        if plot_mode > 0:
            # 비교 플롯
            plt.figure(figsize=(15, 6))
            
            # 첫 번째 서브플롯: 비교
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
            
            # 두 번째 서브플롯: 스무딩된 결과만
            plt.subplot(1, 2, 2)
            plt.plot(track_cycle[:, 0], track_cycle[:, 1], 'b-', 
                    linewidth=2, color='blue')
            plt.scatter(track_cycle[::50, 0], track_cycle[::50, 1], 
                       c='red', s=20, zorder=5, alpha=0.8)  # 50개마다 포인트 표시
            plt.title(f'Smoothed Centerline ({smooth_method})')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            
            plt.tight_layout()
            plt.show()
            
            # 스무딩된 결과만 따로 큰 플롯
            plt.figure(figsize=(12, 10))
            plt.plot(track_cycle[:, 0], track_cycle[:, 1], 'b-', 
                    linewidth=3, color='darkblue', label='스무딩된 centerline')
            plt.scatter(track_cycle[::30, 0], track_cycle[::30, 1], 
                       c='red', s=30, zorder=5, alpha=0.8, label='포인트 (30개마다)')
            plt.title(f'최종 스무딩된 Centerline\n({len(track_cycle)}개 포인트, {smooth_method} 방법)', 
                     fontsize=14, fontweight='bold')
            plt.xlabel('X (m)', fontsize=12)
            plt.ylabel('Y (m)', fontsize=12)
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            
            # 시작점 표시
            plt.scatter(track_cycle[0, 0], track_cycle[0, 1], 
                       c='green', s=100, marker='s', zorder=10, 
                       label='시작점', edgecolor='black', linewidth=2)
            plt.legend()
            plt.show()

    # Calculate track widths if requested
    if calculate_track_widths:
        print("좌우 트랙 폭 계산 중...")
        left_widths, right_widths = calculate_track_widths_integrated(track_cycle, depth, resolution, origin)
        print(f"좌우 폭 계산 완료: 평균 좌측 폭 {np.mean(left_widths):.3f}m, 평균 우측 폭 {np.mean(right_widths):.3f}m")
        
        # Track width 시각화
        if plot_mode > 0:
            print("트랙 폭 시각화 중...")
            plt.figure(figsize=(15, 10))
            
            # Centerline 그리기
            plt.plot(track_cycle[:, 0], track_cycle[:, 1], 'b-', linewidth=3, label='Centerline')
            
            # 좌우 경계 포인트 계산 및 시각화 (일부만)
            visualization_step = max(1, len(track_cycle) // 50)  # 50개 정도만 시각화
            for i in range(0, len(track_cycle), visualization_step):
                # 방향 벡터 계산 (Raceline-Optimization 방식)
                if i == 0:
                    direction = track_cycle[1] - track_cycle[0]
                elif i == len(track_cycle) - 1:
                    direction = track_cycle[i] - track_cycle[i-1]
                else:
                    direction = track_cycle[i+1] - track_cycle[i-1]
                
                # 방향 벡터 정규화
                direction_length = np.linalg.norm(direction)
                if direction_length > 0:
                    direction = direction / direction_length
                    
                    # 좌우 수직 벡터 (오른쪽이 양수)
                    normal_right = np.array([direction[1], -direction[0]])
                    normal_left = -normal_right
                    
                    # 좌우 경계점 계산
                    left_point = track_cycle[i] + normal_left * left_widths[i]
                    right_point = track_cycle[i] + normal_right * right_widths[i]
                    
                    # 경계선 그리기
                    plt.plot([left_point[0], track_cycle[i, 0]], [left_point[1], track_cycle[i, 1]], 
                            'r-', alpha=0.3, linewidth=1)
                    plt.plot([track_cycle[i, 0], right_point[0]], [track_cycle[i, 1], right_point[1]], 
                            'g-', alpha=0.3, linewidth=1)
            
            plt.title(f'Centerline with Track Boundaries\n평균 좌측 폭: {np.mean(left_widths):.3f}m, 평균 우측 폭: {np.mean(right_widths):.3f}m', 
                     fontsize=14, fontweight='bold')
            plt.xlabel('X (m)', fontsize=12)
            plt.ylabel('Y (m)', fontsize=12)
            plt.legend(['Centerline', 'Left Boundaries', 'Right Boundaries'])
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            plt.show()
        
        # 확장된 데이터 준비 (x, y, left_width, right_width)
        extended_data = np.column_stack((track_cycle, left_widths, right_widths))
        csv_header = "x,y,left_width,right_width"
    else:
        extended_data = track_cycle
        csv_header = "x,y"

    print("Processing Time:", time.time() - start_time, "seconds")

    # Save to CSV File
    print("Writing Output CSV")
    output_dir = "/home/ojg/sim_ws/maps/"
    base_name = os.path.splitext(os.path.basename(name))[0]
    if calculate_track_widths:
        output_path = os.path.join(output_dir, base_name + '_centerline_with_widths.csv')
    else:
        output_path = os.path.join(output_dir, base_name + '_centerline.csv')
    
    # 헤더와 함께 저장
    np.savetxt(output_path, extended_data, delimiter=",", header=csv_header, comments='')

    print("Finished. Output saved to", output_path)

    # Display Plots
    if plot_mode > 0:
        plt.show()
