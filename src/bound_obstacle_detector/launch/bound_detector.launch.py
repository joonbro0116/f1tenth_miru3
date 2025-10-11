# launch/bound_detector.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bound_obstacle_detector',      # 패키지명
            executable='bound_detector',            # 실행 파일(엔트리포인트)명
            name='bound_obstacle_detector',
            output='screen',
            parameters=[{
                # --- 토픽 ---
                'scan_topic': '/scan',
                'odom_topic': '/ego_racecar/odom',

                # --- 경계 CSV (outer는 단일, inner는 다중/글롭/콤마 구분 지원) ---
                'outer_csv':  '/home/moon/sim_ws/bounds_out/outer_bound_world.csv',
                'inner_csvs': '/home/moon/sim_ws/bounds_out/inner_bound_world.csv',
                # 예) 여러 개: 'bounds_out/inner_*.csv' 또는 'a.csv,b.csv;c.csv'

                # --- 새 파이프라인 필수(맵과 반드시 일치) ---
                'mask_width':      361,                # 맵 이미지 width(px)
                'mask_height':     226,                # 맵 이미지 height(px)
                'mask_resolution': 0.05,                # m/px  (map.yaml의 resolution)
                'mask_origin':     [-6.73, -3.63, 0],     # [x, y, yaw] (map.yaml의 origin)

                # --- CORE / RING 설정 ---
                'core_shrink_m':     0.20,   # 내부를 이만큼 침식하여 CORE 생성(0.15~0.30 권장)
                'ring_small_margin': 0.08,   # RING 전용 소(小)마진(0.02~0.04)

                # --- 탐지/클러스터 파라미터 ---
                # (cluster_method는 새 코드에서 사용하지 않음—유클리드 고정)
                # 'cluster_method': 'euclid',
                'euclid_eps': 0.28,
                'euclid_min_pts': 3,         # CORE=1~2, RING은 코드에서 자동으로 2 이상 사용

                # --- 스캔/ROI/센서 보정 ---
                'roi_deg': 90.0,
                'range_max': 10.0,
                'laser_yaw_offset_deg': 0.0,  # 라이다 장착 각도 보정(+CCW)
                'include_boundary': True,      # 경계선 위 포함 여부

                # --- 차로 차단 판정 ---
                'lane_block_half_width': 0.5,
                'lane_block_dist': 8.0,

                # --- (이전 파이프라인 전용: 현재 코드에선 미사용) ---
                # 'wall_margin': 0.00,            # 새 파이프라인에서는 RING에만 작은 마진 사용
                # 'free_mask_path': '/home/moon/sim_ws/bounds_out/free_eroded.png',
                # 'map_yaml': '/home/moon/sim_ws/map.yaml',
            }]
        )
    ])
