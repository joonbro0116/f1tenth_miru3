include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,

  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4 -- 8코어 이상이 아니면 4 정도로 낮추는 것이 안정적일 수 있습니다.

TRAJECTORY_BUILDER_2D.use_imu_data = true -- IMU 사용 (launch 파일에서 remapping 필수!)
TRAJECTORY_BUILDER_2D.min_range = 0.1 -- 센서 스펙에 맞게 조정 (보통 0.1m 정도부터 유효)
TRAJECTORY_BUILDER_2D.max_range = 10.0

-- Scan matching: 스캔 매칭 정확도 향상
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e2  -- 값을 높여서 스캔 데이터를 더 신뢰하도록 함
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 1e2  -- 값을 높여서 스캔 데이터를 더 신뢰하도록 함

-- Submaps / optimization: 서브맵 안정성 향상
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 서브맵 당 스캔 데이터 수를 늘림
POSE_GRAPH.optimize_every_n_nodes            = 35  -- 서브맵 크기에 맞춰 조정

-- Constraints (loop closure): 루프 클로징 신뢰도 향상
POSE_GRAPH.constraint_builder.sampling_ratio            = 0.3   -- 검사 빈도를 약간 줄임
POSE_GRAPH.constraint_builder.max_constraint_distance   = 12.0
POSE_GRAPH.constraint_builder.min_score                 = 0.85  -- 정확도 기준을 높여 잘못된 연결 방지
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9 -- 정확도 기준을 높여 잘못된 연결 방지

return options
