#!/usr/bin/env python3

"""
F1TENTH C++ 최소 곡률 경로 플래너 런치 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="시뮬레이션 시간 사용 여부"
    )
    
    track_width_arg = DeclareLaunchArgument(
        "track_width",
        default_value="2.0",
        description="트랙 너비 (미터)"
    )
    
    safety_margin_arg = DeclareLaunchArgument(
        "safety_margin",
        default_value="0.3",
        description="안전 마진 (미터)"
    )
    
    num_waypoints_arg = DeclareLaunchArgument(
        "num_waypoints",
        default_value="100",
        description="웨이포인트 개수"
    )
    
    min_radius_arg = DeclareLaunchArgument(
        "min_radius",
        default_value="0.5",
        description="최소 회전 반경 (미터)"
    )

    # 새로운 Launch arguments 추가
    map_pgm_file_arg = DeclareLaunchArgument(
        "map_pgm_file",
        default_value="/home/f1/f1tenth_ws/maps/map.pgm",
        description="PGM 형식의 맵 파일 경로"
    )
    
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value="/home/f1/f1tenth_ws/maps/map.yaml",
        description="YAML 형식의 맵 정보 파일 경로"
    )
    
    subsample_period_arg = DeclareLaunchArgument(
        "subsample_period",
        default_value="6",
        description="중심선 포인트 서브샘플링 주기"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    track_width = LaunchConfiguration("track_width")
    safety_margin = LaunchConfiguration("safety_margin")
    num_waypoints = LaunchConfiguration("num_waypoints")
    min_radius = LaunchConfiguration("min_radius")
    map_pgm_file = LaunchConfiguration("map_pgm_file")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    subsample_period = LaunchConfiguration("subsample_period")

    # # C++ 최소 곡률 플래너 노드
    # planner_node = Node(
    #     package="path_planner",
    #     executable="path_planner",
    #     name="path_planner",
    #     output="screen",
    #     parameters=[{
    #         "use_sim_time": use_sim_time,
    #         "track_width": track_width,
    #         "safety_margin": safety_margin,
    #         "num_waypoints": num_waypoints,
    #         "optimization_weight": 1.0,
    #         "min_radius": min_radius
    #     }]
    # )

    # 중심선 추출 노드 추가
    centerline_extractor_node = Node(
        package="path_planner",
        executable="path_planner_centerline_extractor",
        name="track_centerline_extractor",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_pgm_file": map_pgm_file,
            "map_yaml_file": map_yaml_file,
            "subsample_period": subsample_period
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        track_width_arg,
        safety_margin_arg,
        num_waypoints_arg,
        min_radius_arg,
        map_pgm_file_arg,
        map_yaml_file_arg,
        subsample_period_arg,
        centerline_extractor_node,
        centerline_extractor_node
    ])