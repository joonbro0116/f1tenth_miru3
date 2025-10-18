#!/usr/bin/env python3
"""
Combined Launch: MAP Controller + Bound Obstacle Detector

This launch file starts both:
1. MAP Controller - Path following using pre-computed raceline
2. Bound Obstacle Detector - Real-time obstacle detection using LiDAR and track bounds

The two packages work independently:
- MAP controller generates drive commands following the raceline
- Bound detector publishes obstacle information on /bound_obstacles and /lane_blocked
- The controller can optionally subscribe to /lane_blocked to adjust behavior

Usage:
  ros2 launch map_controller map_with_bound.launch.py \
    csv_file_path:=/path/to/raceline.csv \
    outer_csv:=/path/to/outer_bound_world.csv \
    inner_csvs:=/path/to/inner_bound_world.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_controller_share = get_package_share_directory('map_controller')
    bound_detector_share = get_package_share_directory('bound_obstacle_detector')

    # Common arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for both packages.'
    )

    # MAP Controller arguments
    map_params_arg = DeclareLaunchArgument(
        'map_params_file',
        default_value=PathJoinSubstitution([
            map_controller_share,
            'config',
            'map_controller_params.yaml',
        ]),
        description='Full path to the MAP controller parameter file.'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value='',
        description='Path to waypoint CSV file (x, y, speed)'
    )

    # Bound Detector arguments
    bound_params_arg = DeclareLaunchArgument(
        'bound_params_file',
        default_value=PathJoinSubstitution([
            bound_detector_share,
            'config',
            'bound_detector_params.yaml',
        ]),
        description='Full path to the bound detector parameter file.'
    )

    outer_csv_arg = DeclareLaunchArgument(
        'outer_csv',
        default_value='',
        description='Path to outer bound CSV file (world coordinates)'
    )

    inner_csvs_arg = DeclareLaunchArgument(
        'inner_csvs',
        default_value='',
        description='Path to inner bound CSV file(s) (world coordinates)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_params_file = LaunchConfiguration('map_params_file')
    csv_file_path = LaunchConfiguration('csv_file_path')
    bound_params_file = LaunchConfiguration('bound_params_file')
    outer_csv = LaunchConfiguration('outer_csv')
    inner_csvs = LaunchConfiguration('inner_csvs')

    # MAP Controller launch
    map_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_controller_share, 'launch', 'map_controller.launch.py')
        ),
        launch_arguments={
            'params_file': map_params_file,
            'use_sim_time': use_sim_time,
            'csv_file_path': csv_file_path,
        }.items()
    )

    # Bound Detector launch
    bound_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bound_detector_share, 'launch', 'bound_detector.launch.py')
        ),
        launch_arguments={
            'params_file': bound_params_file,
            'use_sim_time': use_sim_time,
            'outer_csv': outer_csv,
            'inner_csvs': inner_csvs,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_params_arg,
        csv_file_arg,
        bound_params_arg,
        outer_csv_arg,
        inner_csvs_arg,
        map_controller_launch,
        bound_detector_launch,
    ])
