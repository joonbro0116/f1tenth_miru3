#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    map_pgm_file_arg = DeclareLaunchArgument(
        'map_pgm_file',
        default_value='/home/f1/f1tenth_ws/maps/gap_map_final_processed.pgm',
        description='Path to PGM map file'
    )
    
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='Map resolution in meters per pixel'
    )
    
    map_origin_x_arg = DeclareLaunchArgument(
        'map_origin_x',
        default_value='0.0',
        description='Map origin X coordinate'
    )
    
    map_origin_y_arg = DeclareLaunchArgument(
        'map_origin_y',
        default_value='0.0',
        description='Map origin Y coordinate'
    )
    
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.5',
        description='Pure pursuit lookahead distance'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',
        description='Maximum driving speed'
    )
    
    min_speed_arg = DeclareLaunchArgument(
        'min_speed',
        default_value='0.8',
        description='Minimum driving speed (gap follow mode)'
    )
    
    # Path follow node
    path_follow_node = Node(
        package='path_follow',
        executable='path_follow_node',
        name='path_follow_node',
        parameters=[{
            'map_pgm_file': LaunchConfiguration('map_pgm_file'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_origin_x': LaunchConfiguration('map_origin_x'),
            'map_origin_y': LaunchConfiguration('map_origin_y'),
            'track_width': 2.0,
            'safety_margin': 0.3,
            'boundary_erosion': 3,
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': LaunchConfiguration('min_speed'),
            'wheelbase': 0.3302,
            'obstacle_detection_distance': 2.0
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        map_pgm_file_arg,
        map_resolution_arg,
        map_origin_x_arg,
        map_origin_y_arg,
        lookahead_distance_arg,
        max_speed_arg,
        min_speed_arg,
        path_follow_node
    ])