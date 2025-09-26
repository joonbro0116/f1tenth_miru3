#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments for real vehicle
    csv_file_path_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value='/home/f1/f1tenth_ws/joon_path_generate/raceline/jg_thursday.csv',
        description='Path to the CSV file containing waypoints for real vehicle'
    )

    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.0',
        description='Lookahead distance for pure pursuit (real vehicle)'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='5.0',
        description='Maximum speed for real vehicle'
    )

    target_speed_straight_arg = DeclareLaunchArgument(
        'target_speed_straight',
        default_value='3.0',
        description='Target speed for straight sections'
    )

    target_speed_curve_arg = DeclareLaunchArgument(
        'target_speed_curve',
        default_value='1.5',
        description='Target speed for curve sections'
    )

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.3302',
        description='Wheelbase of the real vehicle'
    )

    # PID parameters
    speed_kp_arg = DeclareLaunchArgument(
        'speed_kp',
        default_value='0.3',
        description='PID proportional gain for speed control'
    )

    speed_ki_arg = DeclareLaunchArgument(
        'speed_ki',
        default_value='0.01',
        description='PID integral gain for speed control'
    )

    speed_kd_arg = DeclareLaunchArgument(
        'speed_kd',
        default_value='0.08',
        description='PID derivative gain for speed control'
    )

    # Topic configurations for real vehicle
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='Odometry topic for real vehicle'
    )

    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/vesc/low_level/ackermann_cmd_mux/output',
        description='Drive command topic for real vehicle'
    )

    # Real Pure pursuit node
    real_pure_pursuit_node = Node(
        package='real_pure_pursuit',
        executable='real_pure_pursuit_node',
        name='real_pure_pursuit_node',
        output='screen',
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file_path'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_speed': LaunchConfiguration('max_speed'),
            'target_speed_straight': LaunchConfiguration('target_speed_straight'),
            'target_speed_curve': LaunchConfiguration('target_speed_curve'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'speed_kp': LaunchConfiguration('speed_kp'),
            'speed_ki': LaunchConfiguration('speed_ki'),
            'speed_kd': LaunchConfiguration('speed_kd'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'drive_topic': LaunchConfiguration('drive_topic'),
            'use_sim_time': False  # Real vehicle uses system time
        }]
    )

    return LaunchDescription([
        csv_file_path_arg,
        lookahead_distance_arg,
        max_speed_arg,
        target_speed_straight_arg,
        target_speed_curve_arg,
        wheelbase_arg,
        speed_kp_arg,
        speed_ki_arg,
        speed_kd_arg,
        odom_topic_arg,
        drive_topic_arg,
        real_pure_pursuit_node,
    ])