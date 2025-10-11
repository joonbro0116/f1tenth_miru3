#!/usr/bin/env python3
"""
Forza MAP Controller - Simulation Launch
For use with f1tenth_gym_ros simulator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('forza_map_controller')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'forza_map_params.yaml',
        ]),
        description='Full path to the Forza MAP controller parameter file.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true.'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Odom to AMCL bridge (for simulator without AMCL)
    odom_to_amcl_bridge = Node(
        package='map_controller',  # Using existing bridge from map_controller
        executable='odom_to_amcl_bridge.py',
        name='odom_to_amcl_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('odom_in', '/ego_racecar/odom'),
            ('amcl_pose_out', '/amcl_pose'),
        ],
    )

    # Forza MAP Controller
    controller_node = Node(
        package='forza_map_controller',
        executable='controller_manager',
        name='forza_map_controller',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odom', '/ego_racecar/odom'),
            ('/drive', '/drive'),
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        odom_to_amcl_bridge,
        controller_node,
    ])
