#!/usr/bin/env python3
"""
Forza MAP Controller - Real Vehicle Launch
For use with real F1TENTH car with AMCL localization
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
        default_value='false',
        description='Use simulation time if true.'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Forza MAP Controller
    controller_node = Node(
        package='forza_map_controller',
        executable='controller_manager',
        name='forza_map_controller',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # Real car topics (no remapping needed if using default /amcl_pose and /odom)
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        controller_node,
    ])
