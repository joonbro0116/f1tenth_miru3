#!/usr/bin/env python3
"""
Launch both the f1tenth_gym_ros simulator stack and the Forza MAP controller.

This ties together the simulator (bridge, map server, rviz, etc.) with the
forza_map_controller node so the whole stack can be started with one command.
Arguments are forwarded to the controller launch where relevant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    forza_pkg_share = get_package_share_directory('forza_map_controller')
    gym_pkg_share = get_package_share_directory('f1tenth_gym_ros')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            forza_pkg_share,
            'config',
            'forza_map_params.yaml',
        ]),
        description='Full path to the Forza MAP controller parameter file.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for both simulator and controller.'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gym_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gym_pkg_share, 'launch', 'gym_bridge_launch.py')
        )
    )

    forza_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(forza_pkg_share, 'launch', 'forza_map_sim.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        gym_launch,
        forza_launch,
    ])
