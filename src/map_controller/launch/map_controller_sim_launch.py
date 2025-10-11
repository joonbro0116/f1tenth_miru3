#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('map_controller')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'map_controller_params.yaml',
        ]),
        description='Full path to the MAP controller parameter file.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Simulation time enabled
        description='Use simulation time if true.'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # MAP Controller node with remappings for gym-ros simulator
    controller_node = Node(
        package='map_controller',
        executable='map_controller_node',
        name='map_controller_manager',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # Remap to gym-ros simulator topics
            ('/odom', '/ego_racecar/odom'),
            ('/drive', '/ego_racecar/drive'),
            # Note: AMCL needs to be run separately for localization
            # Or use /ego_racecar/odom as both position and velocity source temporarily
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        controller_node,
    ])
