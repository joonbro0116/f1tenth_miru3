#!/usr/bin/env python3
"""
MAP Controller Launch File

This standalone package provides MAP (Model-based Adaptive Path-following) controller
for autonomous racing. It follows a pre-computed raceline from CSV file.

Subscribes to:
- /amcl_pose: Vehicle pose in map frame
- /ego_racecar/odom: Vehicle odometry
- /imu/data: IMU data (optional, for acceleration-based steering scaling)

Publishes to:
- /drive: Ackermann drive commands
- /map_controller/lookahead_point: L1 lookahead point visualization
- /map_controller/path: Global raceline path
- /map_controller/waypoints_pose: Waypoint poses for visualization

Usage:
  ros2 launch map_controller map_controller.launch.py \
    csv_file_path:=/path/to/raceline.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        default_value='false',
        description='Use simulation time if true. Set to false for real car.'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value='',
        description='Path to waypoint CSV file (x, y, speed)'
    )

    def launch_setup(context, *args, **kwargs):
        params_file = LaunchConfiguration('params_file').perform(context)
        use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)
        csv_value = LaunchConfiguration('csv_file_path').perform(context).strip()

        param_list = [params_file]

        overrides = {
            'use_sim_time': use_sim_time_value.lower() in ('true', '1', 'yes')
        }
        if csv_value:
            overrides['csv_file_path'] = csv_value
        param_list.append(overrides)

        controller_node = Node(
            package='map_controller',
            executable='map_controller',
            name='map_controller',
            output='screen',
            parameters=param_list,
            remappings=[
                # Real car topics: /odom for velocity, /amcl_pose for localization
                # For sim, override with: odom:=/ego_racecar/odom
            ],
        )
        return [controller_node]

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        csv_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
