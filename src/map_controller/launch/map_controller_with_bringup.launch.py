#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_pkg = get_package_share_directory('map_controller')
    stack_pkg = get_package_share_directory('f1tenth_stack')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            map_pkg,
            'config',
            'map_controller_params.yaml',
        ]),
        description='MAP controller parameter file (includes csv_file_path).'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                stack_pkg,
                'launch',
                'bringup_launch.py',
            ])
        )
    )

    controller_node = Node(
        package='map_controller',
        executable='map_controller_node',
        name='map_controller_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/drive', '/ackermann_cmd_mux/input/navigation'),
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        bringup_launch,
        controller_node,
    ])
