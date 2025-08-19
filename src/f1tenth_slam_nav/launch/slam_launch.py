#!/usr/bin/env python3

"""
F1TENTH SLAM Launch File
Supports slam_toolbox (sync/async) and cartographer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("f1tenth_slam_nav")
    f1tenth_stack_share = get_package_share_directory("f1tenth_stack")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true"
    )

    slam_backend_arg = DeclareLaunchArgument(
        "slam_backend",
        default_value="slam_toolbox",
        description="Choose SLAM backend: slam_toolbox or cartographer"
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sync",
        description="SLAM mode for slam_toolbox (sync or async)"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_backend = LaunchConfiguration("slam_backend")
    mode = LaunchConfiguration("mode")

    # Include F1TENTH bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([f1tenth_stack_share, "launch", "bringup_launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # SLAM Toolbox nodes (sync / async)
    slam_toolbox_group = GroupAction([
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "slam_toolbox_config.yaml"]),
                {"use_sim_time": use_sim_time}
            ],
            condition=IfCondition(
                PythonExpression([
                    "'", slam_backend, "' == 'slam_toolbox' and '", mode, "' == 'sync'"
                ])
            )
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "slam_toolbox_async_config.yaml"]),
                {"use_sim_time": use_sim_time}
            ],
            condition=IfCondition(
                PythonExpression([
                    "'", slam_backend, "' == 'slam_toolbox' and '", mode, "' == 'async'"
                ])
            )
        )
    ])

    # Cartographer nodes
    cartographer_group = GroupAction([
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", PathJoinSubstitution([pkg_share, "config"]),
                "-configuration_basename", "cartographer_config.lua"
            ],
            remappings=[("/scan", "scan")],
            condition=IfCondition(
                PythonExpression(["'", slam_backend, "' == 'cartographer'"])
            )
        ),
        Node(
            package="cartographer_ros",
            executable="occupancy_grid_node",
            name="occupancy_grid_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "resolution": 0.05,
                "publish_period_sec": 1.0
            }],
            condition=IfCondition(
                PythonExpression(["'", slam_backend, "' == 'cartographer'"])
            )
        )
    ])

    return LaunchDescription([
        use_sim_time_arg,
        slam_backend_arg,
        mode_arg,
        bringup_launch,
        slam_toolbox_group,
        cartographer_group
    ])