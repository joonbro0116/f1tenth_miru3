#!/usr/bin/env python3

"""
F1TENTH SLAM Launch File
Supports both slam_toolbox and cartographer backends
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        choices=["slam_toolbox", "cartographer"],
        description="SLAM backend to use"
    )
    
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Custom config file path (optional)"
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sync",
        choices=["sync", "async"],
        description="SLAM mode for slam_toolbox"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_backend = LaunchConfiguration("slam_backend")
    config_file = LaunchConfiguration("config_file") 
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

    # SLAM Toolbox nodes group
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
                ["'", slam_backend, "' == 'slam_toolbox' and '", mode, "' == 'sync'"]
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
                ["'", slam_backend, "' == 'slam_toolbox' and '", mode, "' == 'async'"]
            )
        )
    ])

    # Cartographer nodes group  
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
            condition=IfCondition(["'", slam_backend, "' == 'cartographer'"])
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
            condition=IfCondition(["'", slam_backend, "' == 'cartographer'"])
        )
    ])

    return LaunchDescription([
        use_sim_time_arg,
        slam_backend_arg,
        config_file_arg,
        mode_arg,
        bringup_launch,
        slam_toolbox_group,
        cartographer_group
    ])