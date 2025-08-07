#!/usr/bin/env python3

"""
F1TENTH SLAM & Navigation Unified Launch File

This launch file provides a unified interface for both SLAM and localization modes.
It automatically switches between mapping and localization based on the mode parameter.

Usage:
  # SLAM mode (mapping)
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=slam_toolbox
  
  # Localization mode (using existing map)  
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization map_yaml_file:=/path/to/map.yaml
  
  # Cartographer SLAM
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=cartographer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("f1tenth_slam_nav")
    
    # Get launch configurations
    mode = LaunchConfiguration("mode").perform(context)
    slam_backend = LaunchConfiguration("slam_backend").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    
    launch_actions = []
    
    if mode == "slam":
        # SLAM mode - include slam launch
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, "launch", "slam_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_backend": slam_backend,
                "mode": LaunchConfiguration("slam_mode")
            }.items()
        )
        launch_actions.append(slam_launch)
        
    elif mode == "localization":
        # Localization mode - include localization launch  
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, "launch", "localization_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml_file,
                "autostart": LaunchConfiguration("autostart")
            }.items()
        )
        launch_actions.append(localization_launch)
    
    return launch_actions


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        choices=["slam", "localization"],
        description="Mode: 'slam' for mapping, 'localization' for using existing map"
    )
    
    slam_backend_arg = DeclareLaunchArgument(
        "slam_backend", 
        default_value="slam_toolbox",
        choices=["slam_toolbox", "cartographer"],
        description="SLAM backend to use (only applies in slam mode)"
    )
    
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="sync",
        choices=["sync", "async"],
        description="SLAM mode for slam_toolbox"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false", 
        description="Use simulation time if true"
    )
    
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value="/home/f1/f1tenth_ws/maps/gap_map_final.yaml",
        description="Path to map yaml file (only applies in localization mode)"
    )
    
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start lifecycle nodes"
    )

    return LaunchDescription([
        mode_arg,
        slam_backend_arg,
        slam_mode_arg,
        use_sim_time_arg,
        map_yaml_file_arg,
        autostart_arg,
        OpaqueFunction(function=launch_setup)
    ])