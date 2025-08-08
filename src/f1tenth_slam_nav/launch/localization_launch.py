#!/usr/bin/env python3

"""
F1TENTH Localization Launch File
Uses Nav2 AMCL for particle filter based localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value="/home/f1/f1tenth_ws/maps/gap_map_final.yaml",
        description="Full path to map yaml file to load"
    )
    
    amcl_config_file_arg = DeclareLaunchArgument(
        "amcl_config_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "amcl_config.yaml"]),
        description="Full path to AMCL configuration file"
    )
    
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start lifecycle nodes"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "localization.rviz"]),
        description="Path to RViz config file"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    amcl_config_file = LaunchConfiguration("amcl_config_file")
    autostart = LaunchConfiguration("autostart")
    rviz_config = LaunchConfiguration("rviz_config")

    # Include F1TENTH bringup (sensors, VESC, TF, etc.)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([f1tenth_stack_share, "launch", "bringup_launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # Map server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml_file
        }]
    )

    # AMCL (Adaptive Monte Carlo Localization) node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config_file,
            {"use_sim_time": use_sim_time}
        ]
    )

    # Lifecycle manager for localization nodes
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"]
        }]
    )

    # RViz2 node with localization config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_file_arg,
        amcl_config_file_arg,
        autostart_arg,
        rviz_config_arg,
        bringup_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        rviz_node
    ])