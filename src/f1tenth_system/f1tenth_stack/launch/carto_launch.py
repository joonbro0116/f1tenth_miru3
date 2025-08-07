"""Cartographer mapping launch file for F1TENTH (ROS 2 Foxy)

- bringup_launch.py (sensor & drive stack) is included as‑is.
- Starts Cartographer SLAM (2‑D) and occupancy‑grid publisher.

Usage:
    ros2 launch f1tenth_stack cartographer_mapping_launch.py  \
        use_sim_time:=false                # true if Gazebo / rosbag
        config_basename:=f110_2d.lua       # 다른 lua 쓰려면 덮어쓰기
        config_dir:=$PWD/src/…/config      # (옵션) lua 디렉터리 지정

After mapping, save the state:
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState  \ 
        "{filename: \"$HOME/maps/f110_map.pbstream\"}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("f1tenth_stack")

    # ────────────────────────────── Launch arguments ─────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (/clock) time if true",
    )

    config_basename_arg = DeclareLaunchArgument(
        "config_basename", default_value="f110_2d.lua",
        description="Cartographer Lua configuration file name",
    )

    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value=os.path.join(pkg_share, "config"),  # lua 파일 실제 위치
        description="Directory that contains Cartographer Lua config files",
    )

    # ─────────────────── Bring‑up (sensors, VESC, TF, etc.) ─────────────────────
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "bringup_launch.py")
        )
    )

    # ─────────────────────── Cartographer SLAM node ─────────────────────────────
    carto_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory", LaunchConfiguration("config_dir"),
            "-configuration_basename", LaunchConfiguration("config_basename"),
        ],
        remappings=[("/scan", "scan")],
    )

    # ───────────────────── OccupancyGrid publisher (/map) ───────────────────────
    occ_grid_node = Node(
        package="cartographer_ros",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "resolution": 0.05,
            "publish_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_basename_arg,
        config_dir_arg,
        bringup_launch,
        carto_node,
        occ_grid_node,
    ])
