# MAP Controller (ROS 2)

This package hosts the ROS 2 port of the Forza MAP controller for F1TENTH. It exposes a single `map_controller_manager` component/node that replicates the behaviour of the original Python implementation while fitting the ROS 2 workflow.

## Node Overview

| Entity | Topic | Type | Notes |
| --- | --- | --- | --- |
| Subscription | `/odom` (configurable via `state_topic`) | `nav_msgs/msgs/Odometry` | Vehicle pose, twist and covariance. Used for speed, yaw and acceleration estimation. |
| Subscription | `/local_waypoints` (configurable via `path_topic`) | `nav_msgs/msg/Path` | Optional. Live update of the local reference path. |
| Publication | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Steering, speed and acceleration commands. |
| Publication | `/pure_pursuit_path` | `nav_msgs/msg/Path` | Visualisation of the currently active path. |
| Publication | `/lookahead_point` | `geometry_msgs/msg/PoseStamped` | Visualisation of the computed L1 lookahead point. |

If a CSV path is provided the node will preload the waypoints from disk; otherwise it expects `/local_waypoints` to supply the path at runtime.

## Key Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `csv_file_path` | empty | Path to a raceline CSV (same format as Pure Pursuit). Used when no path topic is supplied. Header lines are ignored. |
| `state_topic` | `/odom` | Odometry topic used to update `VehicleState`. |
| `path_topic` | `/local_waypoints` | Path topic to listen to. Leave empty to rely exclusively on the CSV. |
| `map_frame_id` | `map` | Frame assigned to visualisation messages. |
| `publish_visualisation` | `true` | Enable/disable publishing of `/pure_pursuit_path` and `/lookahead_point`. |
| `steering_lut_name` | `default` | Name of the steering lookup table CSV located in `resources/lut/`. |
| `driving_mode` | `racing` | Either `racing` or `trailing`; switches speed/steering handling. |
| `t_clip_min`, `t_clip_max`, … | (see `MapControllerParams`) | Forza MAP tuning knobs that mirror the original Python defaults. |

All parameters can be overridden through launch files or CLI arguments, e.g.:

```bash
ros2 run map_controller map_controller_main \
  --ros-args \
  -p csv_file_path:=/home/f1/maps_racelines/raceline/fuck_jg_1_mintime_vmax3ms.csv \
  -p publish_visualisation:=true \
  -p steering_lut_name:=NUC2_pacejka
```

## CSV Requirements

The loader accepts the same CSV structure used by the Pure Pursuit package:

```
x, y[, target_speed][, frenet_s][, frenet_d][, curvature][, heading][, accel]
```

- Columns beyond `x` and `y` are optional. Missing curvature/heading data is reconstructed automatically.
- Header rows or whitespace are ignored.

Place additional LUT files in `resources/lut/<name>_lookup_table.csv` and reference them via `steering_lut_name`.

## Runtime Notes

1. Source the ROS 2 Humble environment and the workspace build:  
   `source /opt/ros/humble/setup.zsh`  
   `source install/setup.zsh`
2. Ensure `/odom` is being published (e.g. simulator, bag playback, or actual car).
3. Provide a path via CSV or `/local_waypoints`.
4. Run the node with `ros2 run map_controller map_controller_main` or load it as a component.
5. Observe `/drive`, `/pure_pursuit_path`, and `/lookahead_point` for command/visualisation data.

## Repository Status

All Forza MAP features now live in C++, including:
- Steering LUT lookup with CSV resources.
- Trailing mode support (PID loop on opponent gap).
- L1 lookahead computation identical to the Python logic.
- Visualisation outputs compatible with existing RViz setups.

The original `pure_pursuit` node remains available; choose whichever controller fits your scenario.
