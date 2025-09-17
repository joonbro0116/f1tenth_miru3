# F1TENTH ROS 2 Quick Reference Guide

A comprehensive reference for F1TENTH autonomous racing development with ROS 2.

---

## Table of Contents
- [Quick Start Commands](#quick-start-commands)
- [SLAM & Mapping](#slam--mapping)
- [Localization](#localization)
- [Performance Testing](#performance-testing)
- [Path Planning](#path-planning)
- [Network Setup](#network-setup)
- [File Transfer](#file-transfer)
- [Model Predictive Control (MPC)](#model-predictive-control-mpc)

---

## Quick Start Commands

| Alias | Command | Purpose |
|-------|---------|---------|
| `rbl` | `ros2 launch f1tenth_stack bringup_launch.py` | Bringup launch file |
| `rsp` | `ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam` | SLAM Mapping mode |
| `rsl` | `ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization` | SLAM Localization mode |

---

## SLAM & Mapping

### Unified SLAM Package: `f1tenth_slam_nav`

Our integrated package combines SLAM and localization functionality into a single, easy-to-use interface.

#### Package Structure
```
f1tenth_slam_nav/
├── launch/
│   ├── slam_launch.py              # SLAM modes (slam_toolbox + cartographer)
│   ├── localization_launch.py      # AMCL localization
│   └── slam_nav_launch.py          # Unified launcher
├── config/
│   ├── slam_toolbox_config.yaml
│   ├── cartographer_config.lua
│   └── amcl_config.yaml
├── scripts/
│   └── localization_performance_test.py  # Performance evaluation
└── src/slam_nav_manager.py
```

#### Usage Examples

**SLAM Mapping:**
```bash
# Using slam_toolbox (recommended)
ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=slam_toolbox

# Using Cartographer
ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=cartographer
```

**Localization:**
```bash
ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization map_yaml_file:=/path/to/your/map.yaml
```

#### Service Commands

| Service | Purpose | Command |
|---------|---------|---------|
| Check current mode | Get active mode | `ros2 service call /get_current_mode std_srvs/srv/SetBool` |
| Switch mode | SLAM ↔ Localization | `ros2 service call /switch_slam_mode std_srvs/srv/SetBool "data: true"` |
| Save map | Export current map | `ros2 service call /save_current_map std_srvs/srv/Empty` |

#### Manual Map Saving
```bash
# Create directory first if needed
mkdir -p ~/f1/f1tenth_ws/maps/

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/f1/f1tenth_ws/maps/map_name
```

### SLAM Parameters (slam_toolbox)

#### Key Settings
| Parameter | Value | Description |
|-----------|-------|-------------|
| **Resolution** | `0.05 m` | 5 cm grid resolution |
| **Map Update Interval** | `1.5 s` | Update frequency |
| **Max Laser Range** | `12 m` | Maximum usable LiDAR range |
| **Loop Search Distance** | `5 m` | Loop closure search radius |
| **Occupied Threshold** | `0.70` | Cell ≥ 70% → obstacle |
| **Free Threshold** | `0.25` | Cell ≤ 25% → free space |

#### Vehicle Geometry
- **Footprint**: `0.50 m × 0.30 m` (rectangular)
- **Robot Radius**: `0.30 m`

### Map Data Recording

**Record map topics:**
```bash
ros2 bag record -o ~/f1/f1tenth_ws/bags/maps/map_topic_data /map /map_metadata
```

**Replay recorded data:**
```bash
ros2 bag play ~/f1/f1tenth_ws/bags/map_topic_data
```
> **Tip**: Open RViz2 and add the `/map` topic for visualization after replay.

---

## Localization

### AMCL (Adaptive Monte Carlo Localization)

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/path/to/your_map.yaml \
  params_file:=amcl_params.yaml
```

**Key Features:**
- Particle filter-based localization
- Adaptive particle count
- Real-time pose estimation
- Recovery behaviors

---

## Performance Testing

### Localization Performance Evaluation

Our custom performance testing tool evaluates localization quality in real-time.

#### Usage
```bash
# Start localization first
ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization

# Run performance test
python3 /home/f1/f1tenth_ws/src/f1tenth_slam_nav/scripts/localization_performance_test.py
```

#### Measured Metrics

| Metric | Good Range | Description |
|--------|------------|-------------|
| **Update Frequency** | `> 20 Hz` | Localization update rate |
| **Position Variance** | `< 0.001 m²` | Position stability |
| **Position Uncertainty** | `< 0.1 m` | AMCL covariance |
| **Orientation Uncertainty** | `< 10°` | Angular uncertainty |

#### Key Performance Indicators

**✅ Good Performance:**
- Update frequency ≥ 20 Hz
- Low position variance (< 1mm²)
- Position uncertainty < 10cm
- Stable measurements during stationary periods

**⚠️ Poor Performance:**
- Update frequency < 20 Hz
- High position variance
- Large uncertainty ellipse
- Jumping/drifting pose estimates

#### Real-time Output Example
```
📊 업데이트 성능:
   - 주파수: 25.3 Hz
   - 평균 간격: 39.5 ms
   - 간격 편차: ±2.1 ms

📍 위치 안정성 (최근 50개 샘플):
   - X축 분산: 0.000234 m²
   - Y축 분산: 0.000187 m²
   - 총 분산: 0.000421 m²

🎯 추정 불확실성:
   - X축 불확실성: ±0.045 m
   - Y축 불확실성: ±0.038 m
   - 방향 불확실성: ±3.2°
   - 총 위치 불확실성: ±0.059 m
```

---

## Path Planning

### Global Path Planning

**Launch map-based planner:**
```bash
# Default map
ros2 launch path_planner map_planner_launch.py

# Specific map file
ros2 launch path_planner map_planner_launch.py \
  map_yaml_file:=/home/f1/f1tenth_ws/maps/joon_map_2.yaml
```

**Features:**
- A* path planning
- Obstacle avoidance
- Smooth trajectory generation
- Real-time replanning

---

## Network Setup

### ROS 2 DDS Configuration

Add to **`~/.bashrc`** or **`~/.zshrc`**:
```bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Network Verification

**Test DDS communication:**
```bash
# Terminal 1 (Publisher)
ros2 run demo_nodes_cpp talker

# Terminal 2 (Subscriber)
ros2 run demo_nodes_cpp listener
```

> **Important Notes:**
> - All devices must be on the **same subnet**
> - **HY-WiFi is not supported** for ROS 2 communication
> - If you see "Hello World" messages, DDS discovery is working correctly

---

## File Transfer

### SCP Commands

**Copy file TO Jetson:**
```bash
scp /path/to/local/file user@<jetson_ip>:/remote/path/
```

**Copy file FROM Jetson:**
```bash
scp user@<jetson_ip>:/remote/path/file /local/destination/
```

> **Reference**: [Remote Copy (scp) Guide](https://eehoeskrap.tistory.com/543)

---

## Model Predictive Control (MPC)

### Algorithm Overview

Our MPC implementation uses **OSQP (Operator Splitting Quadratic Program)** solver for real-time optimal control.

#### MPC Process Flow
```
1. State Estimation → Current vehicle state [x, y, θ, v]
2. Reference Trajectory → Desired path with velocity profile
3. Optimization Problem → Solve using OSQP
4. Control Application → Apply first control input [a, δ]
5. Repeat → Receding horizon control
```

#### Vehicle Model
**Kinematic Bicycle Model:**
```
x_dot = v * cos(θ)           # Longitudinal dynamics
y_dot = v * sin(θ)           # Lateral dynamics
θ_dot = v * tan(δ) / L       # Yaw dynamics
v_dot = a                    # Acceleration dynamics
```

#### Optimization Objective
**Minimize:**
- **Tracking Error**: Distance from reference trajectory
- **Control Effort**: Minimize excessive control inputs
- **Smoothness**: Penalize rapid control changes
- **Terminal Cost**: Final state accuracy

#### OSQP Solver Features
- **Real-time Performance**: 10-50 Hz control frequency
- **Warm Starting**: Use previous solution as initial guess
- **Constraint Handling**: Vehicle limits and safety constraints
- **Numerical Stability**: Robust convergence properties

#### Key Parameters
| Parameter | Typical Value | Description |
|-----------|---------------|-------------|
| **Prediction Horizon** | `10-20 steps` | Future time steps to optimize |
| **Sample Time** | `0.1 s` | Discretization time step |
| **Max Steering** | `±24°` | Physical steering limit |
| **Max Acceleration** | `±3.0 m/s²` | Acceleration bounds |

#### Launch MPC Controller
```bash
# Basic MPC launch
ros2 launch mpc_controller mpc_launch.py

# With custom parameters
ros2 launch mpc_controller mpc_launch.py \
  horizon:=15 \
  sample_time:=0.1 \
  max_speed:=8.0
```

---

## Troubleshooting

### Common Issues

**Map saving fails:**
```bash
# Create directory first
mkdir -p ~/f1/f1tenth_ws/maps/
ros2 run nav2_map_server map_saver_cli -f ~/f1/f1tenth_ws/maps/your_map
```

**Localization fails to start:**
- Check if map file exists and is valid
- Verify map_yaml_file path is correct
- Ensure AMCL parameters are properly configured

**LiDAR connection issues:**
```bash
# Check LiDAR connection
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

**Performance testing shows poor results:**
- Check if robot is moving (static tests show high variance)
- Verify sufficient features in environment for localization
- Ensure proper initial pose estimation

---

## Getting Started

1. **Setup Environment**: Configure ROS 2 and network settings
2. **Create Map**: Use SLAM to map your environment
3. **Test Performance**: Run localization performance evaluation
4. **Localize**: Switch to localization mode with your map
5. **Plan Path**: Generate optimal racing trajectory
6. **Control**: Deploy MPC for high-performance tracking
7. **Race**: Enjoy autonomous racing!

---

## Additional Resources

- [F1TENTH Official Documentation](https://f1tenth.org/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [OSQP Solver Documentation](https://osqp.org/)
- [F1TENTH Course Materials](https://f1tenth-coursekit.readthedocs.io/)

---

*Happy Racing!*