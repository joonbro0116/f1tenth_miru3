# ROS 2 F1TENTH Quick Reference

---

## 1. Execution Shortcuts

| Alias | Purpose                          | Underlying Command                                                                                             |
| ----- | -------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `rbl` | Bringup launch file             | `ros2 launch f1_stack bringup_launch.py`                                                                       |
| `rsp` | SLAM Mapping mode      | `ros2 launch slam_toolbox online_async_launch.py \`<br/>`  params_file:=~/f1/f1tenth_ws/config/mapper.yaml`    |
| `rsl` | SLAM Localization mode | `ros2 launch slam_toolbox localization_launch.py \`<br/>`  params_file:=~/f1/f1tenth_ws/config/localizer.yaml` |

> **Save map**
>
> ```bash
> ros2 run nav2_map_server map_saver_cli -f ~/f1/f1tenth_ws/maps/map_name
> ```

---

## 2. SLAM Parameters (online\_async\_launch)

### Frames & Topics

* **Base frame**: `base_frame`
* **Odometry frame**: `odom_frame`
* **Map frame**: `map_frame`
* **Scan topic**: `/scan`
* **Resolution**: `0.05` m (5 cm grid)

### Map Update Conditions

* `map_update_interval`: **1.5 s**
* `minimum_travel_distance`: minimum linear displacement before update
* `minimum_travel_heading`: minimum rotation (rad) before update

### Loop Closure

* `do_loop_closing`: enable/disable loop closing
* `loop_search_maximum_distance`: **5 m**
* `loop_match_minimum_chain_size`: minimum chain length for a match

### Scan Matching

* `correlation_search_space_dimension`: search window size
* `scan_buffer_size`: **20** scans kept in buffer
* `max_laser_range`: **12 m** maximum usable range

### Map Quality

* `occupied_threshold`: **0.70** (cells ≥ 70 % → obstacle)
* `free_threshold`: **0.25** (cells ≤ 25 % → free space)

### Vehicle Geometry

* `footprint`: rectangular **0.50 m × 0.30 m**
* `robot_radius`: **0.30 m**

---

## 3. Cartographer (Alternative SLAM)

```bash
ros2 launch f1_stack carto_launch.py
```

---

## 4. Map Topic Bagging

### Record

```bash
ros2 bag record -o ~/f1/f1tenth_ws/bags/maps/map_topic_data /map /map_metadata
```

### Replay

```bash
ros2 bag play ~/f1/f1tenth_ws/bags/map_topic_data
```

After replay, open **RViz2** and add the */map* topic for visualization.

---

## 5. Localization (AMCL / Nav2)

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/path/to/your_map.yaml \
  params_file:=amcl_params.yaml
```

---

## 6. ROS 2 Network Setup

Add these lines to **\~/.bashrc** or **\~/.zshrc**:

```bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

> **Notes**
>
> * All devices must be on the **same subnet**. *HY‑WiFi is not supported.*
> * Verify with the classic *talker / listener* demo:
>
>   ```bash
>   # Terminal 1 (host A)
>   ros2 run demo_nodes_cpp talker
>
>   # Terminal 2 (host B)
>   ros2 run demo_nodes_cpp listener
>   ```
>
>   If you see “Hello World” messages, DDS discovery is working.

---

## 7. SCP Cheat‑Sheet

* **Copy a file *to* the Jetson**

  ```bash
  scp /path/to/local/file user@<jetson_ip>:/remote/path/
  ```
* **Copy a file *from* the Jetson**

  ```bash
  scp user@<jetson_ip>:/remote/path/file /local/destination/
  ```
**[원격 복사(scp)](https://eehoeskrap.tistory.com/543)**
---
