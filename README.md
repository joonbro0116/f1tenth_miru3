# ROS 2 F1TENTH Quick Reference

---

## 1. Execution Shortcuts

| Alias | Purpose                          |
| ----- | -------------------------------- |
| `rbl` | Bringup launch file             |
| `rsp` | SLAM Mapping mode      |
| `rsl` | SLAM Localization mode |

 ### Save map

 ```bash
 ros2 run nav2_map_server map_saver_cli -f ~/f1/f1tenth_ws/maps/map_name
 ```

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





 📦 새로운 통합 패키지: f1tenth_slam_nav

  🎯 리팩토링 목적

  기존에 여러 패키지와 파일에 분산되어 있던 SLAM과 localization 기능을 하나의 통합된 패키지로 정리했습니다.

  🏗️ 패키지 구조

  f1tenth_slam_nav/
  ├── launch/                          # 런치 파일들
  │   ├── slam_launch.py              # SLAM 전용 (slam_toolbox + cartographer)
  │   ├── localization_launch.py      # Localization 전용 (AMCL)
  │   └── slam_nav_launch.py          # 통합 런치 (모드 선택)
  ├── config/                          # 설정 파일들
  │   ├── slam_toolbox_config.yaml    # slam_toolbox 동기 모드 설정
  │   ├── slam_toolbox_async_config.yaml # slam_toolbox 비동기 모드 설정
  │   ├── cartographer_config.lua     # Cartographer 설정
  │   └── amcl_config.yaml            # AMCL localization 설정
  ├── src/
  │   └── slam_nav_manager.py         # SLAM/Localization 관리 노드
  ├── f1tenth_slam_nav/               # Python 패키지
  │   ├── __init__.py
  │   └── slam_nav_manager.py
  └── package.xml, CMakeLists.txt, setup.py

  🚀 주요 기능

  1. 통합 런치 시스템

  - slam_nav_launch.py: 메인 엔트리 포인트
  # SLAM 모드
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=slam_toolbox

  # Localization 모드  
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization map_yaml_file:=/path/to/map.yaml

  2. 다중 SLAM 백엔드 지원

  - slam_toolbox: 동기/비동기 모드 지원
  - cartographer: Google의 실시간 SLAM
  - 런타임에 백엔드 선택 가능

  3. 스마트 모드 관리

  - slam_nav_manager.py가 SLAM ↔ Localization 동적 전환 제공
  - ROS2 서비스를 통한 모드 변경
  - 자동 맵 저장 기능

  🔧 통합된 기존 기능들

  기존에서 가져온 것들:

  1. f1tenth_stack의 carto_launch.py → slam_launch.py에 통합
  2. f1tenth_stack의 amcl_localization_launch.py → localization_launch.py로 개선
  3. joon_launch.py의 slam_toolbox → slam_launch.py에 통합
  4. 분산된 설정 파일들 → config/ 디렉토리로 정리

  💡 주요 개선점

  1. 사용성 개선

  # 기존: 여러 런치 파일을 개별 실행
  ros2 launch f1tenth_stack carto_launch.py
  ros2 launch f1tenth_stack amcl_localization_launch.py

  # 새로운 방식: 하나의 인터페이스
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam

  2. 동적 모드 전환

  # 매핑 완료 후 바로 localization으로 전환
  ros2 service call /switch_slam_mode std_srvs/srv/SetBool "data: true"

  # 맵 저장
  ros2 service call /save_current_map std_srvs/srv/Empty

  3. 설정 관리 통합

  - F1TENTH에 최적화된 파라미터들을 하나의 위치에서 관리
  - slam_toolbox, cartographer, AMCL 설정을 각각 독립적으로 튜닝 가능

  🎮 사용법

  SLAM (매핑) 모드:

  # slam_toolbox 사용
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=slam_toolbox

  # cartographer 사용  
  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=slam slam_backend:=cartographer

  Localization 모드:

  ros2 launch f1tenth_slam_nav slam_nav_launch.py mode:=localization map_yaml_file:=/path/to/your/map.yaml

  관리 서비스:

  # 현재 모드 확인
  ros2 service call /get_current_mode std_srvs/srv/SetBool

  # 모드 전환 (SLAM → Localization)
  ros2 service call /switch_slam_mode std_srvs/srv/SetBool "data: true"

  # 맵 저장
  ros2 service call /save_current_map std_srvs/srv/Empty

  🔄 기존 코드와의 호환성

  - F1TENTH stack의 bringup_launch.py를 그대로 활용
  - 기존 설정 파일들을 보존하면서 개선
  - 기존 워크플로우를 방해하지 않음

  이제 SLAM과 localization을 하나의 통합된 인터페이스로 편리하게 사용하실 수 있습니다!

