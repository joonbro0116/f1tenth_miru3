# Forza MAP Controller (ROS2 Port)

이 패키지는 ForzaETH race_stack의 MAP 컨트롤러를 ROS2 환경에 맞게 옮긴 것입니다. 원본 코드는 <https://github.com/ForzaETH/race_stack> 를 참고했습니다.

## 주요 기능

- **경로/파라미터 위치**
  - 경로 CSV: `src/forza_map_controller/config/forza_map_params.yaml` 의 `csv_file_path` 로 지정 (기본 경로는 `maps_racelines_/raceline/`).
  - L1, 속도 관련 파라미터: 같은 YAML 파일(`t_clip_min`, `t_clip_max`, `m_l1`, `q_l1`, `speed_lookahead`, `lat_err_coeff` 등)을 수정하여 적용합니다.

- **웨이포인트 + 속도 프로파일**  
  - `config/forza_map_params.yaml` 의 `csv_file_path` 로 지정한 CSV(`maps_racelines_/raceline/0927_speed_5.csv`)를 읽어 경로와 목표 속도를 사용합니다.
  - CSV 포맷은 `x, y, target_speed` (m, m/s).
- **중복 좌표 처리**  
  - 연속된 동일 좌표는 로딩 단계에서 건너뛰고, 마지막 점이 시작점과 같으면 내부 배열에서는 제거하지만 시각화를 위해 Path 마지막에 다시 추가합니다.
- **Frenet 변환**  
  - `FrenetConverter` 를 이용해 (x, y)를 (s, d)로 변환합니다. L1 포인트는 s 축에서 길이만큼 이동하여 찾으며 트랙이 닫혀 있어도 연속적으로 이어집니다.
- **L1 룩어헤드**  
  - 기본식: `L1 = q_l1 + m_l1 * speed`.  
  - 현재 파라미터: `t_clip_min = 0.8`, `t_clip_max = 1.5`, `m_l1 = 0.6`, `q_l1 = -0.18`.  
  - 룩어헤드 포즈는 차량 위치에서 L1 포인트까지 벡터 방향으로 계산해 `/forza_map/lookahead_point` 로 퍼블리시하며, 길이는 `/forza_map/lookahead_distance` 로 `Float32` 메시지로 발행합니다.
- **속도 명령**  
  - raceline 속도 프로파일을 그대로 사용(`lat_err_coeff` 감속 로직 비활성화).  
  - `AckermannDriveStamped` 에 속도만 채워 `/drive` 토픽으로 발행합니다. 실제 속도 추종은 하위 레이어(VESC 등)에서 처리해야 합니다.
- **시각화 토픽**  
  - `/forza_map/path`: 전체 경로(Path).  
  - `/forza_map/waypoints_pose`: 모든 웨이포인트의 PoseArray(위치+헤딩).  
  - `/forza_map/lookahead_point`, `/forza_map/lookahead_distance`: 현재 L1 포즈와 거리.

## 실행 방법 (짐 ROS 시뮬레이터 포함)

```bash
# 1. 시뮬레이터 워크스페이스 환경
source /home/sh/f1tenth_ws/install/setup.zsh

# 2. MAP 컨트롤러 워크스페이스
cd /home/sh/projects/f1tenth_miru3
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# 3. 시뮬레이터 + MAP 컨트롤러 동시 실행
ros2 launch forza_map_controller forza_map_with_gym.launch.py use_sim_time:=false
```

## RViz 디스플레이 권장 목록

- `Path` → `/forza_map/path`
- `PoseArray` → `/forza_map/waypoints_pose`
- `Pose` (Arrow) → `/forza_map/lookahead_point`
- `Text` → `/forza_map/lookahead_distance`

## 참고 식

- **룩어헤드 길이**  
  `L1 = clip(q_l1 + m_l1 * v, [t_clip_min, t_clip_max])`
- **스티어링**  
  `η = asin([-sin(ψ), cos(ψ)]·L1_vector/|L1_vector|)`  
  `lat_acc = 2 * v^2 / L1 * sin(η)`  
  `steer = Lookup(lat_acc, v)`
- **속도 명령**  
  `v_cmd = speed_profile(s)`

## 주의 사항

- 랩 타임/카운트는 기본적으로 퍼블리시하지 않습니다.(필요 시 `controller_manager` 에서 프레넷 s 로 계산하여 토픽을 추가하세요)
- 실제 차량 운용 시 `/drive.speed` 값을 추종할 종방향 제어기(VESC PID 등)를 별도로 설정해야 합니다.

## 원본 race_stack 대비 이식 내용

- `controller_manager.py`, `map_controller.py`, `steering_lookup.py` 는 race_stack/controller/map/src(및 steering_lookup) 의 동일 파일과 비교(diff)하여 ROS2 API에 맞게 수정했습니다.
- 주요 변경 사항:
  - `rospy` 대신 `rclpy`, 메시지 타입, QoS 설정 등 ROS2 포팅.
  - 경로 로딩 시 중복 좌표 처리, PoseArray 퍼블리시 등 시각화 보조 기능 추가.
  - L1 계산 로직, 속도 계산 등 핵심 알고리즘은 race_stack 원본과 동일함을 `git diff race_stack/controller/map/src/MAP_Controller.py` 와 비교해 검증했습니다.
  - Steering lookup 테이블 로딩 역시 race_stack/system_identification/steering_lookup/src/lookup_steer_angle.py 와 동일한 수식을 사용하며, ROS2 패키지 경로(Ament index)만 다릅니다.
  - 필요 시 `race_stack` 폴더의 원본 코드를 참고하여 변경 내역을 추적할 수 있습니다.
