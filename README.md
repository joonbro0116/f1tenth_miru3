# 실행 명령어

bringup launch 파일 실행

```
rl
```

slam 파일
```
# mapping 모드 활성화
rsp
```

```
# localization 모드 활성화
rsl
```

```
# 맵 저장
ros2 run nav2_map_server map_saver_cli -f map_name
```

# SLAM
base_frame, odom_frame, map_frame: ROS 좌표계 설정
scan_topic: 라이다 데이터 토픽 (/scan)
resolution: 맵 해상도 (0.05m = 5cm 격자)

**맵 업데이트 조건**
map_update_interval: 맵 업데이트 주기 (1.5초)
minimum_travel_distance/heading: 맵 업데이트를 위한 최소 이동거리/회전각

**루프 클로저 (Loop Closure)**
do_loop_closing: 루프 클로저 활성화 - 같은 장소 재방문시 맵 보정
loop_search_maximum_distance: 루프 검색 최대 거리 (5m)
loop_match_minimum_chain_size: 루프 매칭을 위한 최소 체인 크기

**스캔 매칭**
correlation_search_space_dimension: 스캔 매칭 검색 범위
scan_buffer_size: 보관할 스캔 데이터 개수 (20개)
max_laser_range: 사용할 라이다 최대 거리 (12m)

**맵 품질**
occupied_threshold: 장애물로 판단하는 임계값 (0.7)
free_threshold: 자유공간으로 판단하는 임계값 (0.25)

**차량 설정**
footprint: F1TENTH 차량 크기 (0.5m x 0.3m 직사각형)
robot_radius: 차량 반지름 (0.3m)

#
