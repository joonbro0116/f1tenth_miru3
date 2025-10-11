# 🚗 F1TENTH-MIRU3-Object_Detection

> **Developed and maintained by [@shdragron](https://github.com/shdragron)**  
> F1TENTH-MIRU3 플랫폼에서 **주행 중 장애물을 인지**하기 위한 ROS2 기반 오브젝트 감지 프로젝트입니다.

---

## 📋 Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [Project Structure](#-project-structure)
- [Usage](#-usage)
- [Configuration](#-configuration)

---
![Simulation Result]("./assets/2d lidar od.png")

---
## 🧠 Overview
이 프로젝트는 **F1TENTH-MIRU3 자율주행 차량**의 센서 데이터를 이용해 **도로 경계(Inner/Outer Bound)와 장애물을 탐지**하는 시스템입니다.  
ROS2, Python, OpenCV를 기반으로 작성되었으며, 시뮬레이션 환경(f1tenth_gym)에서도 동일한 알고리즘을 실행할 수 있습니다.

> **Keywords:** ROS2, 2D LiDAR, Pure Pursuit, Obstacle Detection

---

## ✨ Features
- **👁️ Perception** — 2D LiDAR 센서를 활용한 실시간 장애물 감지  
- **🧩 Planning** — Pure Pursuit 기반의 Global Path 추종  
- **📊 Visualization** — RViz2를 통한 실시간 시각화 및 Bound 표시  

---

## 📂 Project Structure
```bash
f1tenth_miru3/
├── bounds_out/                 # Inner / Outer Bound CSV 파일
│   └── *.csv
├── build/
├── config/
├── install/
├── latest_build/
├── log/
├── maps/                       # 지도 (PGM, YAML)
│   ├── track1.pgm
│   └── track1.yaml
├── pgm_to_bounds.py            # 맵으로부터 Bound(Inner, Outer) 생성 스크립트
└── src/
    ├── bound_obstacle_detector/  # LiDAR 기반 장애물 감지 노드
    ├── centerline_planner/       # Global Path (Centerline) Planner
    ├── csv_obstacle/             # Bound 시각화 및 데이터 로드
    └── f1tenth_gym_ros/          # 시뮬레이터 통신 노드 (gym_bridge)
```

---

## 🚀 Usage

### 🧩 Step 1. Global Map에서 Inner / Outer Bound 생성
```bash
python3 pgm_to_bounds.py   --pgm /home/moon/sim_ws/maps/0927_1.pgm   --yaml /home/moon/sim_ws/maps/0927_1.yaml   --step_px 5   --min_perimeter 200   --clearance_m 0.3   --outdir ./bounds_out   --save_world
```

> ✅ 출력:  
> - `bounds_out/inner_*.csv`  
> - `bounds_out/outer_*.csv`  
> - (옵션) `.world` 파일 (시뮬레이터용)

---

### 🏎️ Step 2. 실행 순서

#### (0) 사전 준비
- **Global Path CSV 지정:**  
  `src/centerline_planner/launch/centerline_follow.launch.py`  
  → `csv_path` 항목 수정

- **Inner / Outer Bound CSV 지정:**  
  `src/csv_obstacle/launch/csv_obstacle.launch.py`  
  → `outer_csv`, `inner_csv` 항목 수정

---

#### (1) 시뮬레이터 실행
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

#### (2) Pure Pursuit 주행 시작
```bash
ros2 launch centerline_planner centerline_follow.launch.py
```

#### (3) Bound 시각화
```bash
ros2 launch csv_obstacle csv_obstacle.launch.py
```

#### (4) 장애물 감지 실행
```bash
ros2 launch bound_obstacle_detector bound_detector.launch.py
```

---

## 📈 Results
| 모듈 | 기능 | 시각화 | 상태 |
|------|------|--------|------|
| Bound Generator | Map → Inner/Outer Bound 생성 | ✅ | 완료 |
| Centerline Planner | Global Path 주행 | ✅ | 완료 |
| Obstacle Detector | LiDAR 감지 및 경계 내 물체 판정 | ✅ | 완료 |

---
