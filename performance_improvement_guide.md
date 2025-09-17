# F1TENTH 로컬라이제이션 성능 향상 가이드

## 🔍 현재 성능 문제점

테스트 결과에서 발견된 주요 문제:
- **업데이트 주파수**: 13.46 Hz (목표: 20+ Hz)
- **위치 분산**: 1.51 m² (목표: <0.001 m²)
- **간격 편차**: ±246.4 ms (불안정)
- **위치 불확실성**: 0.114 m (목표: <0.1 m)

---

## 🚀 성능 향상 방안

### 1. AMCL 파라미터 최적화

#### A. 업데이트 주파수 향상
```yaml
# config/amcl_config.yaml 수정
update_min_d: 0.001     # 현재값 → 0.0005 (더 민감하게)
update_min_a: 0.001     # 현재값 → 0.0005 (더 민감하게)
resample_interval: 2    # 현재값 → 1 (더 자주 리샘플링)
gui_publish_rate: 10.0  # 현재값 → 20.0 (더 자주 발행)
```

#### B. 파티클 수 조정
```yaml
max_particles: 1000     # 현재값 → 500 (계산 부하 감소)
min_particles: 300      # 현재값 → 200 (계산 부하 감소)
laser_max_beams: 40     # 현재값 → 30 (계산 부하 감소)
```

#### C. 센서 모델 최적화
```yaml
z_hit: 0.5              # 현재값 → 0.8 (정확한 측정 신뢰도 증가)
z_rand: 0.5             # 현재값 → 0.2 (랜덤 노이즈 감소)
sigma_hit: 0.2          # 현재값 → 0.1 (더 정밀한 측정)
laser_likelihood_max_dist: 2.0  # 현재값 → 1.0 (범위 축소)
```

### 2. 시스템 레벨 최적화

#### A. CPU 성능 향상
```bash
# CPU 거버너를 performance 모드로 설정
sudo cpufreq-set -g performance

# 또는 모든 코어에 적용
for i in {0..3}; do
    sudo cpufreq-set -c $i -g performance
done
```

#### B. ROS 2 DDS 최적화
```bash
# ~/.bashrc에 추가
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastrtps_profile.xml

# DDS 프로필 설정으로 지연시간 최소화
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

#### C. 프로세스 우선순위 설정
```bash
# AMCL 프로세스 우선순위 높이기
sudo renice -10 $(pgrep amcl)

# 실시간 스케줄링 (주의: 시스템 불안정 가능)
sudo chrt -f 50 $(pgrep amcl)
```

### 3. 하드웨어 최적화

#### A. LiDAR 설정 확인
```bash
# LiDAR 스캔 주파수 확인
ros2 topic hz /scan

# 권장: 40Hz 이상
# 필요시 센서 설정에서 주파수 증가
```

#### B. 네트워크 최적화
```bash
# 네트워크 버퍼 크기 증가
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
sudo sysctl -w net.core.wmem_max=134217728
sudo sysctl -w net.core.wmem_default=134217728
```

### 4. 로컬라이제이션 환경 개선

#### A. 맵 품질 향상
- **해상도**: 0.05m → 0.03m (더 세밀한 맵)
- **특징점 추가**: 벽에 마커나 특이한 물체 배치
- **조명**: 일정한 조명 환경 유지

#### B. 초기 포즈 정확도
```bash
# RViz에서 2D Pose Estimate로 정확한 초기 위치 설정
# 또는 자동 초기화 개선
```

### 5. 실시간 모니터링 및 튜닝

#### A. 성능 모니터링 스크립트
```bash
# 실시간 성능 체크
watch -n 1 'ros2 topic hz /amcl_pose'
```

#### B. 파라미터 동적 조정
```bash
# 런타임에 파라미터 변경
ros2 param set /amcl max_particles 500
ros2 param set /amcl update_min_d 0.0005
```

---

## 📋 단계별 적용 가이드

### Phase 1: 즉시 적용 가능 (5분)
1. **CPU 성능 모드 설정**
   ```bash
   sudo cpufreq-set -g performance
   ```

2. **AMCL 파라미터 기본 최적화**
   ```bash
   ros2 param set /amcl max_particles 500
   ros2 param set /amcl update_min_d 0.0005
   ros2 param set /amcl gui_publish_rate 20.0
   ```

### Phase 2: 설정 파일 수정 (10분)
1. **amcl_config.yaml 수정**
2. **로컬라이제이션 재시작**
3. **성능 테스트 재실행**

### Phase 3: 시스템 최적화 (30분)
1. **DDS 설정 최적화**
2. **네트워크 버퍼 조정**
3. **프로세스 우선순위 설정**

### Phase 4: 환경 개선 (1시간+)
1. **맵 재생성 (더 높은 해상도)**
2. **환경 특징점 추가**
3. **하드웨어 점검**

---

## 🎯 예상 성능 향상

적용 후 예상 결과:
- **업데이트 주파수**: 13 Hz → 25+ Hz
- **위치 분산**: 1.5 m² → 0.0005 m²
- **간격 편차**: ±246 ms → ±20 ms
- **위치 불확실성**: 0.114 m → 0.05 m

---

## 🔧 즉시 적용할 명령어

```bash
# 1. CPU 성능 모드
sudo cpufreq-set -g performance

# 2. AMCL 파라미터 최적화
ros2 param set /amcl max_particles 500
ros2 param set /amcl min_particles 200
ros2 param set /amcl update_min_d 0.0005
ros2 param set /amcl update_min_a 0.0005
ros2 param set /amcl resample_interval 1
ros2 param set /amcl gui_publish_rate 20.0
ros2 param set /amcl laser_max_beams 30

# 3. 센서 모델 최적화
ros2 param set /amcl z_hit 0.8
ros2 param set /amcl z_rand 0.2
ros2 param set /amcl sigma_hit 0.1

# 4. 성능 테스트 재실행
python3 /home/f1/f1tenth_ws/src/f1tenth_slam_nav/scripts/localization_performance_test.py
```

이 가이드를 순서대로 적용하면 로컬라이제이션 성능이 크게 향상될 것입니다!