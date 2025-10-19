MAP Controller (ROS2, miru3) — Parameters, Flow, and Tuning

개요
- 목적: map_controller 패키지 내 신호 흐름(종/횡), 설정 파라미터, 튜닝 요령을 하나의 문서로 정리합니다.
- 구성: 종방향(속도) 제어와 횡방향(조향) 제어를 분리해 설명하고, 각 단계에서 어떤 변수/함수/토픽을 거치는지와 클램핑(제한) 지점을 명시합니다.

설정 파일과 위치
- 파라미터 YAML(권장): `f1tenth_miru3/src/map_controller/config/map_controller_params.yaml`
- 런치 파일: `f1tenth_miru3/src/map_controller/launch/map_controller.launch.py`
- 런타임 설정: `ros2 param set /map_controller <name> <value>`
- 레이싱 라인 CSV(필수): `csv_file_path`로 지정. 포맷: x, y, speed
- 스티어링 LUT CSV(선택): `f1tenth_miru3/src/map_controller/config/steering_lut/<name>_lookup_table.csv`

핵심 파라미터(요약)
- 공통/시스템
  - `csv_file_path`: 레이싱 라인 CSV 절대경로
  - `loop_rate_hz`(기본 80.0): 제어주기(Hz)
  - 토픽: `odom_topic`, `amcl_topic`, `imu_topic`, `drive_topic`
- 스티어링 LUT/룩어헤드
  - `steering_lut`: LUT 접두 이름(예: `NUC2_pacejka`); 빈 값이면 기구학 폴백 사용(휠베이스 0.33 m)
  - `speed_lookahead`(기본 0.20 s): 속도/FF 샘플용 시간 룩어헤드
  - `speed_lookahead_for_steer`(기본 0.0 s): 조향 계산용 시간 룩어헤드
  - `ff_lookahead_time`(기본 0.0 s): a_ff 샘플용 전용 룩어헤드(0.0이면 FF 룩어헤드 미사용)
- L1 컨트롤(횡방향)
  - `t_clip_min`, `t_clip_max`, `m_l1`, `q_l1`
  - `lat_err_coeff`(기본 1.0): 횡오차 기반 속도 감소 계수(스케일링 토글이 켜졌을 때 사용)
  - 스케일링 토글(기본 OFF): `use_lat_err_speed_scale=false`, `use_heading_speed_scale=false`
  - 조향 스케일링: `acc_scaler_for_steer`, `dec_scaler_for_steer`, `start_scale_speed`, `end_scale_speed`, `downscale_factor`
- 종방향(속도) PID + Feedforward
  - FF 토글: `use_ff=true`(false면 FF 완전 차단)
  - PID: `long_kp=0.4`, `long_ki=0.0`, `long_kd=0.01`(80 Hz 기준 권장 시작값)
  - 출력 한계/리밋: `long_min_speed=0.0`, `long_max_speed=15.0`, `long_ff_a_limit=2.0`
  - 레이트 리밋: `long_a_max=1.2`, `long_d_max=2.0` (m/s^2)

추천 기본값/가이드
- 주기(권장): 80 Hz. 더 높이면 지연 감소/부드러움 증가, D항 민감도↑(더 작게 설정 권장)
- FF: `use_ff=true`, `long_ff_acc_gain≈1.0`, `long_ff_a_limit`로 스파이크 억제
- PID: P=0.4부터 시작(±0.05 미세조정), D=0.01(80 Hz), I=0.0(필요 시 0.005–0.01)
- 레이트 리밋: 트랙/그립에 맞춰 `long_a_max/d_max` 조정(과격하면 슬립 위험↑)
- 스케일링 토글: 초기에는 OFF로 단순화. 필요 시 lat/heading 스케일 ON

종방향(속도) — 신호 흐름(변수/함수/토픽)
- 입력 계측
  - `/odom`(Odometry) → `controller_manager.speed_now` 저장
  - 파일: `map_controller/controller_manager.py`
- MAP 참조 생성
  - `controller_manager.control_loop()` → `map_controller.main_loop()` → `map_controller.calc_speed_command()`
  - `global_speed = waypoint_array_in_map[idx_la_position, 2]`
  - (옵션) `use_lat_err_speed_scale=true` → `speed_adjust_lat_err()`
  - (옵션) `use_heading_speed_scale=true` → `speed_adjust_heading()`
  - `speed = max(speed_command, 0)`(비음수 하한)
  - 파일: `map_controller/map_controller/map_controller.py`
- PID + FF
  - 참조/계측: `v_ref = max(speed, 0)`, `v_meas = max(self.speed_now, 0)`
  - FF(a_ff): 웨이포인트 로딩 시 v·dv/ds 사전계산(`waypoints[:,7]`), s-룩어헤드로 조회(`ff_lookahead_time` 사용) → `±long_ff_a_limit`로 클램프
  - 제어: `LongitudinalController.step(v_ref, v_meas, a_ff, use_ff)`
    - 내부 클램프: `long_min_speed ≤ cmd ≤ long_max_speed`
  - 파일: `map_controller/map_controller/controller_manager.py`
- 레이트 리밋(최종 변화율 제한)
  - 범위: `v ∈ [v_prev − long_d_max·dt, v_prev + long_a_max·dt]`
  - 파일: `map_controller/map_controller/controller_manager.py`
- 퍼블리시
  - `/drive`(AckermannDriveStamped)로 최종 속도 `ack_msg.drive.speed` 전송
  - 파일: `map_controller/map_controller/controller_manager.py`

횡방향(조향) — 신호 흐름(변수/함수/토픽)
- L1 포인트/거리
  - 최근접 웨이포인트 찾기 → 곡률 평균 계산(정보용)
  - `L1_distance = q_l1 + m_l1 * speed_now` → `[t_clip_min, t_clip_max]`로 클램프(하한은 횡오차 기반)
  - `L1_point = waypoint_at_distance_before_car(L1_distance, ...)`
  - 파일: `map_controller/map_controller/map_controller.py`
- 조향각 계산
  - 조향 룩어헤드 속도: `speed_lookahead_for_steer`로 미리 위치 전파 후 속도 샘플
  - `eta` 계산 → `lat_acc = 2*v^2/L1 * sin(eta)`
  - LUT 조향: `LookupSteerAngle.lookup_steer_angle(lat_acc, v)`(없으면 기구학 폴백)
  - 스케일링: `acc_scaling()`(가속/감속), `speed_steer_scaling()`(고속 다운스케일)
  - 변화율 제한: 루프당 ±0.4 rad로 클립
  - 파일: `map_controller/map_controller/map_controller.py`, `map_controller/map_controller/steering_lookup.py`
- 퍼블리시
  - `/drive`(AckermannDriveStamped)로 최종 조향 `ack_msg.drive.steering_angle` 전송
  - 파일: `map_controller/map_controller/controller_manager.py`

스케일링/클램프 정리(속도)
- 음수 방지: `speed = max(speed_command, 0)`
- FF 스파이크: `±long_ff_a_limit`
- PID 출력 한계: `long_min_speed`, `long_max_speed`
- 변화율 제한: `long_a_max`, `long_d_max`(dt 반영)
- (옵션) 횡오차/곡률/헤딩 스케일: `use_lat_err_speed_scale`, `use_heading_speed_scale`

CSV/LUT 준비
- CSV(필수): 레이싱 라인 v(s) 품질이 좋을수록 FF가 유리. 급격한 점프는 dv/ds 스파이크 유발
- LUT(선택): `steering_lut`로 모델 접두 지정(NUC2_pacejka 등). 없으면 기구학 폴백 사용

튜닝 순서 제안(요약)
- FF 먼저: `use_ff=true`, `long_ff_acc_gain≈1.0`으로 가감속 크기 맞추기
- PID: P부터(0.4±0.05) → 필요 시 D(0.01±0.003) → 마지막에 I(0.005–0.01)
- 레이트 리밋: 느리면 a_max/d_max↑, 거칠면 ↓
- 스케일링: 안정성 필요 시 lat/heading 스케일을 켜서 참조 속도 감소

첫 주행 튜닝 체크리스트(권장 순서)
- PD부터 체크(종방향 기본 응답)
  - 파라미터: `long_kp`, `long_kd` (I는 초기 0.0 유지)
  - 설명: `long_kp`는 속도 오차에 비례한 보정량, `long_kd`는 오차 변화율(노이즈에 민감)을 감쇠시키는 항목입니다.
  - 이유: 목표 추종의 과/저추종과 진동을 가장 크게 좌우하는 항목
  - 초기값(80 Hz): `long_kp=0.4`, `long_kd=0.01`
  - 조정: `long_kp`±0.05, `long_kd` 0.008–0.013 범위 미세 조정
  - 관측: `/drive.speed` vs `/odom`, 오버슈트/진동/정착시간
  - 기대효과: 과/저추종 감소, 응답성/안정성 균형 확보

- 레이트 리밋(명령 변화율 제한)
  - 파라미터: `long_a_max`(가속), `long_d_max`(감속)
  - 설명: 이전 출력 대비 이번 출력의 최대 증가/감소율을 m/s^2 단위로 제한합니다(출력 저크/슬립 방지용 램프).
  - 이유: 토크 단계/슬립/거친 출력(저크) 방지. 실차 안정성에 직접 영향
  - 초기값: `long_a_max=1.2`, `long_d_max=2.0`
  - 조정: 0.2–0.5 단위 증감(느리면 ↑, 거칠면 ↓). 과도하면 슬립 위험
  - 기대효과: 저크 감소/안정성 향상(↓) 또는 응답성 개선(↑)

- FF 클램프(a_ff 스파이크 억제)
  - 파라미터: `long_ff_a_limit`
  - 설명: 프로파일에서 샘플링한 `a_ff = v·dv/ds`의 절대값 상한(m/s^2). 노치/스파이크를 물리 한계 안으로 제한합니다.
  - 이유: v(s) 급변/희소 샘플로 인한 dv/ds 스파이크 억제
  - 권장 범위: 1.5–2.5 m/s^2 (기본 2.0)
  - 기대효과: 과도 피크 제거, PID 부담 완화

- Feedforward 토글/게인
  - 파라미터: `use_ff`, `long_ff_acc_gain`, `long_ff_vel_gain`
  - 설명: `use_ff`는 FF 경로 사용 여부, `long_ff_acc_gain`은 a_ff 스케일, `long_ff_vel_gain`은 v_ref 비례항(바이어스/steady-state 보정용)입니다.
  - 이유: 모델(프로파일) 기반 예측으로 PID 부담을 줄임
  - 초기값: `use_ff=true`, `long_ff_acc_gain≈1.0`, `long_ff_vel_gain=0.0`
  - 조정: 프로파일 가감속 과하면 0.95↓, 부족하면 1.05↑; vel_gain은 0.02–0.05 소량
  - 기대효과: 응답 향상, steady-state 편차 감소(vel_gain 소량 시)

- FF 룩어헤드(지연 보상)
  - 파라미터: `ff_lookahead_time`
  - 설명: FF 전용 시간 룩어헤드(초). s_now + vs·ff_lookahead_time 위치에서 a_ff를 샘플링해 지연을 보상합니다.
  - 이유: 제어·액추에이터 지연 보상(FF만 선행)
  - 초기값: 0.0 s(OFF). 필요 시 0.10–0.15 s에서 시도
  - 주의: 과도하면 오버슈트 경향. 센서율/지연에 맞춰 소량 사용
  - 기대효과: 코너 진입/탈출 구간 추종 타이밍 보정

- 속도 참조 룩어헤드(옵션)
  - 파라미터: `speed_lookahead`
  - 설명: 속도 참조(global_speed) 샘플링 위치를 v·T만큼 앞당기는 시간 룩어헤드(초). 참조 자체의 지연 보상.
  - 이유: 속도 참조 자체를 앞당겨 지연 보상(FF와 별개)
  - 초기값: 0.25 s. 지연이 작거나 오버슈트 경향이면 0.15 또는 0.0으로 시험
  - 기대효과: 속도 타이밍 정합 향상(과도하면 과선행 → 오버슈트)

- 스케일링 토글(안정성 확보용)
  - 파라미터: `use_lat_err_speed_scale`, `use_heading_speed_scale` (기본 false)
  - 설명: 횡오차/곡률(lat)·헤딩 오차(heading)가 클 때 v_ref를 보수적으로 낮추는 스위치입니다(`lat_err_coeff`로 강도 조절).
  - 이유: 횡오차/헤딩 오차가 클 때 참조 속도를 보수적으로 낮춤
  - 사용 시: `lat_err_coeff`로 감소 정도 조절(1.0 기본, 보수적으로 하려면 ↑)
  - 기대효과: 코너 진입/자세 불일치 구간에서 안전마진 확대(랩타임 ↑ 가능)

- 출력 한계/상하한(안전망)
  - 파라미터: `long_min_speed`, `long_max_speed`
  - 설명: PID+FF 결과 속도에 대한 절대 하한/상한(m/s). 비합리적 명령을 차단하는 마지막 안전망입니다.
  - 이유: 제어 출력 절대 한계 설정(과도한 명령 방지)
  - 권장: min=0.0, max=차량/경기 규정에 맞춰 설정(기본 15.0)

- 로그/지표 권장
  - 비교: `/drive.speed` vs `/odom`(RMSE, 오버슈트, 정착시간)
  - 이벤트: 코너 구간 속도 스파이크/딥 여부(a_ff 클램프/레이트 리밋 조정 근거)
  - 체감: 슬립/휠스핀 여부(레이트 리밋/PD 재조정 근거)

주의/비고
- Trailing(추격) 로직은 현재 비활성화되어 `global_speed`를 그대로 사용합니다(향후 교체 예정).
- LUT가 없을 때는 휠베이스 0.33 m의 기구학 조향 근사로 동작합니다.
