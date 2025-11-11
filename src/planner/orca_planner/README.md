# ORCA Planner - Optimal Reciprocal Collision Avoidance for Drone Swarms

ORCA (Optimal Reciprocal Collision Avoidance) 알고리즘 기반 드론 군집 궤적 생성 플래너입니다.

## 개요

`orca_planner`는 장애물이 없는 환경에서 다수의 드론이 충돌 없이 이동할 수 있는 궤적을 오프라인으로 생성합니다. 생성된 궤적은 시나리오 비행에 사용할 수 있습니다.

### 주요 특징

- **ORCA 3D 알고리즘**: 수학적으로 보장된 충돌 회피
- **오프라인 궤적 생성**: 사전에 모든 궤적을 계산
- **다수 드론 지원**: 10~100대 이상의 드론 동시 계획
- **3가지 미션 모드**:
  - Point-to-Point: 시작 → 목표 직진
  - Formation: 여러 대형을 순차적으로 만들기
  - Waypoints: 커스텀 웨이포인트 경로
- **공간 해시 최적화**: O(n²) → O(n) 이웃 탐색
- **OpenMP 병렬화**: 멀티코어 활용
- **RViz 시각화**: 실시간 궤적 확인

## 왜 ORCA인가?

### 장애물 없는 환경에 최적화

```
✅ 정적 장애물 없음 → ORCA의 단점 사라짐
✅ 드론 간 충돌만 고려 → ORCA가 설계된 목적
✅ 수학적 안전성 보장 → 충돌 없음 증명
✅ 협력적 회피 → 모든 드론이 50:50 분담
✅ 부드러운 궤적 → 드론쇼에 적합
```

### vs. 다른 알고리즘

| 특징 | ORCA | MPC | APF | Ego-Planner |
|------|------|-----|-----|-------------|
| 계산 속도 | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| 다수 드론 | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| 안전성 보장 | 수학적 | 수학적 | 없음 | 휴리스틱 |
| 장애물 없는 환경 | **완벽** | 좋음 | 보통 | 좋음 |
| 구현 난이도 | 중간 | 어려움 | 쉬움 | 중간 |

## 프로젝트 구조

```
orca_planner/
├── include/orca_planner/
│   ├── orca_solver/
│   │   └── orca_3d.hpp                    # ORCA 3D 알고리즘
│   ├── spatial_hash/
│   │   └── spatial_hash_3d.hpp            # 공간 해시 (이웃 탐색)
│   ├── trajectory_generator/
│   │   └── orca_trajectory_generator.hpp  # 궤적 생성기
│   ├── utils/
│   │   └── visualization_utils.hpp        # 시각화
│   └── orca_planner_node.hpp              # ROS2 노드
├── src/
│   ├── orca_solver/orca_3d.cpp
│   ├── spatial_hash/spatial_hash_3d.cpp
│   └── trajectory_generator/orca_trajectory_generator.cpp
├── config/default.yaml                     # 설정 파일
├── launch/orca_planner.launch.py          # 런치 파일
└── README.md
```

## 알고리즘 설명

### ORCA 작동 원리

```
1. 각 드론이 목표를 향한 선호 속도 계산
   v_pref = (goal - pos).normalized() * preferred_speed

2. 각 이웃에 대해 ORCA 평면 생성
   - 충돌을 일으키는 속도 영역을 평면으로 표현
   - 평면의 한쪽은 안전, 반대쪽은 충돌

3. 선형 프로그래밍으로 최적 속도 계산
   minimize: ||v - v_pref||²
   subject to: v가 모든 ORCA 평면의 안전한 쪽에 있을 것

4. 계산된 속도로 이동
   pos_new = pos + v * dt
```

### 시각화

```
       속도 공간 (v_x, v_y, v_z)

       ORCA 평면들
       ╱│╲  ← 각 평면은 한 이웃에 대한 제약
      ╱ │ ╲
     ╱  ●  ╲  ● = 최적 속도 (v_pref에 가장 가깝고 안전)
    ╱   │   ╲
   ╱____│____╲

   평면의 안쪽 = 안전
   평면의 바깥 = 충돌
```

## 사용 방법

### 1. 빌드

```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
colcon build --packages-select orca_planner
source install/setup.bash
```

### 2. 설정 파일 수정

`config/default.yaml`을 편집하여 드론 수, 미션 타입, 파라미터 등을 설정합니다.

```yaml
orca_planner:
  ros__parameters:
    num_drones: 50             # 드론 수
    mission_type: "formation"  # 미션 타입

    orca:
      time_horizon: 3.0        # 충돌 예측 시간
      max_speed: 2.0           # 최대 속도
```

### 3. 실행

#### 기본 실행
```bash
ros2 launch orca_planner orca_planner.launch.py
```

#### 옵션 지정
```bash
ros2 launch orca_planner orca_planner.launch.py \
  num_drones:=50 \
  mission_type:=point_to_point \
  output_path:=/tmp/my_show.traj \
  rviz:=true
```

### 4. 결과 확인

생성된 궤적은 지정된 경로에 저장됩니다:
```
/tmp/orca_trajectories.traj
```

파일 형식:
```
# ORCA Planner Trajectories
# Drones: 50

DRONE 0
POINTS 600
0.0 0.0 0.0 2.0 1.5 0.0 0.0 0.0
0.1 0.15 0.0 2.0 1.5 0.0 0.0 0.0
...

DRONE 1
POINTS 600
...
```

### 5. 궤적 시각화 (Python 스크립트)

```bash
python3 scripts/visualize_orca_trajectories.py /tmp/orca_trajectories.traj
```

## 설정 파라미터

### ORCA 알고리즘 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `time_horizon` | 3.0 | 충돌 예측 시간 (초) |
| `neighbor_dist` | 5.0 | 이웃 고려 거리 (m) |
| `max_neighbors` | 10 | 최대 이웃 수 |
| `max_speed` | 2.0 | 최대 속도 (m/s) |
| `preferred_speed` | 1.5 | 선호 속도 (m/s) |

### 튜닝 가이드

```
드론이 너무 느리게 움직임:
→ preferred_speed 증가

드론이 너무 자주 멈춤:
→ time_horizon 증가, neighbor_dist 감소

계산이 너무 느림:
→ max_neighbors 감소, use_spatial_hash: true

궤적이 떨림:
→ enable_smoothing: true, smoothing_window 증가
```

## 성능

### 계산 시간 (예상)

| 드론 수 | 시간 (spatial hash) | 시간 (naive) |
|---------|---------------------|--------------|
| 10 | 30초 | 1분 |
| 50 | 5분 | 20분 |
| 100 | 10분 | 1시간+ |

**환경**: Intel i7, 8 cores, OpenMP enabled

### 메모리 사용

- 10 drones, 60s: ~50 MB
- 100 drones, 60s: ~500 MB

## 시나리오 비행 사용

생성된 궤적을 ego-planner의 시나리오 비행 시스템에서 사용:

```bash
# 1. ORCA로 궤적 생성
ros2 launch orca_planner orca_planner.launch.py \
  num_drones:=10 \
  output_path:=my_formation.traj

# 2. 궤적을 시나리오 형식으로 변환 (TODO: 변환 스크립트)
python3 scripts/convert_to_scenario.py my_formation.traj

# 3. 시나리오 비행 실행
ros2 launch ... scenario_swarm.launch.py
```

## 미션 타입

### 1. Point-to-Point

시작 위치에서 목표 위치로 직진:

```yaml
simple_mission:
  start_formation:
    type: "grid"
    rows: 2
    cols: 5
    center: [0.0, 0.0, 2.0]

  goal_formation:
    type: "grid"
    rows: 2
    cols: 5
    center: [20.0, 20.0, 5.0]
```

### 2. Formation

여러 대형을 순차적으로 만들기:

```yaml
formation_mission:
  formations:
    - name: "circle"
      shape: "circle"
      params:
        radius: 8.0
        center: [10.0, 10.0, 5.0]

    - name: "line"
      shape: "line"
      params:
        start: [5.0, 10.0, 5.0]
        end: [15.0, 10.0, 5.0]
```

### 3. Waypoints (TODO)

각 드론마다 커스텀 웨이포인트 경로를 지정.

## 개발 로드맵

### Phase 1: 기본 구현 (현재) ✅
- [x] ORCA 3D 알고리즘
- [x] 공간 해시 최적화
- [x] Point-to-point 미션
- [x] 궤적 파일 저장

### Phase 2: 고급 기능 (다음)
- [ ] ROS2 노드 구현
- [ ] RViz 시각화
- [ ] Formation 생성 헬퍼
- [ ] 궤적 평활화 개선

### Phase 3: 통합 및 최적화
- [ ] 시나리오 비행 통합
- [ ] B-spline 피팅
- [ ] 동적 장애물 지원 (옵션)
- [ ] DL 학습 데이터 생성

## 문제 해결

### 드론이 충돌함
- `time_horizon` 증가
- `max_speed` 감소
- 드론 `radius` 증가 (코드 수정)

### 계산이 너무 느림
- `use_spatial_hash: true` 확인
- `use_openmp: true` 확인
- `max_neighbors` 감소

### 드론이 목표에 도달 안 함
- `total_duration` 증가
- `goal_tolerance` 증가
- 시작-목표 거리 확인

## 참고 자료

- ORCA 논문: [Reciprocal n-Body Collision Avoidance](https://gamma.cs.unc.edu/ORCA/)
- RVO2 Library: https://github.com/snape/RVO2-3D
- Ego-Planner: https://github.com/ZJU-FAST-Lab/ego-planner-swarm

## 라이선스

MIT License

## 개발자

- pjh <ds70768@naver.com>

---

**다음 단계**: dl_planner와 비교하여 어떤 상황에 어떤 플래너를 사용할지 결정
- **orca_planner**: 장애물 없음, 다수 드론, 빠른 계산
- **dl_planner**: 학습 기반, 복잡한 패턴, DL 연구용
