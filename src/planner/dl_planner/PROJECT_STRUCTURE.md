# DL Planner 프로젝트 구조

## 개요

`dl_planner`는 Deep Learning 기반 오프라인 궤적 생성 플래너입니다. 이 문서는 프로젝트의 전체 구조와 각 구성 요소를 설명합니다.

## 디렉토리 구조

```
dl_planner/
├── CMakeLists.txt                          # CMake 빌드 설정
├── package.xml                             # ROS2 패키지 정의
├── README.md                               # 프로젝트 설명서
│
├── include/dl_planner/                     # 헤더 파일
│   ├── collision_checker/
│   │   └── swarm_collision_checker.hpp    # 군집 충돌 검사기
│   ├── neural_network/
│   │   └── neural_collision_net.hpp       # 신경망 충돌 예측 모델
│   ├── trajectory_generator/
│   │   └── offline_trajectory_generator.hpp  # 오프라인 궤적 생성기
│   ├── utils/
│   │   ├── data_types.hpp                 # 공통 데이터 타입
│   │   └── visualization.hpp              # RViz 시각화
│   └── dl_planner_node.hpp                # ROS2 노드 인터페이스
│
├── src/                                    # 구현 파일
│   ├── collision_checker/
│   │   └── swarm_collision_checker.cpp
│   ├── neural_network/
│   │   └── neural_collision_net.cpp
│   ├── trajectory_generator/
│   │   └── offline_trajectory_generator.cpp
│   └── (노드 구현 파일 추가 예정)
│
├── config/                                 # 설정 파일
│   └── default.yaml                        # 기본 설정
│
├── launch/                                 # 런치 파일
│   └── dl_planner.launch.py               # 메인 런치 파일
│
├── models/                                 # 신경망 모델
│   └── README.md                           # 모델 사용 가이드
│
└── scripts/                                # 유틸리티 스크립트
    └── visualize_trajectories.py          # 궤적 시각화 도구
```

## 핵심 구성 요소

### 1. Neural Network Module (`neural_network/`)

**NeuralCollisionNet** - 충돌 예측 신경망

- **목적**: 드론 상태를 입력받아 충돌 확률 예측
- **입력**: 두 드론의 위치와 속도 (12차원)
- **출력**: 충돌 확률 [0, 1]
- **주요 메서드**:
  - `loadModel()`: 사전 학습된 모델 로드
  - `predictCollisionProbability()`: 충돌 확률 예측
  - `predictControlAction()`: 회피 제어 명령 생성
  - `batchPredict()`: 배치 예측

**구현 상태**: ⚠️ 스켈레톤 (모델 로딩 및 추론 구현 필요)

### 2. Trajectory Generator Module (`trajectory_generator/`)

**OfflineTrajectoryGenerator** - 오프라인 궤적 생성기

- **목적**: 모든 드론의 충돌 없는 궤적을 사전 생성
- **특징**:
  - 순차적 계획 (드론별로 차례대로)
  - 초기 궤적 생성 (선형 보간)
  - 경사 하강법 기반 최적화
  - 동적 제약 조건 적용 (속도, 가속도)
- **주요 메서드**:
  - `generateTrajectories()`: 전체 드론 궤적 생성
  - `generateSingleTrajectory()`: 단일 드론 궤적 생성
  - `optimizeTrajectory()`: 궤적 최적화
  - `validateTrajectory()`: 안전성 검증
  - `saveTrajectories()`: 파일 저장

**데이터 구조**:
```cpp
struct DroneTrajectory {
  int drone_id;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<Eigen::Vector3d> accelerations;
  std::vector<double> timestamps;
  double total_time;
};

struct TrajectoryConfig {
  double dt;                    // 시간 간격
  double max_duration;          // 최대 지속 시간
  double max_velocity;          // 최대 속도
  double max_acceleration;      // 최대 가속도
  double safety_distance;       // 안전 거리
  int max_iterations;           // 최대 반복
  // ...
};
```

**구현 상태**: ✅ 기본 구현 완료 (최적화 알고리즘 개선 필요)

### 3. Collision Checker Module (`collision_checker/`)

**SwarmCollisionChecker** - 군집 충돌 검사기

- **목적**: 드론 간 충돌 검사 및 분석
- **기능**:
  - 점 대 점 충돌 검사
  - 궤적 대 궤적 충돌 검사
  - 최소 거리 계산
  - CPA (Closest Point of Approach) 계산
  - 충돌 통계 수집
- **주요 메서드**:
  - `checkCollision()`: 두 위치 간 충돌 검사
  - `checkTrajectoryCollision()`: 두 궤적 간 충돌 검사
  - `checkSwarmCollision()`: 한 궤적과 여러 궤적 간 충돌
  - `checkAllCollisions()`: 모든 궤적 쌍 검사
  - `getMinimumDistance()`: 최소 거리 계산

**데이터 구조**:
```cpp
struct CollisionInfo {
  bool has_collision;
  int drone_id_1;
  int drone_id_2;
  double time;
  Eigen::Vector3d position;
  double distance;
};
```

**구현 상태**: ✅ 기본 구현 완료

### 4. Utilities Module (`utils/`)

**data_types.hpp** - 공통 데이터 타입

- `DroneState`: 드론 상태 (위치, 속도, 가속도)
- `SwarmState`: 군집 상태
- `Waypoint`: 웨이포인트
- `DroneMission`: 드론 미션
- `SwarmMission`: 군집 미션
- `PlanningStatistics`: 계획 통계
- `BoundingBox`: 경계 상자

**visualization.hpp** - 시각화 도구

- RViz 마커 생성 및 발행
- 궤적, 드론, 충돌 지점 시각화
- 안전 영역 시각화

**구현 상태**: ⚠️ 스켈레톤 (구현 필요)

### 5. ROS2 Node (`dl_planner_node.hpp`)

**DLPlannerNode** - 메인 ROS2 노드

- **역할**: ROS2 인터페이스 제공
- **기능**:
  - 파라미터 로드
  - 미션 설정 파일 읽기
  - 궤적 생성 실행
  - 결과 시각화 및 저장
- **토픽**:
  - Publishers: `/dl_planner/path`, `/dl_planner/status`
  - Subscribers: (필요시 추가)

**구현 상태**: ⚠️ 스켈레톤 (구현 필요)

## 데이터 흐름

```
1. 설정 로드
   config/default.yaml → DLPlannerNode

2. 모델 로드
   models/*.pt → NeuralCollisionNet

3. 미션 정의
   start_positions, goal_positions → OfflineTrajectoryGenerator

4. 순차적 궤적 생성
   For each drone:
     a. 초기 궤적 생성 (선형 보간)
     b. 최적화 (신경망 + 기하학적 제약)
        - NeuralCollisionNet: 충돌 확률 예측
        - SwarmCollisionChecker: 충돌 검증
     c. 동적 제약 적용
     d. 검증

5. 결과 저장 및 시각화
   trajectories → 파일 저장
   trajectories → Visualization → RViz
```

## 알고리즘 흐름

### 궤적 생성 알고리즘

```python
function generateTrajectories(starts, goals):
    trajectories = []

    for i in range(num_drones):
        # 1. 초기 궤적 (선형 보간)
        traj = linearInterpolation(starts[i], goals[i])

        # 2. 최적화
        for iter in range(max_iterations):
            # 충돌 비용 계산
            collision_cost = 0
            for prev_traj in trajectories:
                # 신경망 예측
                prob = neural_net.predict(traj, prev_traj)
                collision_cost += prob

                # 기하학적 검사
                min_dist = collision_checker.getMinDistance(traj, prev_traj)
                if min_dist < safety_distance:
                    collision_cost += penalty

            # 평활도 비용
            smoothness_cost = sum(accelerations^2)

            # 경사 하강
            total_cost = collision_cost + smoothness_cost
            if total_cost < threshold:
                break

            traj = updateTrajectory(traj, gradient)

        # 3. 동적 제약 적용
        traj = clampVelocity(traj, max_vel)
        traj = clampAcceleration(traj, max_acc)

        # 4. 검증
        if validateTrajectory(traj, trajectories):
            trajectories.append(traj)
        else:
            return FAILURE

    return trajectories
```

## 설정 파일 구조

`config/default.yaml`:

```yaml
dl_planner:
  ros__parameters:
    num_drones: 10
    model_path: "models/collision_net.pt"
    output_path: "trajectories/output.traj"

    trajectory:
      dt: 0.1
      max_velocity: 2.0
      max_acceleration: 2.0
      safety_distance: 1.0

    mission:
      start_positions: [[x, y, z], ...]
      goal_positions: [[x, y, z], ...]
```

## TODO 리스트

### 높은 우선순위
- [ ] DLPlannerNode 구현 (src/)
- [ ] Visualization 클래스 구현
- [ ] 신경망 모델 학습 및 로딩 구현
- [ ] 최적화 알고리즘 개선 (경사 계산)

### 중간 우선순위
- [ ] 파일 I/O 구현 (CSV, JSON 지원)
- [ ] RViz 설정 파일 작성
- [ ] 단위 테스트 작성
- [ ] 예제 미션 파일 추가

### 낮은 우선순위
- [ ] 병렬 계획 (여러 드론 동시 계획)
- [ ] 동적 장애물 지원
- [ ] 실시간 재계획 기능
- [ ] GUI 도구

## 빌드 및 실행

### 빌드
```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
colcon build --packages-select dl_planner
source install/setup.bash
```

### 실행
```bash
ros2 launch dl_planner dl_planner.launch.py
```

### 궤적 시각화
```bash
python3 src/planner/dl_planner/scripts/visualize_trajectories.py output.traj --plot-type both
```

## 의존성

- **ROS2**: Humble 이상
- **Eigen3**: 선형대수 연산
- **traj_utils**: ego-planner 궤적 유틸리티
- **plan_env**: ego-planner 환경 모듈
- **Python**: matplotlib (시각화용)

## 참고 자료

- Ego-Planner: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
- ROS2 Documentation: https://docs.ros.org
- Eigen3: https://eigen.tuxfamily.org

## 연락처

개발자: pjh <ds70768@naver.com>
