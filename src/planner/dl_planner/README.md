# DL Planner - Deep Learning-based Offline Trajectory Planner

드론쇼를 위한 Deep Learning 기반 오프라인 사전 궤적 생성 플래너입니다.

## 개요

`dl_planner`는 군집 드론쇼를 위한 충돌 회피 궤적을 사전에 생성하는 시스템입니다. Deep Learning 모델을 사용하여 드론 간 충돌을 예측하고, 안전하고 부드러운 궤적을 최적화합니다.

### 주요 특징

- **Deep Learning 기반 충돌 예측**: 신경망 모델을 사용한 충돌 확률 예측
- **오프라인 궤적 생성**: 사전에 모든 드론의 궤적을 계산
- **순차적 계획**: 드론별로 순차적으로 계획하여 이전 드론의 궤적 고려
- **동적 제약 조건**: 최대 속도, 가속도 제약 적용
- **충돌 검증**: 생성된 궤적의 안전성 검증
- **RViz 시각화**: 실시간 궤적 시각화

## 프로젝트 구조

```
dl_planner/
├── include/dl_planner/
│   ├── neural_network/
│   │   └── neural_collision_net.hpp       # 신경망 모델 인터페이스
│   ├── trajectory_generator/
│   │   └── offline_trajectory_generator.hpp  # 궤적 생성기
│   ├── collision_checker/
│   │   └── swarm_collision_checker.hpp    # 충돌 검사기
│   ├── utils/
│   │   ├── data_types.hpp                 # 데이터 구조체
│   │   └── visualization.hpp              # 시각화 유틸
│   └── dl_planner_node.hpp                # ROS2 노드
├── src/
│   ├── neural_network/
│   │   └── neural_collision_net.cpp
│   ├── trajectory_generator/
│   │   └── offline_trajectory_generator.cpp
│   ├── collision_checker/
│   │   └── swarm_collision_checker.cpp
│   └── dl_planner_node.cpp (TODO)
├── config/
│   └── default.yaml                       # 기본 설정 파일
├── launch/
│   └── dl_planner.launch.py               # 런치 파일
├── models/
│   └── (신경망 모델 파일 위치)
├── scripts/
│   └── (Python 스크립트 위치)
└── CMakeLists.txt
```

## 아키텍처

### 1. Neural Collision Network (`NeuralCollisionNet`)

- 드론 상태를 입력으로 받아 충돌 확률 예측
- PyTorch/TensorFlow 모델 로드 지원 (TODO)
- 제어 명령 생성 기능 (TODO)

### 2. Offline Trajectory Generator (`OfflineTrajectoryGenerator`)

- 모든 드론의 궤적을 순차적으로 생성
- 초기 궤적 생성 (선형 보간)
- 경사 하강법 기반 최적화
- 동적 제약 조건 적용

### 3. Swarm Collision Checker (`SwarmCollisionChecker`)

- 드론 간 거리 기반 충돌 검사
- 궤적 전체에 대한 충돌 검증
- CPA (Closest Point of Approach) 계산
- 충돌 통계 수집

### 4. Visualization

- RViz를 통한 실시간 시각화
- 궤적, 드론 위치, 안전 영역 표시
- 충돌 지점 시각화

## 사용 방법

### 1. 빌드

```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
colcon build --packages-select dl_planner
source install/setup.bash
```

### 2. 설정 파일 수정

`config/default.yaml` 파일을 편집하여 드론 수, 시작/목표 위치, 제약 조건 등을 설정합니다.

### 3. 실행

```bash
ros2 launch dl_planner dl_planner.launch.py
```

#### 옵션 사용

```bash
ros2 launch dl_planner dl_planner.launch.py \
  num_drones:=20 \
  model_path:=/path/to/model.pt \
  output_path:=/path/to/output.traj \
  rviz:=true
```

### 4. 결과 확인

생성된 궤적은 지정된 경로에 저장됩니다. RViz를 통해 시각적으로 확인할 수 있습니다.

## 설정 파라미터

### 궤적 생성 파라미터

- `dt`: 시간 간격 (초)
- `max_duration`: 최대 궤적 길이 (초)
- `max_velocity`: 최대 속도 (m/s)
- `max_acceleration`: 최대 가속도 (m/s²)

### 안전 파라미터

- `safety_distance`: 드론 간 최소 거리 (m)
- `collision_threshold`: 충돌 확률 임계값

### 최적화 파라미터

- `max_iterations`: 최대 반복 횟수
- `convergence_threshold`: 수렴 기준
- `learning_rate`: 학습률

## 개발 계획

### Phase 1: 기본 프레임워크 (현재)
- [x] 패키지 구조 설계
- [x] 핵심 클래스 인터페이스 정의
- [x] 기본 구현 스켈레톤

### Phase 2: 핵심 기능 구현
- [ ] ROS2 노드 구현
- [ ] 시각화 모듈 구현
- [ ] 파일 I/O 구현
- [ ] 기본 테스트

### Phase 3: Deep Learning 통합
- [ ] 신경망 모델 설계
- [ ] 학습 데이터 생성
- [ ] 모델 학습
- [ ] 모델 로딩 및 추론 구현

### Phase 4: 최적화 및 검증
- [ ] 궤적 최적화 알고리즘 개선
- [ ] 다양한 시나리오 테스트
- [ ] 성능 벤치마크
- [ ] 실제 드론쇼 적용

## vs. Ego-Planner 비교

| 특징 | Ego-Planner | DL-Planner |
|------|-------------|------------|
| 계획 방식 | 온라인 (실시간) | 오프라인 (사전) |
| 최적화 | B-spline + L-BFGS | DL + 경사하강법 |
| 충돌 회피 | 기하학적 제약 | DL 예측 + 기하학적 |
| 용도 | 실시간 내비게이션 | 드론쇼 안무 |
| 계산 시간 | 빠름 (< 100ms) | 느림 (사전 계산 가능) |
| 궤적 품질 | 좋음 | 매우 좋음 (최적화 가능) |

## 참고 자료

- Ego-Planner: [GitHub](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
- ROS2 Documentation: [docs.ros.org](https://docs.ros.org)

## 라이선스

MIT License

## 개발자

- pjh <ds70768@naver.com>
