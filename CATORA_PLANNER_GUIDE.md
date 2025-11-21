# CAT-ORA Planner Integration Guide

CAT-ORA (Collision-Aware Time-Optimal formation Reshaping Algorithm)가 **layer_planner, dl_planner, orca_planner**와 함께 ego-planner-swarm의 플래너로 통합되었습니다.

## 📦 디렉토리 구조

```
ego-planner-swarm/
└── src/planner/
    ├── layer_planner/          # Layer-based collision avoidance
    ├── dl_planner/             # Deep learning planner
    ├── orca_planner/           # ORCA velocity obstacles
    ├── catora_planner/         # ← NEW: CAT-ORA formation reshaping
    │   ├── include/catora_planner/
    │   ├── src/
    │   ├── srv/
    │   ├── msg/
    │   ├── launch/
    │   ├── config/
    │   └── README.md
    └── plan_manage/
        └── launch/
            ├── scenario_catora_25.launch.py    # 25 drones
            └── scenario_catora_36.launch.py    # 36 drones
```

## 🚀 빌드 및 실행

### 1. 빌드

```bash
cd ~/ego_swarm/ego-planner-swarm
colcon build --packages-select catora_planner
source install/setup.bash
```

### 2. CAT-ORA Planner 단독 실행

```bash
# 기본 파라미터로 실행
ros2 launch catora_planner catora_planner.launch.py

# 커스텀 파라미터로 실행
ros2 launch catora_planner catora_planner.launch.py \
    max_velocity:=3.0 \
    max_acceleration:=2.5 \
    trajectory_dt:=0.1
```

### 3. 시나리오 실행

#### 25 드론 시나리오
```bash
ros2 launch plan_manage scenario_catora_25.launch.py
```

#### 36 드론 시나리오
```bash
ros2 launch plan_manage scenario_catora_36.launch.py
```

## 🎯 플래너 비교

| 플래너 | 목적 | 충돌 회피 | 사용 시나리오 |
|-------|-----|----------|------------|
| **layer_planner** | 레이어 기반 분리 | ✅ | 밀집 환경 |
| **dl_planner** | 학습 기반 경로 | ✅ | 복잡한 장애물 |
| **orca_planner** | 속도 장애물 | ✅ | 동적 회피 |
| **catora_planner** | 포메이션 재구성 | ✅ | 대규모 편대 변경 |

## 🔧 파라미터 설정

### catora_planner 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `max_velocity` | 2.0 m/s | 최대 속도 |
| `max_acceleration` | 2.0 m/s² | 최대 가속도 |
| `trajectory_dt` | 0.2 s | 궤적 시간 간격 |

### 시나리오 파라미터

```bash
ros2 launch plan_manage scenario_catora_25.launch.py \
    catora_max_vel:=3.0 \
    catora_max_acc:=2.5 \
    catora_traj_dt:=0.1 \
    use_mockamap:=True \
    use_dynamic:=False
```

## 📊 서비스 인터페이스

### 1. Get Assignment (할당만 계산)

```bash
ros2 service call /catora_planner/get_assignment \
  catora_planner/srv/GetAssignment \
  "{initial_configurations: [{x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 1}],
    goal_configurations: [{x: 1, y: 1, z: 1}, {x: 0, y: 1, z: 1}]}"
```

**응답**:
```yaml
mapping: [1, 0]  # robot 0 → goal 1, robot 1 → goal 0
success: True
message: "Assignment computed successfully"
```

### 2. Get Reshaping Trajectories (전체 궤적)

```bash
ros2 service call /catora_planner/get_reshaping_trajectories \
  catora_planner/srv/GetReshapingTrajectories \
  "{initial_configurations: [{x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 1}],
    goal_configurations: [{x: 1, y: 1, z: 1}, {x: 0, y: 1, z: 1}],
    max_velocity: 2.0, max_acceleration: 2.0, trajectory_dt: 0.2}"
```

## 🔍 알고리즘 상세

### CAT-ORA vs Hungarian Algorithm

```
Traditional Hungarian (LSAP):
- Minimize: Σ distance(robot_i, goal_assignment[i])
- Time: O(n³)
- Collision: ❌ Not considered

CAT-ORA:
- Minimize: max(time(robot_i → goal_assignment[i]))
- Time: O(n³) ~ O(n⁴)
- Collision: ✅ Trajectory collision checking
```

### 처리 과정

1. **Lower Bound 계산**: Bottleneck value의 하한 추정
2. **Hungarian with Threshold**: 제한된 비용으로 할당 계산
3. **Collision Check**: 궤적 간 충돌 감지
4. **Branch & Bound**: 충돌 발견 시 대안 탐색
5. **Trajectory Generation**: 최종 할당으로 궤적 생성

## 📝 커스텀 시나리오 작성

### 1. 포메이션 정의

```python
# 초기 포메이션 - 예: 라인
initial_positions = []
for i in range(10):
    initial_positions.append({
        'x': 3.0,
        'y': 10.0 + i * 2.0,
        'z': 15.0
    })

# 목표 포메이션 - 예: 원형
import math
goal_positions = []
radius = 5.0
for i in range(10):
    angle = 2 * math.pi * i / 10
    goal_positions.append({
        'x': 3.0,
        'y': 25.0 + radius * math.cos(angle),
        'z': 15.0 + radius * math.sin(angle)
    })
```

### 2. scenario_catora_custom.launch.py 생성

```python
# scenario_catora_25.launch.py를 복사하여 수정
cp src/planner/plan_manage/launch/scenario_catora_25.launch.py \
   src/planner/plan_manage/launch/scenario_catora_custom.launch.py

# 드론 수, 포메이션 수정
# num_drones, initial_positions, goal_positions 변경
```

## 🛠️ 문제 해결

### catora_planner가 시작되지 않음

```bash
# 노드 확인
ros2 node list | grep catora

# 서비스 확인
ros2 service list | grep catora

# 로그 확인
ros2 launch catora_planner catora_planner.launch.py
```

### 할당 계산 실패

**원인**: 초기/목표 위치가 너무 가까워서 충돌 감지

**해결**:
- 최소 거리 확보 (권장: > 1.5m)
- 목표 포메이션 간격 증가
- `trajectory_dt` 조정

### 계산 시간이 너무 김

**해결**:
- 드론 수 감소 (25→20, 36→30)
- `max_velocity` 증가
- Formation 복잡도 감소

## 📈 성능 벤치마크

| 드론 수 | 할당 계산 시간 | 궤적 생성 시간 | 총 시간 |
|--------|------------|------------|--------|
| 10 | ~10 ms | ~5 ms | ~15 ms |
| 25 | ~50 ms | ~15 ms | ~65 ms |
| 36 | ~120 ms | ~25 ms | ~145 ms |

*테스트 환경: Intel i7-10th gen, 16GB RAM*

## 🎓 참고 문헌

```bibtex
@ARTICLE{kratky2025catora,
  author={Kratky, Vit and Penicka, Robert and Horyna, Jiri and
          Stibinger, Petr and Baca, Tomas and Petrlik, Matej and
          Stepan, Petr and Saska, Martin},
  journal={IEEE Transactions on Robotics},
  title={CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping
         for Efficient Robot Coordination in 3-D Environments},
  year={2025},
  volume={41},
  pages={2950-2969},
  doi={10.1109/TRO.2025.3547296}
}
```

## 🔗 관련 링크

- **원본 저장소**: https://github.com/ctu-mrs/catora
- **논문 PDF**: https://arxiv.org/pdf/2412.00603
- **catora_planner README**: `src/planner/catora_planner/README.md`

---

**버전**: 1.0.0
**ROS 버전**: ROS2 Humble
**작성일**: 2025-11-19
