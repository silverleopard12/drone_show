# EGO-Planner-Swarm 작업 로그

작업 날짜: 2025-11-04

## 📋 작업 요약

1. scenario_swarm 드론 도착 문제 분석 및 해결 시도
2. 설정값 조정 및 테스트
3. 문제 발생으로 원래 상태로 복구
4. layer_swarm 설정값 scenario_swarm에 맞춰 통일

---

## 🔍 1. 초기 문제 분석

### 문제 상황
- scenario_swarm 실행 시 대부분의 드론은 목표에 도착하지만, 일부 드론이 목표 좌표 근처에서 멈춤
- RViz 상에서는 목표에 도달하지 않았는데 터미널에서는 "도착"으로 표시됨

### 원인 분석

**ego_replan_fsm.cpp (602-605줄)**
```cpp
if (t_cur > info->duration_ - 1e-2)
{
  have_target_ = false;
  have_trigger_ = false;
  // ... 도착 처리
}
```
- Trajectory duration이 끝나면 도착으로 판단
- **실제 위치가 목표에 가까운지는 확인하지 않음**

**mission_timer.py (107줄)**
```python
distance = math.sqrt(...)
if distance < self.arrival_threshold:  # 0.5m
    # 도착으로 간주
```
- 실제 위치가 목표로부터 0.5m 이내여야 도착으로 간주

**결과**: Trajectory가 끝났지만 실제로는 목표에서 0.5m ~ 1.0m 떨어진 상태에서 멈춤

---

## 🛠️ 2. 시도한 해결 방법들

### 2.1 도착 정밀도 향상 (0.5m → 0.2m)

**수정 파일들:**
- `src/planner/plan_manage/src/ego_replan_fsm.cpp`
- `allocation/tools/mission_timer.py`
- `src/planner/plan_manage/launch/scenario_swarm.launch.py`

**변경 내용:**
```cpp
// ego_replan_fsm.cpp
double dist_to_goal = (odom_pos_ - end_pt_).norm();
constexpr double ARRIVAL_THRESHOLD = 0.2; // 0.5m에서 변경

if (dist_to_goal < ARRIVAL_THRESHOLD) {
    // 실제로 도착했을 때만 처리
}
```

### 2.2 충돌 방지 강화

**파일**: `src/planner/plan_manage/launch/advanced_param.launch.py`

| 파라미터 | 기존 값 | 변경 값 | 목적 |
|---------|---------|---------|------|
| swarm_clearance | 1.0m | 1.5m | 드론 간 최소 안전거리 증가 |
| dist0 | 0.5m | 0.7m | 회피 시작 거리 증가 |
| lambda_collision | 4.0 | 6.0 | 충돌 회피 가중치 증가 |

### 2.3 진동 방지

**파일**: `src/planner/plan_manage/src/planner_manager.cpp`

```cpp
// Replan threshold 조정
if ((start_pt - local_target_pt).norm() < 0.05)  // 0.2m에서 변경
{
  return false;  // replan 안 함
}
```

**효과**: 목표 근처 (0.05m ~ 0.2m)에서도 replan 가능

### 2.4 도착 후 Replan 차단

**파일**: `src/planner/plan_manage/src/ego_replan_fsm.cpp`

**4곳 수정:**

1. **EXEC_TRAJ 시작** (587-590줄)
```cpp
if (goal_reached_) { break; }
```

2. **도착 판정 후** (643-644줄)
- `planNextWaypoint` 호출 제거

3. **Replan 조건** (657, 662줄)
```cpp
&& !goal_reached_
```

4. **checkCollisionCallback** (765줄)
```cpp
if (... || goal_reached_) return;
```

### 2.5 고정된 Hover 위치

**파일**: `src/planner/plan_manage/src/ego_replan_fsm.cpp`

```cpp
// 도착 시 고정된 hover
callEmergencyStop(end_pt_);  // odom_pos_ 대신 end_pt_ 사용

// WAIT_TARGET에서
if (!goal_reached_) {
    // hover 업데이트
} else {
    // 고정된 hover 유지
}
```

---

## ⚠️ 3. 발생한 문제

### 진동 문제
- 일부 드론이 목표 지점에 도착 후에도 진동 발생
- 고정된 hover로 해결 시도했으나 완전히 해결되지 않음

### 10번 드론 문제
- 10번 드론만 출발을 안 하는 현상 발생
- 궤적이 RViz에서 안 보임
- **원인**: 단순 시뮬레이션 오류 (재실행 시 해결)

---

## 🔄 4. 원래 상태로 복구

모든 수정사항이 완벽하게 작동하지 않아 **C안(모든 수정 되돌리기)** 선택

### 복구 과정
```bash
# 1. Git으로 모든 파일 복구
git checkout HEAD -- .

# 2. 빌드 아티팩트 제거
rm -rf common install log simulator build

# 3. 전체 재빌드
colcon build --symlink-install
```

### 복구된 파일들
- `src/planner/plan_manage/src/ego_replan_fsm.cpp`
- `src/planner/plan_manage/src/planner_manager.cpp`
- `src/planner/plan_manage/launch/advanced_param.launch.py`
- `src/planner/plan_manage/launch/scenario_swarm.launch.py`
- `allocation/tools/mission_timer.py`

**결과**: ✅ 빌드 성공 (22개 패키지)

---

## 🔧 5. layer_swarm 설정값 통일

### 5.1 사용하는 Launch 파일 구조

**scenario_swarm.launch.py**
```
scenario_swarm.launch.py
  └─ run_in_sim.launch.py (각 드론)
      ├─ advanced_param.launch.py → ego_planner_node
      ├─ simulator.launch.py
      └─ traj_server
```

**layer_swarm.launch.py**
```
layer_swarm.launch.py
  ├─ simulator.launch.py (각 드론)
  ├─ traj_server (각 드론)
  └─ layer_planner_node (단일, 중앙 관리)
```

**핵심 차이**:
- `scenario_swarm`: 각 드론마다 **ego_planner** (분산 planning)
- `layer_swarm`: **layer_planner 1개**가 모든 드론 관리 (중앙 planning)

### 5.2 설정값 비교

| 파라미터 | scenario_swarm | layer_swarm (이전) | 차이 |
|---------|----------------|-------------------|------|
| max_vel | 2.0 m/s | 3.0 m/s | 50% 더 빠름 |
| max_acc | 3.0 m/s² | 4.0 m/s² | 33% 더 빠름 |
| max_jerk | 6.0 m/s³ | 6.0 m/s³ | 동일 |
| swarm_clearance / safety_clearance | 1.0 m | 0.3 m | 233% 차이! |

### 5.3 수정 내용

**파일**: `src/planner/layer_planner/config/layer_planner_params.yaml`

```yaml
# Before
max_vel: 3.0
max_acc: 4.0
safety_clearance: 0.3

# After (scenario_swarm과 동일하게)
max_vel: 2.0
max_acc: 3.0
safety_clearance: 1.0
```

**빌드**:
```bash
colcon build --packages-select layer_planner --symlink-install
```

**결과**: ✅ 성공

---

## 📊 6. 최종 상태

### 현재 설정 (scenario_swarm = layer_swarm)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| max_vel | 2.0 m/s | 최대 속도 |
| max_acc | 3.0 m/s² | 최대 가속도 |
| max_jerk | 6.0 m/s³ | 최대 저크 |
| 안전거리 | 1.0 m | 드론 간 최소 거리 |
| planning_horizon | 7.5 | ego_planner만 |
| planning_rate | 5.0 Hz | layer_planner만 |

### 적용된 변경사항

1. ✅ `run_in_sim_minimal.launch.py` → `run_in_sim.launch.py` (scenario_swarm)
2. ✅ layer_planner 설정값 scenario_swarm에 맞춰 통일

### 보류된 개선사항

다음 기능들은 진동 문제로 인해 보류:
- ❌ 실제 위치 기반 정확한 도착 판정
- ❌ goal_reached_ 플래그 기반 replan 차단
- ❌ 고정된 hover 위치
- ❌ 향상된 충돌 방지 설정

---

## 🎯 7. 주요 학습 내용

### 7.1 드론 도착 판정 메커니즘

**ego_planner**:
- Trajectory duration 기반 판정
- 실제 위치 확인 없음

**mission_timer**:
- 실제 거리 기반 판정 (0.5m threshold)

### 7.2 Planning 아키텍처

**Ego-Planner (분산)**:
- 각 드론이 독립적으로 planning
- Swarm trajectory broadcast로 충돌 회피
- 더 robust하지만 계산 부하 큼

**Layer-Planner (중앙)**:
- 단일 노드가 모든 드론 관리
- Layer별로 드론 할당
- 계산 효율적이지만 단일 실패점

### 7.3 충돌 회피 파라미터

| 파라미터 | 역할 |
|---------|------|
| swarm_clearance | 드론 간 최소 유지 거리 |
| dist0 | 회피 시작 거리 |
| lambda_collision | 충돌 회피 가중치 |
| safety_clearance | Layer 내 최소 거리 |

---

## 📝 8. 향후 작업 제안

### 8.1 도착 정밀도 개선

현재 문제점을 해결하면서 진동 없이 정확한 도착 판정을 위해:

1. **PID 컨트롤러 튜닝**
   - Position controller gain 조정
   - 더 안정적인 hover

2. **Damping 추가**
   - 목표 근처에서 속도 감속
   - Smooth한 도착

3. **Hybrid 도착 판정**
   - Trajectory duration + 실제 위치 + 속도
   - 3가지 조건 모두 만족 시 도착

### 8.2 안전성 향상

1. **적응형 안전거리**
   - 속도에 따라 동적 조정
   - 빠르면 거리 증가, 느리면 감소

2. **충돌 감지 강화**
   - 예측 기반 충돌 회피
   - Emergency brake 조건 개선

### 8.3 성능 모니터링

1. **Real-time 모니터링 도구**
   - 각 드론의 상태 시각화
   - 충돌 위험도 표시

2. **로깅 개선**
   - 도착 시간 기록
   - 궤적 품질 메트릭
   - 충돌 near-miss 기록

---

## 🔧 9. 트러블슈팅 가이드

### 9.1 드론이 출발 안 할 때

**체크리스트**:
```bash
# 1. 노드 실행 확인
ros2 node list | grep drone_X

# 2. Odometry 수신 확인
ros2 topic echo /drone_X_visual_slam/odom --once

# 3. Trigger 수신 확인
ros2 topic echo /traj_start_trigger --once

# 4. FSM 상태 확인
ros2 topic echo /drone_X_ego_planner/data_display --once
```

**원인 가능성**:
- Trigger 미수신
- Odometry 없음
- Planning 실패
- 시뮬레이션 타이밍 이슈 → **재실행**

### 9.2 RViz에서 드론 안 보일 때

```bash
# Visualization 노드 확인
ros2 node list | grep odom_visualization

# Marker 토픽 확인
ros2 topic echo /drone_X_odom_visualization/mesh --once
```

**RViz 설정**:
1. Add → By topic → `/drone_X_odom_visualization/mesh`
2. Marker 색상/크기 조정

### 9.3 빌드 에러 시

```bash
# 클린 빌드
rm -rf build install log
colcon build --symlink-install

# 특정 패키지만
colcon build --packages-select ego_planner --symlink-install
```

---

## 📚 10. 참고 파일 위치

### Launch 파일
```
src/planner/plan_manage/launch/
├── scenario_swarm.launch.py          # 36 drones, ego_planner
├── run_in_sim.launch.py              # 단일 드론 (ego_planner)
├── run_in_sim_minimal.launch.py     # 최소 설정
├── advanced_param.launch.py          # ego_planner 파라미터
└── simulator.launch.py               # 시뮬레이터

src/planner/layer_planner/launch/
└── layer_swarm.launch.py             # 36 drones, layer_planner
```

### 설정 파일
```
src/planner/layer_planner/config/
└── layer_planner_params.yaml         # Layer planner 설정

allocation/tools/
├── mission_timer.py                  # 미션 타이머
└── swarm_synchronizer.py            # Swarm 동기화
```

### 코어 코드
```
src/planner/plan_manage/src/
├── ego_replan_fsm.cpp               # FSM 로직
├── planner_manager.cpp              # Planning 관리
└── ego_planner_node.cpp             # Main node

src/planner/layer_planner/src/
└── layer_planner_node.cpp           # Layer planner
```

---

## ✅ 체크리스트

- [x] scenario_swarm 문제 분석 완료
- [x] 해결 방법 시도 및 문제 발생 확인
- [x] 원래 상태로 안전하게 복구
- [x] layer_swarm 설정값 scenario_swarm과 통일
- [x] 작업 내용 문서화 완료
- [ ] 도착 정밀도 개선 (향후 작업)
- [ ] 진동 문제 근본 해결 (향후 작업)
- [ ] 성능 모니터링 도구 개발 (향후 작업)

---

**작성자**: Claude Code
**마지막 수정**: 2025-11-04
**버전**: 1.0
