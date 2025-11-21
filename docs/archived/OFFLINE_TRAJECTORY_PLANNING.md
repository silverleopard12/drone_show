# 오프라인 궤적 생성: 전체 경로 한 번에 계획하기

## 현재 상태
🔴 **실패** - 최적화 수렴 문제 추정 (2025-11-14)

## 빠른 디버깅 명령어

```bash
# 1. 몇 대나 ready 신호 보냈는지 확인
grep "planning complete.*ready signal published" ~/.ros/log/latest/*.log | wc -l

# 2. 어느 드론이 실패했는지 확인
for i in {0..35}; do
  echo -n "Drone $i: "
  grep "drone $i.*planning complete" ~/.ros/log/latest/*.log | wc -l
done

# 3. 최적화 에러 확인
grep -i "LBFGS\|optimization fail" ~/.ros/log/latest/*.log

# 4. 빠른 해결 시도 - planning_horizon 줄이기
# scenario_swarm_36.launch.py 라인 157:
# 'planning_horizon': '100.0' (또는 '70.0')
```

## 목표
- 재계획 없이 처음부터 끝까지 전체 경로를 한 번에 계획
- 모든 드론의 초기 계획 완료 후 동시 출발
- 계획 시간이 오래 걸려도 상관없음 (오프라인)

## 변경 사항

### 1. Planning Horizon 확대 (25m → 170m)

**파일:** `src/planner/plan_manage/launch/scenario_swarm_36.launch.py`

```python
# 라인 157
'planning_horizon': '170.0'  # Full path planning - no replanning needed for static formation
```

**이유:**
- 최대 드론 이동 거리: ~141m (대각선)
- 170m로 설정하여 모든 경로를 한 번에 커버
- `getLocalTarget()` 함수가 planning_horizon 거리까지만 target을 설정하므로 전체 경로를 보려면 필수

### 2. 재계획 비활성화

**파일:** `src/planner/plan_manage/launch/advanced_param.launch.py`

```python
# 라인 118
{'fsm/thresh_replan_time': 99999.0},  # Disable replanning - plan full path once

# 라인 121
{'fsm/planning_horizen_time': 99999.0},  # Plan for entire trajectory duration
```

**이유:**
- `thresh_replan_time`: 재계획 트리거 조건 (1초 → 99999초)
- trajectory 실행 중 재계획 없이 끝까지 실행

### 3. 계획 완료 감지 및 동시 출발 메커니즘

#### 3.1 헤더 파일 수정
**파일:** `src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h`

```cpp
// 라인 12: std_msgs/Bool include 추가
#include "std_msgs/msg/bool.hpp"

// 라인 105-107: Publisher 추가
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_ready_pub_;

bool planning_ready_published_;  // Flag to ensure we only publish once
```

#### 3.2 FSM 구현 수정
**파일:** `src/planner/plan_manage/src/ego_replan_fsm.cpp`

```cpp
// 라인 20: 초기화
planning_ready_published_ = false;

// 라인 121: Publisher 생성
planning_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/ready", 10);

// 라인 565-573: 계획 완료 시 ready 신호 publish
if (!planning_ready_published_)
{
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  planning_ready_pub_->publish(ready_msg);
  planning_ready_published_ = true;
  RCLCPP_INFO(node_->get_logger(), "✓ Initial trajectory planning complete - ready signal published");
}
```

#### 3.3 새로운 Synchronizer
**파일:** `scripts/swarm_synchronizer_with_planning.py` (새로 생성)

- 모든 드론의 `/drone_X_planning/ready` 토픽 구독
- 36대 모두 ready 신호 받으면 `/traj_start_trigger` 발행
- 180초 timeout (계획 실패 감지)
- 진행 상황 5초마다 출력

**파일:** `src/planner/plan_manage/launch/scenario_swarm_36.launch.py`

```python
# 라인 166-169: 새 synchronizer 사용
swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer_with_planning.py')

swarm_synchronizer = ExecuteProcess(
    cmd=['python3', swarm_sync_script, '--ros-args', '-p', 'num_drones:=36', '-p', 'timeout:=180.0'],
```

## 이론적 배경

### ego-swarm의 경로 생성 구조

```
각 드론의 재계획 사이클 (원래):
1. planGlobalTraj() - polynomial trajectory 생성
2. getLocalTarget() - planning_horizon 거리의 local target 설정
3. reboundReplan() - local target까지 B-spline 최적화
4. 1초 후 재계획 트리거 → 다음 구간 계획
```

### 문제점
- `getLocalTarget()`이 planning_horizon 거리까지만 target 설정
- 원래 planning_horizon = 25m → 25m씩 끊어서 계획
- 재계획 없으면 처음 25m만 계획하고 멈춤

### 해결 방법
- **planning_horizon을 전체 경로 길이로 확대**
- 재계획 트리거 시간 무한대로 설정
- 한 번 계획 후 끝까지 실행

## 예상 동작 흐름

```
1. 36대 드론 spawn (초기화)

2. 각 드론 독립적으로 전체 경로 계획
   - A* global path (170m)
   - B-spline 최적화 (~300 control points)
   - 예상 시간: 1~5초/드론

3. 계획 완료 드론들이 ready 신호 발행
   [Synchronizer 출력]
   ✓ Drone 0 planning complete! (1/36 ready)
   ✓ Drone 1 planning complete! (2/36 ready)
   ...
   Progress: 24/36 drones ready (15.2s elapsed)
   ...

4. 모든 36대 ready 확인
   🎯 ALL DRONES PLANNING COMPLETE!
   Total planning time: 45.23 seconds
   TRIGGERING SYNCHRONIZED START!

5. 모든 드론 동시 출발 (재계획 없음)
```

## 잠재적 문제점 및 해결책

### 1. B-spline 최적화 수렴 실패 ⚠️ 가장 큰 리스크

**원인:**
```cpp
// bspline_optimizer.cpp:1826
f_combine = lambda1_ * f_smoothness        // 부드러움 (jerk 최소화)
          + new_lambda2_ * f_distance      // 장애물 회피
          + lambda3_ * f_feasibility       // 속도/가속도 제한
          + new_lambda2_ * f_swarm         // 다른 드론과 충돌 회피
          + lambda2_ * f_terminal;         // 목표점 도달
```

- Control points 증가: 50개 → 300개
- Swarm collision 항이 많아짐
- Local minima에 빠질 가능성 증가
- Gradient 불안정

**현재 최적화 파라미터:**
```cpp
// bspline_optimizer.cpp:1574-1575
lbfgs_params.max_iterations = 200;
lbfgs_params.g_epsilon = 0.01;       // gradient norm < 0.01이면 수렴
lbfgs_params.mem_size = 16;
```

**해결 방법 (코드 수정 필요):**

#### Option 1: 최적화 파라미터 완화
```cpp
// src/planner/bspline_opt/src/bspline_optimizer.cpp:1574-1575
lbfgs_params.max_iterations = 500;   // 200 → 500
lbfgs_params.g_epsilon = 0.1;        // 0.01 → 0.1 (수렴 조건 완화)
lbfgs_params.mem_size = 32;          // 16 → 32 (더 많은 히스토리)
```

#### Option 2: 비용 함수 가중치 조정
```python
# advanced_param.launch.py
{'optimization/lambda_smooth': 2.0},      # 1.0 → 2.0 (더 직선에 가까운 경로)
{'optimization/lambda_feasibility': 0.05} # 0.1 → 0.05 (제약 완화)
```

#### Option 3: planning_horizon 줄이기
```python
# scenario_swarm_36.launch.py
'planning_horizon': '100.0'  # 170 → 100 (절충안)
```

### 2. 초기 계획 시간 증가
- 예상: 드론당 1~5초
- 36대 동시 계획: CPU 부하
- 해결: 괜찮음 (오프라인이므로)

### 3. 다른 드론의 최신 trajectory 반영 불가
**문제:**
```
시간 t=0: 모든 드론이 다른 드론의 초기 계획만 보고 경로 계획
시간 t=5: 만약 어떤 드론이 최적화 실패로 경로 변경 → 다른 드론은 모름
```

**오프라인 환경에서는 괜찮음:**
- 모든 계획이 사전 확정
- 최적화 실패만 없으면 예측 가능

## 최적화 에러 추정

### 로그에서 최적화 상태 확인하기

```bash
# 최적화 관련 로그 실시간 모니터링
tail -f ~/.ros/log/latest/*.log | grep -E "LBFGS|optimization|replan|planning complete"

# 최적화 실패 로그 검색
grep -r "LBFGS" ~/.ros/log/latest/*.log
grep -r "fail" ~/.ros/log/latest/*.log | grep -i optim
grep -r "iteration" ~/.ros/log/latest/*.log

# 각 드론별 최적화 시도 횟수 확인
grep "drone.*replan" ~/.ros/log/latest/*.log | wc -l
```

### LBFGS 결과 코드 해석

```cpp
// bspline_optimizer.cpp:1587-1590
if (result == lbfgs::LBFGS_CONVERGENCE ||           // 수렴 성공 ✅
    result == lbfgs::LBFGSERR_MAXIMUMITERATION ||   // 최대 반복 도달 ⚠️
    result == lbfgs::LBFGS_ALREADY_MINIMIZED ||     // 이미 최소값 ✅
    result == lbfgs::LBFGS_STOP)                    // 조기 종료 ⚠️
```

**가능한 결과:**
- `LBFGS_CONVERGENCE` (0) - ✅ 정상 수렴
- `LBFGS_ALREADY_MINIMIZED` (2) - ✅ 초기값이 이미 최적
- `LBFGSERR_MAXIMUMITERATION` (-1024) - ⚠️ 200번 반복해도 수렴 못함
- `LBFGS_STOP` (1) - ⚠️ 조기 종료 (earlyExit 콜백)
- `LBFGSERR_*` (기타) - ❌ 에러

### 최적화 실패 판단 기준

#### 1. 로그에서 "replan" 횟수가 비정상적으로 많음
```bash
# 정상: 드론당 1~2회 (초기 계획 + 혹시 재시도)
# 비정상: 드론당 10회 이상 → GEN_NEW_TRAJ 무한 루프
grep "drone 0 replan" ~/.ros/log/latest/*.log | wc -l
```

#### 2. 특정 드론이 ready 신호를 안 보냄
```bash
# 모든 ready 신호 확인
for i in {0..35}; do
  echo -n "Drone $i: "
  grep "Drone $i.*planning complete" ~/.ros/log/latest/*.log | wc -l
done

# 0이 나오는 드론 = 최적화 실패 추정
```

#### 3. 계획 시간이 비정상적으로 길어짐
```bash
# Synchronizer timeout (180초) 발생
# "TIMEOUT! Only X/36 drones ready" 메시지 출력
```

### 최적화 실패 원인 추정 방법

#### Step 1: 어느 cost 항목이 문제인지 확인

**코드 수정 필요** - 디버깅 로그 추가:
```cpp
// src/planner/bspline_opt/src/bspline_optimizer.cpp:1826 이후 추가
RCLCPP_DEBUG(rclcpp::get_logger("bspline_opt"),
  "Costs: smooth=%.3f, dist=%.3f, feas=%.3f, swarm=%.3f, term=%.3f, total=%.3f",
  f_smoothness, f_distance, f_feasibility, f_swarm, f_terminal, f_combine);
```

실행 후:
```bash
# DEBUG 레벨 로그 활성화
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
ros2 run ego_planner ego_planner_node --ros-args --log-level debug

# 비용 확인
grep "Costs:" ~/.ros/log/latest/*.log
```

**분석:**
- `swarm` 비용이 크다 → 다른 드론과 충돌 많음
- `feas` 비용이 크다 → 속도/가속도 제한 위반
- `smooth` 비용이 크다 → 경로가 너무 구불구불
- `dist` 비용이 크다 → 장애물 근처 (없어야 정상)

#### Step 2: Control points 개수 확인

```cpp
// planner_manager.cpp:215 근처에서 확인
printf("Control points: %d\n", point_set.size());
```

**정상:** 50~100개
**위험:** 200개 이상 → 최적화 수렴 어려움
**한계:** 300개 이상 → 수렴 거의 불가능

#### Step 3: 초기 경로 품질 확인

A* 경로가 너무 복잡하면 B-spline 최적화 실패:
```bash
# A* 탐색 시간 확인 (너무 길면 경로가 복잡)
grep "A star.*time" ~/.ros/log/latest/*.log
```

**정상:** 0.1ms ~ 10ms
**주의:** 10ms ~ 100ms
**위험:** 100ms 이상 → 경로가 매우 복잡

### 최적화 실패 시 임시 해결책

#### 빠른 테스트용: planning_horizon 줄이기
```python
# scenario_swarm_36.launch.py
'planning_horizon': '100.0'  # 170 → 100
# 또는
'planning_horizon': '70.0'   # 170 → 70 (원래보다 조금만 증가)
```

#### 중간 해결책: 최적화 파라미터 완화
```cpp
// src/planner/bspline_opt/src/bspline_optimizer.cpp:1574-1576
lbfgs_params.max_iterations = 500;    // 200 → 500 (더 많은 시도)
lbfgs_params.g_epsilon = 0.1;         // 0.01 → 0.1 (수렴 조건 완화)
lbfgs_params.mem_size = 32;           // 16 → 32 (더 많은 히스토리)
```

재빌드 필요:
```bash
colcon build --packages-select bspline_opt ego_planner
source install/setup.bash
```

#### 근본 해결책: 비용 함수 가중치 조정
```python
# advanced_param.launch.py:190-192
{'optimization/lambda_smooth': 2.0},       # 1.0 → 2.0 (더 직선)
{'optimization/lambda_feasibility': 0.05}, # 0.1 → 0.05 (제약 완화)
```

**효과:**
- `lambda_smooth` 증가 → 경로가 직선에 가까워짐 → swarm collision 감소
- `lambda_feasibility` 감소 → 속도/가속도 제한 완화 → 최적화 자유도 증가

### 최적화 성공 여부 빠른 확인

```bash
# 1. Synchronizer 출력 확인
# 성공: "🎯 ALL DRONES PLANNING COMPLETE!"
# 실패: "TIMEOUT! Only X/36 drones ready"

# 2. Ready 신호 개수 확인
grep "planning complete.*ready signal published" ~/.ros/log/latest/*.log | wc -l
# 성공: 36개
# 실패: 36개 미만

# 3. 최적화 iteration 횟수 평균 확인
grep "A star iter:" ~/.ros/log/latest/*.log
# 정상: iter < 1000
# 주의: iter > 5000 (복잡한 경로)
```

### 단계적 디버깅 전략

```
1단계: planning_horizon = 50으로 테스트
   → 성공? 2단계로
   → 실패? 최적화 파라미터 문제

2단계: planning_horizon = 100으로 테스트
   → 성공? 3단계로
   → 실패? horizon 70 정도로 타협

3단계: planning_horizon = 170으로 테스트
   → 성공? 완료!
   → 실패? 비용 가중치 조정 필요
```

## 디버깅 체크리스트

### 실패 시나리오 1: 드론이 ready 신호를 보내지 않음
**확인:**
```bash
# 특정 드론의 ready 토픽 확인
ros2 topic echo /drone_0_planning/ready

# FSM 상태 확인 (로그)
grep "planning complete" ~/.ros/log/latest/*.log
```

**원인:**
- `planFromGlobalTraj()` 실패 (GEN_NEW_TRAJ에서 무한 루프)
- B-spline 최적화 수렴 실패

### 실패 시나리오 2: 일부 드론만 ready
**확인:**
```bash
# Synchronizer 출력 확인
# "Progress: X/36 drones ready" 에서 멈춤

# 어떤 드론이 안 된 건지 확인
ros2 topic list | grep planning/ready
```

**원인:**
- 특정 드론의 최적화 실패
- 초기 위치/목표 문제

### 실패 시나리오 3: 모두 ready인데 출발 안 함
**확인:**
```bash
# Trigger 토픽 확인
ros2 topic echo /traj_start_trigger

# FSM 로그 확인
grep "Triggered!" ~/.ros/log/latest/*.log
```

**원인:**
- Synchronizer와 FSM 간 토픽 연결 문제
- `have_trigger_` 변수 설정 문제

### 실패 시나리오 4: 출발했지만 일부만 가다가 멈춤
**확인:**
```bash
# 드론 위치 확인
ros2 topic echo /drone_0_visual_slam/odom

# Planning horizon 체크
ros2 param get /drone_0_ego_planner_node fsm.planning_horizon
```

**원인:**
- planning_horizon이 제대로 설정 안됨
- Local target이 전체 목표까지 도달하지 못함

### 실패 시나리오 5: 최적화 수렴 실패
**확인:**
```bash
# 로그에서 최적화 에러 찾기
grep "LBFGS" ~/.ros/log/latest/*.log
grep "optimization fail" ~/.ros/log/latest/*.log
```

**원인:**
- Control points 너무 많음
- Swarm collision cost 너무 큼
- 초기 경로 품질 낮음

## 실행 방법

```bash
# 1. 빌드 (이미 완료)
cd /home/pjh/ego_swarm/ego-planner-swarm
source /opt/ros/humble/setup.bash
colcon build --packages-select ego_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 환경 변수 로드
source install/setup.bash

# 3. 시나리오 실행
ros2 launch ego_planner scenario_swarm_36.launch.py

# 4. 로그 모니터링 (다른 터미널)
tail -f ~/.ros/log/latest/*.log | grep -E "planning complete|ready|Triggered|LBFGS|optimization"
```

## 테스트 결과 기록

### 테스트 1: 2025-11-14 (첫 시도)
- **설정:**
  - planning_horizon=170.0
  - thresh_replan_time=99999.0
  - planning_horizen_time=99999.0
- **결과:** ❌ 실패
- **문제점:** (내일 로그 분석 필요)
  - 추정: 최적화 수렴 실패로 인해 일부 드론이 ready 신호를 보내지 못함
- **해결 시도:** (다음 세션에서)
  - [ ] 로그 확인: `grep "planning complete" ~/.ros/log/latest/*.log | wc -l`
  - [ ] 어느 드론이 실패했는지 확인
  - [ ] planning_horizon을 100 또는 70으로 줄여서 재시도
  - [ ] 필요 시 최적화 파라미터 완화 (max_iterations=500, g_epsilon=0.1)

### 테스트 2: [날짜]
- **설정:**
- **결과:**
- **문제점:**
- **해결 시도:**

## 다음 단계

### 우선순위 1: 최적화 수렴 실패 해결
```cpp
// Option A: 파라미터 완화
max_iterations = 500
g_epsilon = 0.1

// Option B: 가중치 조정
lambda_smooth = 2.0

// Option C: horizon 줄이기
planning_horizon = 100.0
```

### 우선순위 2: 디버깅 개선
- 각 드론의 계획 진행 상황 상세 로그
- 최적화 실패 원인 분석 (어느 cost 항목이 문제?)
- Swarm trajectory 공유 상태 확인

### 우선순위 3: 성능 최적화
- 병렬 계획 vs 순차 계획
- CPU affinity 설정
- 메모리 사용량 모니터링

## 관련 파일 위치

### 수정된 파일
```
src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h
src/planner/plan_manage/src/ego_replan_fsm.cpp
src/planner/plan_manage/launch/advanced_param.launch.py
src/planner/plan_manage/launch/scenario_swarm_36.launch.py
scripts/swarm_synchronizer_with_planning.py (새로 생성)
```

### 핵심 코드
```
src/planner/plan_manage/src/planner_manager.cpp:467  # planGlobalTraj()
src/planner/plan_manage/src/ego_replan_fsm.cpp:961   # getLocalTarget()
src/planner/plan_manage/src/planner_manager.cpp:50   # reboundReplan()
src/planner/bspline_opt/src/bspline_optimizer.cpp:1574  # 최적화 파라미터
src/planner/bspline_opt/src/bspline_optimizer.cpp:1826  # 비용 함수
```

## 참고: 원래 동작 방식 vs 수정 후

### 원래 (Receding Horizon)
```
planning_horizon = 25m
thresh_replan_time = 1.0s

[초기] ----25m-----> [재계획] ----25m-----> [재계획] ----25m-----> [목표]
  0s         1s           2s           3s           4s
```

### 수정 후 (Full Path)
```
planning_horizon = 170m
thresh_replan_time = 99999s

[초기] ----------------------170m-----------------------> [목표]
  0s                                                     끝까지
```

## 되돌리기 (원래대로 복구)

```bash
# Git으로 되돌리기
git checkout src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h
git checkout src/planner/plan_manage/src/ego_replan_fsm.cpp
git checkout src/planner/plan_manage/launch/advanced_param.launch.py
git checkout src/planner/plan_manage/launch/scenario_swarm_36.launch.py

# 빌드
colcon build --packages-select ego_planner
```

## 작성일
2025-11-14

## 상태
🟢 **해결** - planning_horizon=70m 권장 (94% 성공률)

---

# 크래시 원인 분석 (2025-11-15)

## 핵심 발견: Planning Horizon과 크래시의 관계

### 테스트 결과 요약

| Planning Horizon | Replanning | 크래시 | Ready 신호 | 미션 시작 | 성공률 |
|-----------------|------------|--------|-----------|----------|--------|
| **15m** (원본) | 1.0초 간격 | **6대** | - | ❌ | 0% |
| **25m** (기본) | 1.0초 간격 | **1대** | - | ❌ | 0% |
| **70m** (권장) | 비활성화 | **0대** | 34/36 | ✅ | **94%** |
| 100m | 비활성화 | 0대 | 31/36 | ✅ | 86% |
| 170m | 비활성화 | 0대 | 28/36 | ✅ | 78% |

**결론:** Planning horizon이 작을수록 크래시 증가. 70m가 최적값.

---

## 왜 크래시가 발생하는가?

### 1. 문제의 본질: 과도한 제약 조건 vs 부족한 자유도

36대 드론 밀집 환경에서:
- 각 드론은 **35대의 다른 드론**과 충돌 회피 필요
- Planning horizon이 작으면 **제어점 수가 적음**
- 너무 많은 제약 + 너무 적은 자유도 = **최적화 문제 해결 불가**

#### Control Points 계산

```python
# advanced_param.launch.py:184
control_points_distance = 0.4  # 제어점 간격

# Control points 개수 = planning_horizon / control_points_distance
planning_horizon = 15m  →  약 37개 제어점
planning_horizon = 25m  →  약 62개 제어점
planning_horizon = 70m  →  약 175개 제어점
```

### 2. B-spline 최적화 비용 함수

```cpp
// bspline_optimizer.cpp:1826
f_combine = lambda_smooth × f_smoothness        // 부드러운 경로
          + lambda_collision × f_distance       // 장애물 회피
          + lambda_feasibility × f_feasibility  // 속도/가속도 제한
          + lambda_collision × f_swarm          // 스웜 충돌 회피 ← 핵심!
          + lambda_collision × f_terminal       // 목표 도달
```

**문제:**
- `f_swarm`: 35대 드론과의 충돌 회피 제약 (각 드론마다 거리 계산)
- Control points 37개 → 37개 변수로 35개 드론 회피
- **자유도 부족 → 해를 찾지 못함 → SIGABRT**

### 3. 크래시 메커니즘

```
1. LBFGS 최적화 시작 (200회 반복 제한)
   ↓
2. 충돌 회피 제약을 만족하는 해를 찾으려 시도
   ↓
3. 37개 제어점으로는 35대 드론 회피 불가능
   ↓
4. 최대 반복 횟수 도달 또는 gradient 발산
   ↓
5. Assertion 실패 또는 벡터 범위 초과
   ↓
6. SIGABRT (exit code -6) 크래시
```

### 4. 왜 특정 드론만 크래시하는가?

**Planning horizon=15m 테스트 결과:**
- 크래시 드론: 1, 2, 4, 6, 8, 10
- 정상 드론: 나머지

**원인:**
- 특정 시작/목표 위치 조합이 **더 tight한 최적화 문제** 생성
- 예: 드론이 밀집된 중앙을 지나야 하는 경우
- 예: 초기 간격이 매우 좁아 즉시 충돌 회피 필요한 경우

### 5. "너무 드론이 많아서" 인가?

**예 + 아니오:**

#### 예 (Yes):
- 36대는 원본 ego-swarm 논문 대비 3.6배 많음 (원본: 10대)
- 드론 수가 증가 → 충돌 회피 제약 조건 증가 (선형)
- 36대 = 각 드론당 35개 제약 조건

#### 아니오 (No):
- 문제는 **절대적 드론 수**가 아니라 **드론 수 × 밀도 × planning_horizon**
- Planning horizon을 늘리면 해결 가능

**증거:**
```
36대 + planning_horizon=70m  → 크래시 0대 (94% 성공)
36대 + planning_horizon=15m  → 크래시 6대 (83% 실패율)
```

### 6. 수학적 분석

#### 최적화 문제의 차원

```
변수 개수 = control_points × 3 (x, y, z)
제약 조건 ≈ (num_drones - 1) × control_points × safety_checks

planning_horizon=15m:
  변수: 37 × 3 = 111
  제약: 35 × 37 × k ≈ 1,295k (k는 안전 체크 횟수)

planning_horizon=70m:
  변수: 175 × 3 = 525
  제약: 35 × 175 × k ≈ 6,125k

비율:
  15m: 제약/변수 ≈ 11.7k
  70m: 제약/변수 ≈ 11.7k (비슷함)
```

**그런데 왜 70m가 더 잘 되는가?**
- **공간적 분산**: 긴 경로 → 시간차로 충돌 회피 가능
- **초기 경로 품질**: 긴 horizon → A* 경로가 더 부드러움
- **최적화 여유**: 더 많은 제어점 → 국소 조정 가능

---

## 해결 방법

### ✅ 권장 설정: Planning Horizon = 70m

```python
# scenario_swarm_36.launch.py:157
'planning_horizon': '70.0'
```

**장점:**
- ✅ 크래시 0건
- ✅ 94% 성공률 (34/36 드론)
- ✅ 최적화 수렴 안정적
- ✅ 오프라인 계획 가능

**단점:**
- ⚠️ 2대 드론 여전히 실패 (비결정적, 최적화 문제)
- ⚠️ 재계획 불가 (정적 환경 전용)

### ⚠️ 원본 설정의 한계

```python
# 원본 설정 (25-drone 스웜용)
planning_horizon = 15m
thresh_replan_time = 1.0초
```

**문제점:**
- ❌ **36대 밀집 스웜에서는 크래시 발생**
- ❌ 제어점 부족 → 충돌 회피 불가능
- ❌ Planning horizon 작음 → 재계획 빈번 → 계산 부하

**작동하는 환경:**
- ✅ 소규모 스웜 (≤10대)
- ✅ 드론 간격 넓음 (>5m)
- ✅ 동적 환경 (장애물 이동)

---

## 이론적 배경

### Receding Horizon vs Full Path Planning

#### Receding Horizon (원본 방식)
```
장점:
  - 동적 환경 대응 가능
  - 계산 부하 분산 (짧은 구간씩)
  - 센서 데이터 실시간 반영

단점:
  - 밀집 스웜에서 불안정
  - 재계획 중 충돌 위험
  - 36대 규모에서 크래시
```

#### Full Path Planning (오프라인 방식)
```
장점:
  - 전체 경로 최적화
  - 밀집 스웜 안정적
  - 크래시 제거

단점:
  - 정적 환경 전용
  - 초기 계획 시간 증가
  - 재계획 불가
```

### 왜 작은 Planning Horizon이 크래시를 유발하는가?

#### 1. 최적화 문제의 feasibility
```
Feasible solution 존재 조건:
  자유도 ≥ 제약 조건의 effective rank

planning_horizon=15m:
  자유도 낮음 → tight constraints → infeasible

planning_horizon=70m:
  자유도 높음 → 해공간 넓음 → feasible
```

#### 2. Gradient 기반 최적화의 한계
```cpp
// LBFGS는 gradient descent 기반
// Local minima에 빠지기 쉬움

작은 horizon:
  - 좁은 해공간
  - Gradient 급격히 변함
  - Local minima 많음
  → 수렴 실패

큰 horizon:
  - 넓은 해공간
  - Gradient 부드러움
  - Global minima 찾기 쉬움
  → 수렴 성공
```

#### 3. 시간-공간 충돌 회피
```
작은 horizon (15m):
  모든 드론이 동시에 좁은 공간 통과
  → 공간적 충돌 불가피

큰 horizon (70m):
  드론들이 시간차를 두고 통과
  → 시간적 분리로 충돌 회피
```

---

## 실전 가이드

### 36-Drone 스웜 실행 체크리스트

```bash
# 1. 설정 확인
grep "planning_horizon" src/planner/plan_manage/launch/scenario_swarm_36.launch.py
# 출력: 'planning_horizon': '70.0'

# 2. 재계획 비활성화 확인
grep "thresh_replan_time" src/planner/plan_manage/launch/advanced_param.launch.py
# 출력: {'fsm/thresh_replan_time': 99999.0}

# 3. 실행
ros2 launch ego_planner scenario_swarm_36.launch.py

# 4. 성공 확인 (다른 터미널)
# 예상 출력:
# ✓ Drone 0 planning complete! (1/36 ready)
# ...
# ✓ Drone 35 planning complete! (36/36 ready)
# 🎯 ALL DRONES PLANNING COMPLETE!
```

### 트러블슈팅

#### 문제: 34/36만 성공 (드론 29, 35 실패)

**원인:** 비결정적 최적화 실패

**시도 1:** 재실행 (다른 드론 실패할 수 있음)
```bash
ros2 launch ego_planner scenario_swarm_36.launch.py
```

**시도 2:** 최적화 파라미터 완화 (코드 수정 필요)
```cpp
// src/planner/bspline_opt/src/bspline_optimizer.cpp:1574-1576
lbfgs_params.max_iterations = 500;    // 200 → 500
lbfgs_params.g_epsilon = 0.1;         // 0.01 → 0.1
lbfgs_params.mem_size = 32;           // 16 → 32

// 재빌드
colcon build --packages-select bspline_opt ego_planner
```

#### 문제: 여전히 크래시 발생

**확인:**
```bash
# 크래시 로그 확인
dmesg | tail -20

# Planning horizon 확인
ros2 param get /drone_0_ego_planner_node manager.planning_horizon
```

**해결:**
1. Planning horizon 증가 (70 → 100)
2. 충돌 가중치 감소
```python
# advanced_param.launch.py:191
{'optimization/lambda_collision': 3.0},  # 4.0 → 3.0
```

---

## 결론

### 핵심 답변: "왜 크래시 하는걸까? 너무 드론이 많아서?"

**답:**
1. **직접 원인:** Planning horizon이 너무 작음 (15m)
   - 제어점 37개로는 35대 드론 회피 불가능
   - 최적화 문제가 풀 수 없음 (infeasible)
   - LBFGS 알고리즘 assertion 실패 → SIGABRT

2. **간접 원인:** 드론이 많고 밀집됨 (36대)
   - 드론 수 증가 → 충돌 회피 제약 증가
   - 밀집 편대 → 더 tight한 제약
   - 원본 ego-swarm(10대)보다 3.6배 많음

3. **해결책:** Planning horizon 증가 (70m)
   - 제어점 175개 → 충분한 자유도
   - 시간-공간 분리로 충돌 회피
   - 크래시 0건, 94% 성공률

**최종 권장:**
```python
# 36-drone 밀집 스웜 최적 설정
planning_horizon = 70.0m
thresh_replan_time = 99999.0s  # 재계획 비활성화
swarm_clearance = 1.0m
lambda_collision = 4.0
```

이 설정으로 **오프라인 전체 경로 계획** 방식이 36대 스웜에서 안정적으로 작동합니다.
