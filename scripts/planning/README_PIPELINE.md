# Formation Planner Pipeline

통합 포메이션 플래닝 프레임워크 - 포메이션 입력부터 궤적 출력까지 자동화

## 전체 구조

```
┌─────────────────────────────────┐
│   1. Formation Input            │
│   - Preset (10/25/36)           │
│   - Custom files                │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│   2. Fair Hungarian Allocation  │
│   - 공평한 드론-목표 할당        │
│   - 최대 거리 최소화             │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│   3. Launch File Generation     │
│   - 동적 launch 파일 생성        │
│   - Ego-Swarm 설정               │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│   4. Ego-Swarm Simulation       │
│   - 충돌 회피 궤적 생성          │
│   - B-spline 궤적 publish        │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│   5. Trajectory Recorder        │
│   - node_1.txt ~ node_N.txt     │
│   - 날짜_시간 폴더               │
└─────────────────────────────────┘
```

## 사용 방법

### 1. 기본 사용 (Preset 포메이션)

**36대 드론, Grid → Triangle:**
```bash
cd /home/pjh/ego_swarm/ego-planner-swarm/scripts
python3 formation_planner_pipeline.py --num_drones 36
```

**25대 드론:**
```bash
python3 formation_planner_pipeline.py --num_drones 25
```

**10대 드론:**
```bash
python3 formation_planner_pipeline.py --num_drones 10
```

### 2. Custom 포메이션 사용

**포메이션 파일 형식** (`my_formation_a.txt`):
```
3.0,10.0,15.0
3.0,12.0,15.0
3.0,14.0,15.0
...
```

**실행:**
```bash
python3 formation_planner_pipeline.py \
    --current my_formation_a.txt \
    --target my_formation_b.txt \
    --output_folder my_test
```

### 3. 전체 워크플로우

**Step 1: 계획 생성**
```bash
cd /home/pjh/ego_swarm/ego-planner-swarm/scripts
python3 formation_planner_pipeline.py --num_drones 36 --output_folder scenarios_FHA_36
```

출력:
```
======================================================================
Formation Planner Pipeline - Full Run
======================================================================

======================================================================
Loading preset formations for 36 drones
======================================================================
✓ Current formation: Grid 36 drones
✓ Target formation: Triangle 36 drones

======================================================================
Running Fair Hungarian Allocation
======================================================================
Fair allocation converged at iteration 12

=== Fair Allocation Result ===
Drone 0 → Goal 14: 12.34m
Drone 1 → Goal 5: 8.92m
...
✓ Allocation completed

======================================================================
Generating launch file: generated_swarm.launch.py
======================================================================
✓ Launch file generated

======================================================================
Next Steps:
======================================================================

1. Start simulation (Terminal 1):
   cd /home/pjh/ego_swarm/ego-planner-swarm
   source install/setup.bash
   ros2 launch src/planner/plan_manage/launch/generated_swarm.launch.py

2. Start trajectory recorder (Terminal 2):
   cd /home/pjh/ego_swarm/ego-planner-swarm
   source install/setup.bash
   ros2 launch traj_recorder record_trajectory.launch.py \
       num_drones:=36 \
       output_folder:=scenarios_FHA_36 \
       sampling_dt:=0.05

3. Wait for simulation to complete, then press Ctrl+C

4. Check output folder: scenarios_FHA_36_YYYYMMDD_HHMMSS/
======================================================================
```

**Step 2: 터미널 1 - 시뮬레이션 실행**
```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
source install/setup.bash
ros2 launch src/planner/plan_manage/launch/generated_swarm.launch.py
```

**Step 3: 터미널 2 - 궤적 기록**
```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
source install/setup.bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=scenarios_FHA_36 \
    sampling_dt:=0.05
```

**Step 4: 시뮬레이션 완료 후 Ctrl+C로 종료**

**Step 5: 결과 확인**
```bash
ls -lh scenarios_FHA_36_*/
head -20 scenarios_FHA_36_*/node_1.txt
```

## 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `--num_drones` | Preset 포메이션 (10/25/36) | Required (또는 --current) |
| `--current` | 현재 포메이션 파일 | - |
| `--target` | 목표 포메이션 파일 (--current와 함께) | - |
| `--output_folder` | 궤적 출력 폴더 base 이름 | trajectories |
| `--sampling_dt` | 샘플링 시간 간격 (초) | 0.05 |
| `--mode` | 실행 모드 (plan/sim/record/full) | plan |
| `--run_time` | 자동 종료 시간 (초) | - |

## 예시 시나리오

### 시나리오 1: 36대 Grid → Triangle

```bash
# 1. 계획 생성
python3 formation_planner_pipeline.py --num_drones 36 --output_folder test_36

# 2. 터미널 1: 시뮬레이션
cd ..
source install/setup.bash
ros2 launch src/planner/plan_manage/launch/generated_swarm.launch.py

# 3. 터미널 2: 기록
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=test_36

# 4. 종료 후 확인
ls test_36_*/
```

### 시나리오 2: Custom 포메이션

**포메이션 파일 생성:**
```bash
# formation_circle.txt - 원형 포메이션 (5대)
cat > formation_circle.txt << EOF
3.0,20.0,15.0
3.0,22.0,17.0
3.0,20.0,19.0
3.0,18.0,17.0
3.0,20.0,15.0
EOF

# formation_line.txt - 일렬 포메이션 (5대)
cat > formation_line.txt << EOF
3.0,30.0,15.0
3.0,32.0,15.0
3.0,34.0,15.0
3.0,36.0,15.0
3.0,38.0,15.0
EOF
```

**실행:**
```bash
python3 formation_planner_pipeline.py \
    --current formation_circle.txt \
    --target formation_line.txt \
    --output_folder circle_to_line
```

### 시나리오 3: 빠른 샘플링

```bash
# 20ms 간격 샘플링 (더 촘촘한 궤적)
python3 formation_planner_pipeline.py \
    --num_drones 36 \
    --output_folder detailed_traj \
    --sampling_dt 0.02
```

## 출력 구조

```
scenarios_FHA_36_20241031_143025/
├── node_1.txt    # 드론 1 궤적
├── node_2.txt    # 드론 2 궤적
├── ...
└── node_36.txt   # 드론 36 궤적
```

각 파일 형식:
```
1,1,0.00,move,3.00,5.00,10.00,0.0,255,255,255
2,1,0.05,move,3.01,5.12,10.05,0.0,255,255,255
3,1,0.10,move,3.02,5.24,10.10,0.0,255,255,255
...
```

## 기존 코드 활용

이 파이프라인은 다음 기존 모듈들을 통합합니다:

1. **Fair_Hungarian_Allocator.py**
   - 공평한 헝가리안 할당
   - 최대 거리 최소화 (Min-Max 최적화)

2. **formations_large.py**
   - 10/25/36대 Preset 포메이션
   - Grid, Triangle 형태

3. **scenario_swarm.launch.py**
   - Ego-Swarm 기반
   - 충돌 회피 궤적 생성

4. **traj_recorder**
   - ROS2 노드
   - B-spline 궤적 샘플링 및 저장

## Preset 포메이션 상세

### 10대 드론
- **Current**: S 형태
- **Target**: U 형태

### 25대 드론
- **Current**: 5x5 Grid (간격 4.0m)
- **Target**: Triangle (간격 3.2m)

### 36대 드론
- **Current**: 6x6 Grid (Y 간격 8.0m, Z 간격 5.0m)
- **Target**: Triangle (간격 3.2m, 1+3+5+7+9+11 rows)

## 문제 해결

### 1. "Module not found: scipy"
```bash
pip3 install scipy numpy
```

### 2. Launch 파일 생성 실패
- 쓰기 권한 확인
- src/planner/plan_manage/launch/ 디렉토리 존재 확인

### 3. 포메이션 파일 형식 오류
- 각 줄: `x,y,z` (콤마로 구분)
- 주석 라인: `#`로 시작
- 빈 줄 허용

### 4. 드론 수 불일치
```
ValueError: Formation size mismatch: 10 vs 12
```
→ current와 target 포메이션의 드론 수가 같아야 함

## 향후 계획 (GUI)

현재는 CLI 버전이지만, 향후 GUI로 확장 예정:

```
┌─────────────────────────────────────────────┐
│           Formation Planner GUI              │
├─────────────────────────────────────────────┤
│  Number of Drones:   [36        ]           │
│                                               │
│  Current Formation:  [Browse...] [Preset ▼] │
│  Target Formation:   [Browse...] [Preset ▼] │
│                                               │
│  Output Folder:      [scenarios_test     ]  │
│  Sampling dt:        [0.05          ] sec   │
│                                               │
│  [ Preview ] [ Generate Plan ] [ Run All ]  │
└─────────────────────────────────────────────┘
```

## 요약

✅ **1단계**: 포메이션 입력 (Preset 또는 파일)
✅ **2단계**: Fair Hungarian 할당
✅ **3단계**: Launch 파일 자동 생성
✅ **4단계**: Ego-Swarm 시뮬레이션
✅ **5단계**: 궤적 TXT 파일 출력

**단일 명령으로 1~3단계 완료, 4~5단계는 수동 실행**
