# Assignment Management

드론 형상 할당(assignment)을 관리하는 디렉토리입니다.

## 📁 디렉토리 구조

```
Allocation/
├── Fair_Hungarian_25/
│   └── assignment_25_drones.txt    # Hungarian 알고리즘으로 계산된 25대 할당
├── Fair_Hungarian_36/
│   └── assignment_36_drones.txt    # Hungarian 알고리즘으로 계산된 36대 할당
├── CATORA_25/
│   └── assignment_25_drones.txt    # CAT-ORA 알고리즘으로 계산된 25대 할당
├── CATORA_36/
│   └── assignment_36_drones.txt    # CAT-ORA 알고리즘으로 계산된 36대 할당
├── Fair_Hungarian_Allocator.py    # Hungarian 알고리즘 구현
├── CATORA_Allocator.py             # CAT-ORA 알고리즘 인터페이스
└── README.md                       # 이 파일
```

## 🎯 알고리즘 비교

### Fair Hungarian Algorithm
- **목표**: Min-Max 최적화 (최대 거리 최소화)
- **특징**:
  - 배터리 소모 균등화
  - 공정한 할당
  - 순수 거리 기반 (충돌 미고려)
- **계산 시간**: 빠름 (~50-100ms)
- **장점**: 빠르고 구현이 간단
- **단점**: 궤적 충돌 가능성

### CAT-ORA Algorithm
- **목표**: Collision-Aware Time-Optimal (충돌 회피 + 시간 최적)
- **특징**:
  - 궤적 충돌 감지
  - Bottleneck 최소화
  - Branch & Bound 탐색
- **계산 시간**: 느림 (~100-500ms)
- **장점**: 충돌 없는 안전한 할당
- **단점**: 계산 비용이 높음

## 🚀 사용 방법

### 1. Hungarian 알고리즘 사용

```bash
# 25대 드론 할당 생성
python3 generate_assignment.py --algorithm hungarian --num_drones 25

# 36대 드론 할당 생성
python3 generate_assignment.py --algorithm hungarian --num_drones 36
```

**출력**:
```
================================================================================
HUNGARIAN Assignment for 25 Drones
Algorithm: Fair Hungarian (min-max distance)
Formation: 5x5 Grid -> Triangle
================================================================================

Generating assignment...
...
✓ Assignment saved to: .../Allocation/Fair_Hungarian_25/assignment_25_drones.txt
```

### 2. CAT-ORA 알고리즘 사용 ⭐ NEW: Standalone Mode

**✨ Standalone Mode (추천)**: ROS2 서비스 없이 바로 실행!

```bash
# ROS2 서비스 불필요! 바로 실행 가능
python3 scripts/generate_assignment.py --algorithm catora --num_drones 25
python3 scripts/generate_assignment.py --algorithm catora --num_drones 36
```

**출력**:
```
================================================================================
CATORA Assignment for 25 Drones
Algorithm: CAT-ORA (collision-aware time-optimal)
Formation: 5x5 Grid -> Triangle
================================================================================

Generating assignment...
CAT-ORA Standalone mode initialized (no ROS2 required)
Computing CAT-ORA assignment for 25 drones... (Standalone mode)
Initial Hungarian assignment: bottleneck = 15.93m
Collision detected! Running Branch & Bound...
Final assignment: bottleneck = 15.93m
CAT-ORA assignment computed in 54.0ms

✓ Assignment saved to: .../Allocation/CATORA_25/assignment_25_drones.txt

Statistics:
  Min distance: 3.10m
  Max distance: 15.93m
  Avg distance: 11.43m
```

**🔧 Advanced: ROS2 Service Mode (Optional)**

원본 C++ CAT-ORA 구현을 사용하려면:

```bash
# Terminal 1: CAT-ORA 플래너 실행
ros2 launch catora_planner catora_planner.launch.py

# Terminal 2: ROS2 서비스 모드로 할당 생성
# Note: generate_assignment.py를 수정하여 use_ros2_service=True 설정 필요
```

### 3. 기존 할당 재사용

```bash
# 이미 생성된 할당이 있으면 자동으로 재사용
python3 generate_assignment.py --algorithm hungarian --num_drones 36

# 출력:
# ✓ Existing assignment found: .../Fair_Hungarian_36/assignment_36_drones.txt
# Using existing assignment (use --force to regenerate)
```

### 4. 강제 재생성

```bash
# 형상을 변경했을 때 재생성
python3 generate_assignment.py --algorithm hungarian --num_drones 36 --force

# CAT-ORA도 마찬가지
python3 generate_assignment.py --algorithm catora --num_drones 25 --force
```

### 5. 모든 할당 확인

```bash
python3 generate_assignment.py --list
```

**출력 예시**:
```
================================================================================
Existing Assignments:
================================================================================
  [Fair Hungarian] 25 drones: .../Allocation/Fair_Hungarian_25/assignment_25_drones.txt
    Last modified: 2025-11-19 21:12:00
  [Fair Hungarian] 36 drones: .../Allocation/Fair_Hungarian_36/assignment_36_drones.txt
    Last modified: 2025-11-19 21:12:00
  [CAT-ORA] 25 drones: .../Allocation/CATORA_25/assignment_25_drones.txt
    Last modified: 2025-11-19 21:15:00
================================================================================
```

## 📋 할당 파일 형식

각 할당 파일은 다음 정보를 포함합니다:

```
================================================================================
HUNGARIAN Assignment for 25 Drones
Algorithm: Fair Hungarian (min-max distance)
Generated: 2025-11-19 21:12:00
Formation: 5x5 Grid -> Triangle
================================================================================

Drone Configurations for scenario_swarm_25.launch.py:
================================================================================

drone_configs = [
    {'drone_id': 0, 'init_x': 0.0, 'init_y': 8.0, 'init_z': 10.0, 'target_x': 20.7, 'target_y': 16.5, 'target_z': 3.0},  # Distance: 15.23m
    {'drone_id': 1, 'init_x': 0.0, 'init_y': 8.0, 'init_z': 14.0, 'target_x': 27.1, 'target_y': 13.3, 'target_z': 3.0},  # Distance: 19.45m
    ...
]

================================================================================
Assignment Mapping (Drone ID -> Target ID):
================================================================================
Drone  0 -> Target  5
Drone  1 -> Target  8
...

================================================================================
Statistics:
================================================================================
Algorithm: HUNGARIAN
Total drones: 25
Min distance: 12.34m
Max distance: 25.67m
Avg distance: 18.45m
```

## 🔄 워크플로우

### 시나리오 1: Hungarian 할당 사용

```bash
# 1. 할당 생성 (또는 재사용)
python3 generate_assignment.py --algorithm hungarian --num_drones 36

# 2. 시뮬레이션 실행
ros2 launch ego_planner scenario_swarm_36.launch.py
```

### 시나리오 2: CAT-ORA 할당 사용

```bash
# 1. CAT-ORA 플래너 실행
ros2 launch catora_planner catora_planner.launch.py

# 2. 할당 생성
source install/setup.bash
python3 generate_assignment.py --algorithm catora --num_drones 25

# 3. 시뮬레이션 실행 (CAT-ORA 할당 사용)
ros2 launch catora_planner scenario_catora_25.launch.py
```

### 시나리오 3: 형상 변경 후 재생성

```bash
# 1. formations_large.py 수정
vim scripts/formations_large.py
# FORMATION_36_A 또는 FORMATION_36_B 수정

# 2. 두 알고리즘 모두 재생성
python3 generate_assignment.py --algorithm hungarian --num_drones 36 --force
python3 generate_assignment.py --algorithm catora --num_drones 36 --force

# 3. 결과 비교
cat Allocation/Fair_Hungarian_36/assignment_36_drones.txt
cat Allocation/CATORA_36/assignment_36_drones.txt
```

## 🛠️ CAT-ORA 설정

### CAT-ORA 플래너 설치 및 실행

```bash
# 1. 빌드
cd ~/ego_swarm/ego-planner-swarm
colcon build --packages-select catora_planner
source install/setup.bash

# 2. 실행
ros2 launch catora_planner catora_planner.launch.py

# 3. 파라미터 조정 (선택사항)
ros2 launch catora_planner catora_planner.launch.py \
    max_velocity:=3.0 \
    max_acceleration:=2.5 \
    trajectory_dt:=0.1
```

### 서비스 확인

```bash
# CAT-ORA 서비스가 실행 중인지 확인
ros2 service list | grep catora

# 출력:
# /catora_planner/get_assignment
# /catora_planner/get_reshaping_trajectories
```

## 📊 성능 비교 예시 (Updated with Standalone Mode)

| 알고리즘 | 드론 수 | 계산 시간 | Max 거리 | Avg 거리 | 충돌 검사 | ROS2 필요 |
|---------|--------|----------|---------|---------|----------|-----------|
| Hungarian | 25 | ~50ms | 16.20m | 11.92m | ❌ | ❌ |
| **CAT-ORA Standalone** | 25 | **~54ms** | **15.93m** | **11.43m** | ✅ | ❌ |
| Hungarian | 36 | ~100ms | N/A | N/A | ❌ | ❌ |
| **CAT-ORA Standalone** | 36 | **~105ms** | **35.00m** | **23.13m** | ✅ | ❌ |
| CAT-ORA (ROS2) | 25 | ~150ms | ~15.93m | ~11.43m | ✅ | ✅ |

**결론**:
- **Hungarian**: 가장 빠르지만 충돌 가능성
- **CAT-ORA Standalone** ⭐: Hungarian과 비슷한 속도 + 충돌 회피 + ROS2 불필요 (추천!)
- **CAT-ORA (ROS2)**: 원본 C++ 구현, 느리지만 정확

자세한 비교는 [COMPARISON.md](COMPARISON.md) 참조

## 🔗 관련 파일

- `generate_assignment.py` - 통합 할당 생성기 (Hungarian + CAT-ORA)
- `formations_large.py` - 형상 정의
- `Fair_Hungarian_Allocator.py` - Hungarian 알고리즘 구현
- `CATORA_Allocator.py` - CAT-ORA 인터페이스 (Standalone + ROS2 Service)
- `CATORA_Allocator_Standalone.py` - ⭐ CAT-ORA Standalone 구현 (NEW!)
- `COMPARISON.md` - ⭐ Hungarian vs CAT-ORA 상세 비교 (NEW!)
- `scenario_swarm_*.launch.py` - 시뮬레이션 실행 파일

## 🎓 참고 문헌

### Fair Hungarian Algorithm
- Hungarian Algorithm: O(n³) 복잡도
- Min-Max 최적화: 반복적 개선

### CAT-ORA Algorithm
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

**논문**: https://arxiv.org/pdf/2412.00603
**원본 저장소**: https://github.com/ctu-mrs/catora

## 📅 업데이트 내역

- **2025-11-21**: ⭐ CAT-ORA Standalone 구현 추가 (ROS2 불필요!)
- **2025-11-19**: CAT-ORA 알고리즘 추가, 폴더 구조 개편
- **2025-11-18**: Fair Hungarian 알고리즘 통합

## 🎯 요약

| 특징 | Hungarian | CAT-ORA Standalone ⭐ | CAT-ORA (ROS2) |
|-----|-----------|---------------------|----------------|
| **폴더** | `Fair_Hungarian_{num}/` | `CATORA_{num}/` | `CATORA_{num}/` |
| **명령어** | `--algorithm hungarian` | `--algorithm catora` | (수동 설정 필요) |
| **의존성** | Python, scipy | Python, scipy, numpy | ROS2, catora_planner |
| **속도** | ⚡ 빠름 (~50ms) | ⚡ 빠름 (~54ms) | 🐢 느림 (~150ms) |
| **안전성** | ⚠️ 충돌 가능 | ✅ 충돌 회피 | ✅ 충돌 회피 |
| **사용 시나리오** | 빠른 프로토타이핑 | **✨ 추천: 대부분의 경우** | 원본 C++ 구현 필요 시 |
