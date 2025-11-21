# 25대 드론 궤적 샘플링 빠른 가이드

## 실행 방법

### 터미널 1: 시뮬레이션 실행

```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/setup.bash

# 25대 드론 시나리오 실행
ros2 launch ego_planner scenario_swarm_25.launch.py
```

### 터미널 2: 궤적 샘플링 (30초 후 실행)

시뮬레이션이 시작되고 30초 정도 대기한 후:

```bash
cd /home/pjh/ego_swarm/ego-planner-swarm

# 60초 동안 50Hz로 샘플링
python3 scripts/sample_trajectories.py \
    --num_drones 25 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format all

# 또는 특정 포맷만
python3 scripts/sample_trajectories.py \
    --num_drones 25 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format csv
```

## 출력 결과

```
trajectories_YYYYMMDD_HHMMSS/
├── drone_0_trajectory.csv    # 각 드론별 CSV
├── drone_1_trajectory.csv
├── ...
├── drone_24_trajectory.csv
├── drone_0_trajectory.npy    # NumPy 배열
├── ...
├── all_trajectories.json     # 전체 JSON
├── node_0.txt                # DroneShow 형식
└── ...
```

## 빠른 확인

```bash
# B-spline 토픽 확인
ros2 topic list | grep bspline

# 특정 드론의 궤적 확인
ros2 topic echo /drone_0_planning/bspline

# 노드 실행 확인
ros2 node list | grep ego_planner | wc -l
# 25가 나와야 함
```

## 트러블슈팅

### 1. "No trajectory data" 나오는 경우

**원인:** 드론들이 아직 계획을 시작하지 않음

**해결:**
- 더 오래 기다리기 (60초 정도)
- 로그 확인: `grep "planning complete" ~/.ros/log/latest/*.log`

### 2. 일부 드론만 데이터가 있는 경우

**원인:** 일부 드론의 계획이 실패

**해결:**
- `--duration`을 늘려서 더 오래 녹화
- 또는 성공한 드론만 사용

### 3. ROS2 토픽이 안 보이는 경우

**원인:** 환경 변수 설정 안 됨

**해결:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 예제: 성공적인 샘플링

```bash
# 터미널 1
ros2 launch ego_planner scenario_swarm_25.launch.py

# 60초 대기...

# 터미널 2
python3 scripts/sample_trajectories.py --num_drones 25 --sampling_rate 50.0

# 예상 출력:
# [INFO] [trajectory_sampler]: Drone 0: Sampled trajectory #1 (500 points, duration=10.00s)
# [INFO] [trajectory_sampler]: Drone 1: Sampled trajectory #1 (500 points, duration=10.00s)
# ...
#
# Trajectory Statistics:
# ================================================================================
#   Drone  0:  3000 samples,  60.00s duration,  120.45m length, 1 trajectory segments
#   Drone  1:  3000 samples,  60.00s duration,  118.23m length, 1 trajectory segments
#   ...
# ================================================================================
#   Total: 75000 samples, 60.00s max duration, 3012.34m total length
#
# ✓ Export complete!
```

## Python으로 분석

```python
import pandas as pd
import numpy as np

# CSV 읽기
df = pd.read_csv('trajectories_*/drone_0_trajectory.csv')
print(df.head())

# NumPy 읽기 (더 빠름)
traj = np.load('trajectories_*/drone_0_trajectory.npy')
print(traj.shape)  # (N, 4): timestamp, x, y, z

# JSON 읽기 (전체 정보)
import json
with open('trajectories_*/all_trajectories.json') as f:
    data = json.load(f)
    print(f"Total drones: {data['metadata']['num_drones']}")
```

## 참고

- **샘플링 레이트**: 50Hz (0.02초 간격) 권장
- **녹화 시간**: 60초면 충분 (전체 미션 ~40-50초)
- **출력 크기**: 25대 × 3000 샘플 = ~75,000 points (~2MB)
