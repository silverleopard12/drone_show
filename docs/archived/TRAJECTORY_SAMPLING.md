# 궤적 샘플링 가이드

완성된 드론 궤적을 각 드론마다 샘플링해서 추출하는 방법입니다.

## 빠른 시작

### 방법 1: 간단한 실행 (bash 스크립트)

```bash
# 36대 드론, 50Hz 샘플링, 60초 녹화
./scripts/sample_trajectories_launcher.sh

# 커스텀 설정
./scripts/sample_trajectories_launcher.sh 25 100.0 120.0 csv
#                                         ^^  ^^^^  ^^^^  ^^^
#                                         |   |     |     출력 포맷
#                                         |   |     녹화 시간(초)
#                                         |   샘플링 레이트(Hz)
#                                         드론 개수
```

### 방법 2: Python 직접 실행

```bash
# 기본 설정 (36대, 50Hz, 60초)
python3 scripts/sample_trajectories.py

# 모든 옵션 지정
python3 scripts/sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format all \
    --output_dir my_trajectories
```

## 사용 방법

### 1단계: 시뮬레이션 실행

먼저 ego-swarm 시뮬레이션을 실행하세요:

```bash
# 터미널 1: 시뮬레이션 실행
ros2 launch ego_planner scenario_swarm_36.launch.py
```

### 2단계: 궤적 샘플러 시작

시뮬레이션이 실행되는 동안 새 터미널에서:

```bash
# 터미널 2: 궤적 샘플링 시작
./scripts/sample_trajectories_launcher.sh
```

또는:

```bash
python3 scripts/sample_trajectories.py --num_drones 36 --sampling_rate 50.0
```

### 3단계: 결과 확인

샘플링이 완료되면 `trajectories_YYYYMMDD_HHMMSS/` 디렉토리에 결과가 저장됩니다.

## 옵션 설명

### 필수 파라미터

- `--num_drones`: 드론 개수 (기본값: 36)
  - 25대 스웜: `--num_drones 25`
  - 10대 스웜: `--num_drones 10`

- `--sampling_rate`: 샘플링 주파수 (Hz) (기본값: 50.0)
  - **기본 50Hz (0.02초)**: 대부분의 경우 충분, 권장
  - 더 세밀한 샘플링: `--sampling_rate 100.0` (0.01초마다)
  - 드론 제어기 레벨: `--sampling_rate 200.0` (0.005초, 매우 정밀)
  - 빠른 시각화용: `--sampling_rate 20.0` (0.05초마다)

  **참고:** B-spline은 연속 곡선이므로 50Hz면 충분히 정확합니다.
  제어점 간격(0.4m)과 속도(~3m/s)를 고려하면 0.02초 간격이 적절합니다.

- `--duration`: 녹화 시간 (초) (기본값: 60.0)
  - 전체 미션 녹화: `--duration 120.0`
  - 빠른 테스트: `--duration 10.0`

### 출력 포맷

- `--format`: 출력 파일 형식 (기본값: all)
  - `csv`: 각 드론마다 별도 CSV 파일
  - `json`: 모든 드론을 하나의 JSON 파일에
  - `numpy`: NumPy 배열로 저장 (.npy)
  - `droneshow`: DroneShow 형식 (node_X.txt)
  - `all`: 모든 형식으로 저장

### 출력 디렉토리

- `--output_dir`: 출력 폴더 이름 (기본값: 자동 생성)
  - 기본: `trajectories_20251118_143022/`
  - 커스텀: `--output_dir my_experiment_1`

## 출력 형식 상세

### 1. CSV 형식 (`drone_X_trajectory.csv`)

각 드론마다 별도의 CSV 파일:

```csv
timestamp,x,y,z
0.00,0.00,0.00,1.00
0.02,0.15,0.02,1.01
0.04,0.31,0.05,1.02
...
```

**사용 예:**
```python
import pandas as pd

# 드론 0의 궤적 읽기
df = pd.read_csv('trajectories_*/drone_0_trajectory.csv')
print(df.head())

# 3D 플롯
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['x'], df['y'], df['z'])
plt.show()
```

### 2. JSON 형식 (`all_trajectories.json`)

모든 드론의 궤적을 하나의 파일에:

```json
{
  "metadata": {
    "num_drones": 36,
    "timestamp": "2025-11-18T14:30:22"
  },
  "drones": {
    "0": {
      "drone_id": 0,
      "trajectory_count": 1,
      "num_samples": 3000,
      "duration": 60.0,
      "length": 141.23,
      "samples": [
        {"t": 0.0, "x": 0.0, "y": 0.0, "z": 1.0},
        {"t": 0.02, "x": 0.15, "y": 0.02, "z": 1.01},
        ...
      ]
    },
    ...
  }
}
```

**사용 예:**
```python
import json

# JSON 읽기
with open('trajectories_*/all_trajectories.json') as f:
    data = json.load(f)

# 드론 0의 궤적 추출
drone_0 = data['drones']['0']
print(f"드론 0: {drone_0['num_samples']} 샘플, {drone_0['duration']:.2f}초")
print(f"경로 길이: {drone_0['length']:.2f}m")

# 전체 통계
total_length = sum(d['length'] for d in data['drones'].values())
print(f"전체 경로 길이: {total_length:.2f}m")
```

### 3. NumPy 형식 (`drone_X_trajectory.npy`)

각 드론마다 NumPy 배열 (N × 4: timestamp, x, y, z):

**사용 예:**
```python
import numpy as np

# 드론 0의 궤적 읽기
traj = np.load('trajectories_*/drone_0_trajectory.npy')

# 배열 구조: [timestamp, x, y, z]
timestamps = traj[:, 0]
positions = traj[:, 1:4]  # x, y, z

# 속도 계산
dt = np.diff(timestamps)
velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]

# 평균 속도
avg_vel = np.mean(np.linalg.norm(velocities, axis=1))
print(f"평균 속도: {avg_vel:.2f} m/s")

# 최대 가속도 계산
accelerations = np.diff(velocities, axis=0) / dt[1:, np.newaxis]
max_acc = np.max(np.linalg.norm(accelerations, axis=1))
print(f"최대 가속도: {max_acc:.2f} m/s²")
```

### 4. DroneShow 형식 (`node_X.txt`)

DroneShow 시뮬레이터 호환 형식:

```
1,0,0.00,move,0.00,0.00,1.0,0.0,255,0,0
2,0,0.02,move,0.15,0.02,1.0,0.0,255,0,0
3,0,0.04,move,0.31,0.05,1.0,0.0,255,0,0
```

형식: `line_number,drone_id,timestamp,move,x,y,z,yaw,r,g,b`

## 실전 예제

### 예제 1: 고정밀 샘플링 (논문용)

```bash
python3 scripts/sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 200.0 \
    --duration 120.0 \
    --format numpy \
    --output_dir paper_experiment_1
```

### 예제 2: 빠른 테스트

```bash
python3 scripts/sample_trajectories.py \
    --num_drones 5 \
    --sampling_rate 20.0 \
    --duration 10.0 \
    --format csv
```

### 예제 3: DroneShow 시각화용

```bash
python3 scripts/sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format droneshow \
    --output_dir droneshow_formation
```

### 예제 4: 분석용 (CSV + JSON)

```bash
python3 scripts/sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 100.0 \
    --format csv,json \
    --output_dir analysis_data
```

## 출력 통계

샘플링 완료 시 다음 통계가 출력됩니다:

```
Trajectory Statistics:
================================================================================
  Drone  0:  3000 samples,  60.00s duration,  141.23m length, 1 trajectory segments
  Drone  1:  3000 samples,  60.00s duration,  138.45m length, 1 trajectory segments
  ...
  Drone 35:  3000 samples,  60.00s duration,  142.67m length, 1 trajectory segments
================================================================================
  Total: 108000 samples, 60.00s max duration, 5034.56m total length
```

## 데이터 분석 예제

### Python으로 전체 스웜 시각화

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 모든 드론의 궤적 로드
num_drones = 36
trajectories = []
for i in range(num_drones):
    traj = np.load(f'trajectories_*/drone_{i}_trajectory.npy')
    trajectories.append(traj)

# 3D 플롯
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

for i, traj in enumerate(trajectories):
    x, y, z = traj[:, 1], traj[:, 2], traj[:, 3]
    ax.plot(x, y, z, alpha=0.6, linewidth=0.5, label=f'Drone {i}' if i < 5 else '')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('36-Drone Swarm Trajectories')
ax.legend()
plt.tight_layout()
plt.savefig('swarm_trajectories_3d.png', dpi=300)
plt.show()
```

### 충돌 거리 분석

```python
import numpy as np

# 특정 시간에서 모든 드론 위치 확인
def get_positions_at_time(trajectories, t):
    positions = []
    for traj in trajectories:
        # 시간 t에 가장 가까운 샘플 찾기
        idx = np.argmin(np.abs(traj[:, 0] - t))
        positions.append(traj[idx, 1:4])
    return np.array(positions)

# 최소 드론 간 거리 계산
def min_inter_drone_distance(positions):
    n = len(positions)
    min_dist = float('inf')

    for i in range(n):
        for j in range(i+1, n):
            dist = np.linalg.norm(positions[i] - positions[j])
            min_dist = min(min_dist, dist)

    return min_dist

# 전체 시간에 걸쳐 최소 거리 추적
times = np.arange(0, 60, 0.1)
min_distances = []

for t in times:
    positions = get_positions_at_time(trajectories, t)
    min_dist = min_inter_drone_distance(positions)
    min_distances.append(min_dist)

# 플롯
plt.figure(figsize=(10, 6))
plt.plot(times, min_distances)
plt.axhline(y=1.0, color='r', linestyle='--', label='Safety clearance (1.0m)')
plt.xlabel('Time (s)')
plt.ylabel('Minimum inter-drone distance (m)')
plt.title('Collision Safety Analysis')
plt.legend()
plt.grid(True)
plt.savefig('collision_safety.png', dpi=300)
plt.show()

print(f"절대 최소 거리: {np.min(min_distances):.2f}m")
print(f"평균 최소 거리: {np.mean(min_distances):.2f}m")
```

### 궤적 품질 메트릭

```python
import numpy as np

def analyze_trajectory_quality(traj):
    """궤적 품질 분석"""

    timestamps = traj[:, 0]
    positions = traj[:, 1:4]

    # 속도 계산
    dt = np.diff(timestamps)
    velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]
    vel_magnitudes = np.linalg.norm(velocities, axis=1)

    # 가속도 계산
    accelerations = np.diff(velocities, axis=0) / dt[1:, np.newaxis]
    acc_magnitudes = np.linalg.norm(accelerations, axis=1)

    # Jerk 계산 (가속도 변화율)
    jerks = np.diff(accelerations, axis=0) / dt[2:, np.newaxis]
    jerk_magnitudes = np.linalg.norm(jerks, axis=1)

    # 경로 길이
    path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))

    # 통계
    metrics = {
        'path_length': path_length,
        'duration': timestamps[-1] - timestamps[0],
        'avg_velocity': np.mean(vel_magnitudes),
        'max_velocity': np.max(vel_magnitudes),
        'avg_acceleration': np.mean(acc_magnitudes),
        'max_acceleration': np.max(acc_magnitudes),
        'avg_jerk': np.mean(jerk_magnitudes),
        'max_jerk': np.max(jerk_magnitudes),
    }

    return metrics

# 모든 드론 분석
for i in range(num_drones):
    traj = np.load(f'trajectories_*/drone_{i}_trajectory.npy')
    metrics = analyze_trajectory_quality(traj)

    print(f"드론 {i}:")
    print(f"  경로 길이: {metrics['path_length']:.2f}m")
    print(f"  평균/최대 속도: {metrics['avg_velocity']:.2f}/{metrics['max_velocity']:.2f} m/s")
    print(f"  평균/최대 가속도: {metrics['avg_acceleration']:.2f}/{metrics['max_acceleration']:.2f} m/s²")
    print(f"  평균/최대 jerk: {metrics['avg_jerk']:.2f}/{metrics['max_jerk']:.2f} m/s³")
    print()
```

## 트러블슈팅

### 문제 1: "No trajectory data" 에러

**원인:** 드론이 궤적을 publish하지 않음

**해결:**
```bash
# B-spline 토픽 확인
ros2 topic list | grep bspline

# 특정 드론의 토픽 모니터링
ros2 topic echo /drone_0_planning/bspline
```

### 문제 2: 샘플이 너무 적음

**원인:** 샘플링 레이트가 너무 낮거나 녹화 시간이 짧음

**해결:**
```bash
# 샘플링 레이트 증가 + 녹화 시간 증가
python3 scripts/sample_trajectories.py \
    --sampling_rate 100.0 \
    --duration 120.0
```

### 문제 3: 메모리 부족

**원인:** 너무 많은 드론 + 너무 높은 샘플링 레이트

**해결:**
```bash
# 샘플링 레이트 감소
python3 scripts/sample_trajectories.py \
    --sampling_rate 20.0 \
    --format csv  # NumPy 대신 CSV 사용
```

### 문제 4: 타이밍이 맞지 않음

**원인:** 시뮬레이션 시작 전에 샘플러를 실행함

**해결:**
1. 먼저 시뮬레이션 실행
2. 드론이 계획을 완료할 때까지 대기
3. 그 다음 샘플러 실행

## 성능 가이드

### 메모리 사용량

| 드론 수 | 샘플링 레이트 | 녹화 시간 | 예상 메모리 |
|---------|---------------|-----------|-------------|
| 10      | 50 Hz         | 60s       | ~20 MB      |
| 25      | 50 Hz         | 60s       | ~50 MB      |
| 36      | 50 Hz         | 60s       | ~70 MB      |
| 36      | 200 Hz        | 120s      | ~280 MB     |

### CPU 사용량

- 샘플러는 매우 가볍습니다 (~1-2% CPU)
- B-spline 평가가 주요 연산
- 실시간 처리 가능

## 기존 traj_recorder와의 차이

| 기능 | traj_recorder | sample_trajectories.py |
|------|---------------|------------------------|
| 샘플링 방식 | B-spline 직접 평가 | B-spline 직접 평가 |
| 출력 형식 | DroneShow만 | CSV, JSON, NumPy, DroneShow |
| Python 분석 | 어려움 | 쉬움 (pandas, numpy) |
| 메타데이터 | 없음 | 통계, 길이, 시간 포함 |
| 실시간 통계 | 없음 | 있음 |

## 다음 단계

샘플링된 궤적으로 할 수 있는 작업:

1. **시각화**: Matplotlib, Plotly로 3D 애니메이션
2. **분석**: 충돌 안전성, 궤적 품질, 에너지 효율
3. **검증**: 실제 드론 궤적과 비교
4. **최적화**: 궤적 평활화, 재계획
5. **시뮬레이션**: DroneShow로 재생

## 관련 파일

- `scripts/sample_trajectories.py` - 메인 샘플링 스크립트
- `scripts/sample_trajectories_launcher.sh` - 실행 래퍼
- `src/planner/traj_recorder/` - 기존 레코더 (C++)
- `OFFLINE_TRAJECTORY_PLANNING.md` - 오프라인 계획 가이드

## 작성일

2025-11-18

## 문의

궤적 샘플링 관련 문제가 있으면:
1. 로그 확인: `ros2 topic echo /drone_0_planning/bspline`
2. 통계 확인: 샘플러가 출력하는 통계 메시지
3. 파일 확인: 출력 디렉토리의 파일들
