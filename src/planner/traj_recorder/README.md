# Multi-Drone Trajectory Recorder

ROS2 노드로 여러 드론의 ego-planner B-spline 궤적을 동시에 TXT 파일로 기록합니다.

## 주요 기능

- **여러 드론 동시 기록**: N대의 드론 궤적을 한 번에 기록
- **자동 폴더 생성**: 날짜+시간으로 폴더 자동 생성 (예: `trajectories_20241031_143025`)
- **개별 파일 저장**: 각 드론마다 `node_1.txt`, `node_2.txt`, ... 형식으로 저장
- **조절 가능한 샘플링**: 원하는 간격으로 궤적 샘플링 (기본 0.05초)

## 출력 구조

```
trajectories_20241031_143025/
├── node_1.txt
├── node_2.txt
├── node_3.txt
└── node_N.txt
```

각 파일 형식:
```
1,1,0.00,move,0.00,0.00,3.0,0.0,255,255,255
2,1,0.05,move,0.10,0.00,3.0,0.0,255,255,255
3,1,0.10,move,0.20,0.00,3.0,0.0,255,255,255
...
```

## 사용 방법

### 1. 단일 드론 기록

```bash
source install/setup.bash
ros2 launch traj_recorder record_trajectory.launch.py
```

출력: `trajectories_YYYYMMDD_HHMMSS/node_1.txt`

### 2. 여러 드론 기록 (10대)

```bash
ros2 launch traj_recorder record_trajectory.launch.py num_drones:=10
```

출력: `trajectories_YYYYMMDD_HHMMSS/node_{1..10}.txt`

### 3. 36대 드론 기록 (FHA 시나리오)

```bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=scenarios_FHA_36
```

출력: `scenarios_FHA_36_20241031_143025/node_{1..36}.txt`

### 4. 샘플링 주기 변경

```bash
# 더 세밀하게 (20ms 간격)
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    sampling_dt:=0.02

# 더 성기게 (100ms 간격)
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    sampling_dt:=0.1
```

### 5. 시뮬레이션과 함께 실행

**터미널 1** - 시뮬레이션 실행:
```bash
source install/setup.bash
ros2 launch plan_manage scenario_swarm.launch.py
```

**터미널 2** - 궤적 기록:
```bash
source install/setup.bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=10 \
    output_folder:=test_scenario
```

**시뮬레이션 실행 후** → Ctrl+C로 종료 → 자동으로 파일 저장됨

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `num_drones` | 1 | 기록할 드론 개수 (1~N) |
| `output_folder` | trajectories | 출력 폴더 base 이름 (날짜시간 자동 추가됨) |
| `sampling_dt` | 0.05 | 샘플링 시간 간격 (초) |
| `default_rgb` | [255,255,255] | 기본 LED 색상 (RGB) |
| `record_height` | 3.0 | 기록 고도 (현재 사용 안 함) |

## 실제 사용 예시

### 시나리오 1: 소규모 테스트 (3대)

```bash
# 터미널 1: 시뮬레이션
ros2 launch plan_manage run_in_sim.launch.py

# 터미널 2: 기록
ros2 launch traj_recorder record_trajectory.launch.py num_drones:=3
```

### 시나리오 2: 대규모 스웜 (36대)

```bash
# 터미널 1: 스웜 시뮬레이션
ros2 launch plan_manage scenario_swarm.launch.py

# 터미널 2: 기록
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=scenarios_FHA_36 \
    sampling_dt:=0.05
```

### 시나리오 3: 레이어 플래너 테스트

```bash
# 터미널 1: 레이어 플래너
ros2 launch layer_planner test_scenario.launch.py

# 터미널 2: 기록
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=25 \
    output_folder:=layer_test
```

## 출력 파일 확인

```bash
# 폴더 확인
ls -la trajectories_*/

# 파일 개수 확인
ls trajectories_20241031_143025/ | wc -l

# 특정 드론 궤적 확인
head -20 trajectories_20241031_143025/node_1.txt

# 모든 파일 라인 수 확인
wc -l trajectories_20241031_143025/*.txt
```

## 초기화 명령 추가하기

생성된 TXT 파일에는 `move` 명령만 있으므로, 초기화 명령을 수동으로 추가해야 합니다.

### 방법 1: 수동 편집

각 `node_N.txt` 파일 맨 앞에 추가:
```
1,N,1.00,cmd,arm
2,N,2.00,offboard
3,N,3.00,takeoff,3.0
4,N,4.00,move,...
```

### 방법 2: Python 스크립트

```python
import os
import glob

def add_header(folder_path, drone_id):
    """각 드론 파일에 초기화 헤더 추가"""
    input_file = f"{folder_path}/node_{drone_id}.txt"
    output_file = f"{folder_path}/node_{drone_id}_with_header.txt"

    with open(output_file, 'w') as out:
        # 헤더 작성
        out.write(f"1,{drone_id},1.00,cmd,arm\n")
        out.write(f"2,{drone_id},2.00,offboard\n")
        out.write(f"3,{drone_id},3.00,takeoff,3.0\n")

        # 기존 내용 복사 (라인 번호 조정)
        line_offset = 3
        with open(input_file, 'r') as inp:
            for line in inp:
                parts = line.strip().split(',', 1)
                if len(parts) == 2:
                    old_num = int(parts[0])
                    new_num = old_num + line_offset
                    out.write(f"{new_num},{parts[1]}\n")

# 사용 예시
folder = "trajectories_20241031_143025"
for drone_id in range(1, 37):  # 1~36번 드론
    add_header(folder, drone_id)

print(f"Headers added! Check {folder}/node_*_with_header.txt")
```

### 방법 3: Bash 스크립트

```bash
#!/bin/bash

FOLDER="trajectories_20241031_143025"
NUM_DRONES=36

for i in $(seq 1 $NUM_DRONES); do
    INPUT="$FOLDER/node_$i.txt"
    OUTPUT="$FOLDER/node_${i}_final.txt"

    # 헤더 작성
    echo "1,$i,1.00,cmd,arm" > $OUTPUT
    echo "2,$i,2.00,offboard" >> $OUTPUT
    echo "3,$i,3.00,takeoff,3.0" >> $OUTPUT

    # 기존 내용 추가 (라인 번호 +3)
    awk -F',' -v offset=3 '{print $1+offset","$2","$3","$4","$5","$6","$7","$8","$9","$10","$11}' \
        $INPUT >> $OUTPUT
done

echo "Done! Check $FOLDER/node_*_final.txt"
```

## LED 색상 변경

기본적으로 모든 LED는 흰색(255,255,255)입니다. 색상을 변경하려면:

### Python으로 구간별 색상 변경

```python
import pandas as pd

# 파일 읽기
df = pd.read_csv('trajectories_20241031_143025/node_1.txt',
                 header=None,
                 names=['line', 'drone_id', 'time', 'cmd',
                        'x', 'y', 'z', 'yaw', 'r', 'g', 'b'])

# 시간대별 색상 변경
# 0-5초: 흰색 (그대로)
# 5-10초: 초록색
mask1 = (df['time'] >= 5.0) & (df['time'] < 10.0)
df.loc[mask1, ['r', 'g', 'b']] = [0, 255, 0]

# 10-15초: 빨강색 (회피 중)
mask2 = (df['time'] >= 10.0) & (df['time'] < 15.0)
df.loc[mask2, ['r', 'g', 'b']] = [255, 0, 0]

# 15초 이후: 파랑색
mask3 = (df['time'] >= 15.0)
df.loc[mask3, ['r', 'g', 'b']] = [0, 0, 255]

# 저장
df.to_csv('trajectories_20241031_143025/node_1_colored.txt',
          header=False, index=False)
```

## 문제 해결

### 토픽이 없다고 나올 때
```bash
# 사용 가능한 토픽 확인
ros2 topic list | grep bspline

# 특정 드론 토픽 확인
ros2 topic info /drone_1_planning/bspline
ros2 topic echo /drone_1_planning/bspline --once
```

### 일부 드론만 파일이 생성될 때
- 해당 드론이 실제로 실행 중인지 확인
- 해당 드론이 궤적을 생성했는지 확인 (목표점 설정됨?)
- 노드 출력에서 "Drone N: trajectory" 메시지 확인

### 파일이 비어있을 때
- 시뮬레이션이 실행 중이었는지 확인
- 드론에게 목표점이 설정되었는지 확인
- 노드 출력에서 "Received trajectory" 메시지가 있는지 확인

### 폴더를 만들 수 없을 때
- 쓰기 권한이 있는지 확인
- 디스크 공간이 충분한지 확인

## 추가 팁

### 1. 백그라운드 실행
```bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=bg_test &
```

### 2. 로그 저장
```bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=test 2>&1 | tee recorder.log
```

### 3. 특정 시간 후 자동 종료
```bash
timeout 60 ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36
# 60초 후 자동 종료
```

## 전체 워크플로우 예시

```bash
# 1. 시뮬레이션 시작
cd /home/pjh/ego_swarm/ego-planner-swarm
source install/setup.bash
ros2 launch plan_manage scenario_swarm.launch.py

# (별도 터미널)
# 2. 궤적 기록 시작
cd /home/pjh/ego_swarm/ego-planner-swarm
source install/setup.bash
ros2 launch traj_recorder record_trajectory.launch.py \
    num_drones:=36 \
    output_folder:=scenarios_FHA_36 \
    sampling_dt:=0.05

# 3. 시뮬레이션 실행 및 관찰

# 4. 종료 (Ctrl+C)
# 자동으로 scenarios_FHA_36_YYYYMMDD_HHMMSS/ 폴더에 저장됨

# 5. 결과 확인
ls -lh scenarios_FHA_36_*/
head -20 scenarios_FHA_36_*/node_1.txt

# 6. 초기화 헤더 추가 (Python 스크립트 사용)
python3 add_headers.py scenarios_FHA_36_20241031_143025

# 7. 최종 파일 확인
cat scenarios_FHA_36_20241031_143025/node_1_final.txt | head -30
```

## 요약

- **단일 명령으로 N대 드론 동시 기록** ✅
- **자동 폴더 생성 (날짜_시간)** ✅
- **개별 파일 (node_1.txt ~ node_N.txt)** ✅
- **초기화 명령은 나중에 추가** ✅
- **LED 색상은 나중에 편집** ✅
