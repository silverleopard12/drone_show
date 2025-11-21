# Trajectory Sampling Guide

## 개요
`traj_recorder_node`는 ego-planner의 B-spline 궤적을 실시간으로 샘플링하여 DroneShow 포맷으로 저장하는 C++ ROS2 노드입니다.

---

## 1. 전체 흐름

```
ego_planner → B-spline 발행 → traj_recorder 구독 → 샘플링 → 파일 저장 → 종료 시 패딩
```

### 단계별 설명
1. **초기화**: 각 드론마다 `node_N.txt` 파일 생성 및 토픽 구독
2. **실시간 기록**: B-spline 메시지 수신 시마다 샘플링하여 파일에 기록
3. **종료 시 후처리**: 모든 파일을 가장 긴 파일 길이로 패딩 (호버링)

---

## 2. B-spline 샘플링 과정

### 2.1 B-spline 메시지 수신
```cpp
void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg, int drone_id)
```

**입력 데이터:**
- `msg->pos_pts`: Control points (제어점) 배열 - 3D 위치 좌표들
- `msg->knots`: Knot vector (매듭 벡터) - B-spline 파라미터 시퀀스
- `msg->order`: B-spline 차수 (일반적으로 3 = cubic B-spline)
- `msg->start_time`: 이 궤적의 시작 시간 (ROS 타임스탬프)
- `msg->traj_id`: 궤적 ID (재계획 시마다 증가)

### 2.2 B-spline 평가기 생성
```cpp
// Control points를 Eigen 형식으로 변환
std::vector<Eigen::Vector3d> ctrl_pts;
for (const auto& pt : msg->pos_pts) {
    ctrl_pts.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
}

// B-spline evaluator 생성
SimpleBsplineEvaluator bspline(ctrl_pts, msg->knots, msg->order);
double duration = bspline.getDuration();
```

**B-spline Duration 계산:**
```cpp
duration = knots[m - p] - knots[p]
```
- `m`: knot 개수 - 1
- `p`: B-spline 차수 (order)
- Duration은 유효한 파라미터 범위의 길이

### 2.3 시간 오프셋 계산
```cpp
rclcpp::Time msg_time(msg->start_time);
double msg_offset = (msg_time - global_start_time_).seconds();
```

**목적:** 모든 드론의 궤적을 전역 시작 시간 기준으로 동기화

예시:
- `global_start_time_`: 0.0초 (첫 메시지 수신 시각)
- `msg->start_time`: 5.2초 (이 궤적의 시작)
- `msg_offset`: 5.2초
- 샘플 시간: `msg_offset + t` = 5.2 + 0.0, 5.2 + 0.02, 5.2 + 0.04, ...

### 2.4 궤적 샘플링 루프
```cpp
for (double t = 0.0; t <= duration; t += sampling_dt_) {
    Eigen::Vector3d pos = bspline.evaluate(t);
    double timestamp = msg_offset + t;
    writeTrajectoryPoint(handler, drone_id, timestamp, pos.x(), pos.y(), pos.z(), 0.0);
    handler.last_timestamp = timestamp;
    handler.last_position = pos;
}
```

**파라미터:**
- `t`: B-spline 내부 시간 (0.0 ~ duration)
- `sampling_dt_`: 샘플링 주기 (기본 0.02초 = 50Hz)
- `timestamp`: 전역 시간 좌표계에서의 절대 시간

**샘플링 예시:**
```
Duration = 2.0초, sampling_dt = 0.02초, msg_offset = 5.2초

t=0.00 → timestamp=5.20 → pos=(3.0, 8.0, 10.0)
t=0.02 → timestamp=5.22 → pos=(3.0, 8.1, 10.1)
t=0.04 → timestamp=5.24 → pos=(3.0, 8.2, 10.2)
...
t=2.00 → timestamp=7.20 → pos=(3.0, 20.7, 16.5)
```

### 2.5 마지막 점 추가
```cpp
// Always write the final point
Eigen::Vector3d final_pos = bspline.evaluate(duration);
double final_timestamp = msg_offset + duration;
writeTrajectoryPoint(handler, drone_id, final_timestamp,
                   final_pos.x(), final_pos.y(), final_pos.z(), 0.0);
```

**이유:** 루프가 정확히 `duration`에서 끝나지 않을 수 있으므로, 마지막 점을 명시적으로 추가

---

## 3. de Boor's Algorithm (B-spline 평가)

### 3.1 시간 → 파라미터 변환
```cpp
Eigen::Vector3d evaluate(double t) {
    double u = t + knots_[p_];
    return evaluateDeBoor(u);
}
```

**변환 이유:**
- `t`: 실제 경과 시간 (0 ~ duration)
- `u`: B-spline 파라미터 공간 (knots[p] ~ knots[m-p])
- 변환 공식: `u = t + knots[p]`

### 3.2 de Boor 알고리즘 핵심
```cpp
Eigen::Vector3d evaluateDeBoor(double u) {
    // 1. u를 유효 범위로 클램핑
    double u_min = knots_[p_];
    double u_max = knots_[m_ - p_];
    double ub = std::min(std::max(u_min, u), u_max);

    // 2. Knot span 찾기: u가 [knots[k], knots[k+1]) 구간에 속하는 k 찾기
    int k = p_;
    while (k < m_ - p_) {
        if (knots_[k + 1] >= ub)
            break;
        ++k;
    }

    // 3. 초기 제어점 설정 (p+1개)
    std::vector<Eigen::Vector3d> d;
    for (int i = 0; i <= p_; ++i) {
        int idx = k - p_ + i;
        if (idx >= 0 && idx < control_points_.size()) {
            d.push_back(control_points_[idx]);
        } else {
            d.push_back(Eigen::Vector3d::Zero());
        }
    }

    // 4. 재귀적 보간 (Recursive blending)
    for (int r = 1; r <= p_; ++r) {
        for (int i = p_; i >= r; --i) {
            int idx_alpha = i + k - p_;

            // Alpha 계산 (블렌딩 가중치)
            double denominator = knots_[idx_alpha + 1 + p_ - r] - knots_[idx_alpha];
            double alpha = 0.0;
            if (std::abs(denominator) > 1e-10) {
                alpha = (ub - knots_[idx_alpha]) / denominator;
            }

            // 선형 보간
            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
        }
    }

    return d[p_];  // 최종 평가 결과
}
```

**알고리즘 단계:**
1. **Clamping**: 파라미터 u를 유효 범위로 제한
2. **Knot Span Search**: u가 속한 구간 찾기
3. **Initial Setup**: 관련된 p+1개의 제어점 선택
4. **Recursive Blending**: p번의 선형 보간을 통해 최종 점 계산

**예시 (Cubic B-spline, p=3):**
```
Step 0: d[0], d[1], d[2], d[3] (4개 제어점)
Step 1: d[1] = blend(d[0], d[1])
        d[2] = blend(d[1], d[2])
        d[3] = blend(d[2], d[3])
Step 2: d[2] = blend(d[1], d[2])
        d[3] = blend(d[2], d[3])
Step 3: d[3] = blend(d[2], d[3])
Result: d[3] = 최종 평가된 3D 좌표
```

---

## 4. 파일 출력 형식

### 4.1 DroneShow 포맷
```cpp
void writeTrajectoryPoint(DroneFileHandler& handler, int drone_id,
                         double timestamp, double x, double y, double z, double yaw)
```

**출력 형식:**
```
line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b
```

**예시:**
```
1,0,7.65,move,3.00,8.00,13.0,0.0,255,255,255
2,0,7.67,move,3.00,8.01,13.0,0.0,255,255,255
3,0,7.69,move,3.00,8.03,13.0,0.0,255,255,255
```

**필드 설명:**
- `line_number`: 줄 번호 (1부터 시작)
- `drone_id`: 드론 ID (0부터 시작, 파일명은 drone_id+1)
- `timestamp`: 전역 타임스탬프 (초 단위, 소수점 2자리)
- `move`: 명령 타입 (항상 "move")
- `x, y, z`: 3D 위치 (소수점 2자리, z만 1자리)
- `yaw`: 요 각도 (현재 0.0)
- `r, g, b`: RGB 색상 (0~255)

---

## 5. 종료 시 Padding (호버링)

### 5.1 최대 길이 찾기
```cpp
~MultiDroneTrajectoryRecorder() {
    // 모든 파일 닫고 최대 line 수 찾기
    int max_lines = 0;
    for (auto& [drone_id, handler] : drone_files_) {
        if (handler.file_stream.is_open()) {
            handler.file_stream.close();
            max_lines = std::max(max_lines, handler.line_number - 1);
        }
    }

    // 모든 파일을 max_lines로 패딩
    if (max_lines > 0) {
        padAllTrajectories(max_lines);
    }
}
```

### 5.2 Padding 과정
```cpp
void padAllTrajectories(int max_lines) {
    for (auto& [drone_id, handler] : drone_files_) {
        int current_lines = handler.line_number - 1;

        if (current_lines >= max_lines) continue;

        int lines_to_add = max_lines - current_lines;

        // 파일을 append 모드로 재오픈
        std::ofstream file(handler.filename, std::ios::app);

        double current_timestamp = handler.last_timestamp;
        int current_line_num = current_lines;

        // 마지막 위치에서 호버링 데이터 추가
        for (int i = 0; i < lines_to_add; ++i) {
            current_line_num++;
            current_timestamp += sampling_dt_;

            // 마지막 위치 (last_position)를 계속 반복
            file << current_line_num << ","
                 << drone_id << ","
                 << std::fixed << std::setprecision(2) << current_timestamp << ","
                 << "move,"
                 << handler.last_position.x() << ","
                 << handler.last_position.y() << ","
                 << handler.last_position.z() << ","
                 << "0.0," << r << "," << g << "," << b << "\n";
        }
    }
}
```

**효과:**
- 모든 파일이 동일한 줄 수를 가짐
- 먼저 도착한 드론은 자신의 마지막 위치에서 정지
- 타임스탬프는 계속 증가 (시간 동기화 유지)

**예시:**
```
Drone 0: 1140 lines → 5146 lines (4006 lines 추가, hover at (3.00, 20.73, 16.5))
Drone 1: 408 lines  → 5146 lines (4738 lines 추가, hover at (3.00, 26.80, 13.2))
Drone 7: 5146 lines → 5146 lines (이미 최대 길이, 패딩 불필요)
```

---

## 6. 주요 파라미터

### Launch 파일 설정 (scenario_swarm_25.launch.py)
```python
trajectory_recorder_node = Node(
    package='traj_recorder',
    executable='traj_recorder_node',
    name='trajectory_recorder',
    output='screen',
    parameters=[{
        'num_drones': 25,                          # 드론 수
        'output_folder': 'trajectories_25_drones', # 출력 폴더 이름
        'sampling_dt': 0.02,                       # 샘플링 주기 (50Hz)
        'default_rgb': [255, 255, 255],            # 기본 RGB 색상
    }]
)
```

### 파라미터 설명
- **num_drones**:
  - 녹화할 드론 수
  - `/drone_0_planning/bspline` ~ `/drone_{N-1}_planning/bspline` 토픽 구독

- **output_folder**:
  - 기본 폴더 이름
  - 실제 폴더: `{output_folder}_YYYYMMDD_HHMMSS`

- **sampling_dt**:
  - B-spline 샘플링 주기 (초)
  - 0.02초 = 50Hz (권장)
  - 0.01초 = 100Hz (더 부드럽지만 파일 크기 증가)

- **default_rgb**:
  - 궤적 포인트의 RGB 색상
  - [255, 255, 255] = 흰색

---

## 7. 실행 예시

### 7.1 Launch 파일 실행
```bash
source install/setup.bash
ros2 launch ego_planner scenario_swarm_25.launch.py
```

### 7.2 출력 예시
```
[trajectory_recorder]: Created output folder: trajectories_25_drones_20251120_215805
[trajectory_recorder]:   Drone 0: subscribed to /drone_0_planning/bspline -> .../node_1.txt
[trajectory_recorder]:   Drone 1: subscribed to /drone_1_planning/bspline -> .../node_2.txt
...
[trajectory_recorder]: Drone 0: trajectory (traj_id: 1, 64 points)
[trajectory_recorder]: Drone 1: trajectory (traj_id: 1, 70 points)
...
[trajectory_recorder]: Saving trajectories...
[trajectory_recorder]:   Drone 0: 1140 lines
[trajectory_recorder]:   Drone 1: 408 lines
...
[trajectory_recorder]: Max lines: 5146
[trajectory_recorder]: Padding all trajectories to 5146 lines...
[trajectory_recorder]:   Drone 0: padded 4006 lines (hover at 3.00, 20.73, 16.5)
[trajectory_recorder]:   Drone 1: padded 4738 lines (hover at 3.00, 26.80, 13.2)
...
[trajectory_recorder]: All trajectories saved to: trajectories_25_drones_20251120_215805
```

### 7.3 결과 파일
```
trajectories_25_drones_20251120_215805/
├── node_1.txt   (5146 lines)
├── node_2.txt   (5146 lines)
├── ...
└── node_25.txt  (5146 lines)
```

---

## 8. 재계획 (Replan) 처리

### 8.1 재계획 발생 시
- Ego-planner는 장애물 회피나 동적 환경에서 궤적을 재계획
- 재계획 시마다 새로운 B-spline 메시지 발행 (traj_id 증가)
- `traj_recorder`는 각 메시지를 순차적으로 샘플링하여 이어붙임

### 8.2 파일 길이 차이 발생 원인
1. **재계획 횟수 차이**: 어떤 드론은 재계획을 많이, 어떤 드론은 적게
2. **도착 시간 차이**: 먼저 도착한 드론은 더 이상 궤적이 발행되지 않음
3. **궤적 duration 차이**: 경로 길이에 따라 각 궤적의 길이가 다름

**→ 이것이 padding이 필요한 이유!**

---

## 9. 디버깅 팁

### 9.1 샘플링이 안 되는 경우
```bash
# B-spline 토픽이 발행되는지 확인
ros2 topic list | grep bspline

# 특정 드론의 B-spline 메시지 확인
ros2 topic echo /drone_0_planning/bspline
```

### 9.2 파일이 비어있는 경우
- 드론이 목표를 받지 못함 → `swarm_synchronizer` 확인
- FSM 상태 확인 → `WAIT_TARGET` 상태에 머물러 있는지 확인

### 9.3 파일 길이가 여전히 다른 경우
- Padding 로그 확인: `"Padding all trajectories to X lines..."`
- 노드가 정상 종료되었는지 확인 (Ctrl+C 또는 자연 종료)

---

## 10. 요약

**샘플링 흐름:**
```
B-spline 수신 → Control points 추출 → de Boor 알고리즘 →
시간 동기화 → 주기적 샘플링 → 파일 기록 → 종료 시 패딩du
```

**핵심 개념:**
1. **B-spline**: 제어점과 knot vector로 정의된 부드러운 곡선
2. **de Boor**: B-spline 위의 임의 점을 계산하는 효율적 알고리즘
3. **Sampling**: 연속 궤적을 이산 포인트로 변환 (50Hz)
4. **Padding**: 모든 파일을 같은 길이로 맞춤 (시각화 목적)
5. **Hovering**: 도착한 드론은 마지막 위치에서 정지

**파일 구조:**
- `node_1.txt` ~ `node_N.txt`: Drone 0 ~ Drone N-1의 궤적
- 모든 파일이 동일한 줄 수 (가장 긴 궤적 기준)
- DroneShow 포맷: line_num, drone_id, timestamp, move, x, y, z, yaw, r, g, b
