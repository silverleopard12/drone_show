# Allocation Module

드론 형상 할당(assignment) 알고리즘과 결과를 관리하는 모듈입니다.

## 📁 Directory Structure

```
Allocation/
├── algorithms/              # 할당 알고리즘 구현
│   ├── Fair_Hungarian_Allocator.py         # Hungarian 알고리즘
│   ├── CATORA_Allocator.py                 # CAT-ORA 인터페이스
│   ├── CATORA_Allocator_Standalone.py      # CAT-ORA Standalone 구현
│   └── __init__.py
├── results/                 # 할당 결과 저장
│   ├── Fair_Hungarian_25/
│   │   └── assignment_25_drones.txt
│   ├── Fair_Hungarian_36/
│   │   └── assignment_36_drones.txt
│   ├── CATORA_25/
│   │   └── assignment_25_drones.txt
│   └── CATORA_36/
│       └── assignment_36_drones.txt
├── docs/                    # 문서
│   ├── README.md           # 상세 사용법 및 비교
│   └── COMPARISON.md       # Hungarian vs CAT-ORA 비교
└── __init__.py             # Python 패키지 초기화
```

## 🚀 Quick Start

### 1. Hungarian 할당 생성

```bash
python3 scripts/generate_assignment.py --algorithm hungarian --num_drones 25
python3 scripts/generate_assignment.py --algorithm hungarian --num_drones 36
```

### 2. CAT-ORA 할당 생성 (Standalone)

```bash
# ROS2 서비스 불필요!
python3 scripts/generate_assignment.py --algorithm catora --num_drones 25
python3 scripts/generate_assignment.py --algorithm catora --num_drones 36
```

### 3. 기존 할당 확인

```bash
python3 scripts/generate_assignment.py --list
```

## 📊 알고리즘 비교

| 특징 | Hungarian | CAT-ORA Standalone |
|-----|-----------|-------------------|
| **속도** | ⚡ ~50ms | ⚡ ~54ms |
| **충돌 검사** | ❌ | ✅ |
| **ROS2 필요** | ❌ | ❌ |
| **Bottleneck 최적화** | ✅ | ✅ |

자세한 비교는 [docs/COMPARISON.md](docs/COMPARISON.md) 참조

## 📖 Documentation

- **[docs/README.md](docs/README.md)** - 전체 사용법, 알고리즘 설명, 예제
- **[docs/COMPARISON.md](docs/COMPARISON.md)** - Hungarian vs CAT-ORA 상세 비교

## 🔧 Files

### Algorithms (`algorithms/`)

- **Fair_Hungarian_Allocator.py** - Min-max Hungarian 알고리즘
- **CATORA_Allocator.py** - CAT-ORA 인터페이스 (Standalone + ROS2 Service)
- **CATORA_Allocator_Standalone.py** - CAT-ORA Standalone 구현

### Results (`results/`)

각 알고리즘과 드론 수에 대한 할당 결과:
- `Fair_Hungarian_25/` - Hungarian 25대
- `Fair_Hungarian_36/` - Hungarian 36대
- `CATORA_25/` - CAT-ORA 25대
- `CATORA_36/` - CAT-ORA 36대

## 💡 Usage Examples

### Python에서 직접 사용

```python
# Hungarian 알고리즘
from Allocation.algorithms.Fair_Hungarian_Allocator import HungarianAllocator

allocator = HungarianAllocator()
assignment = allocator.allocate_fair(initial_positions, goal_positions)

# CAT-ORA Standalone
from Allocation.algorithms.CATORA_Allocator import create_allocator

allocator = create_allocator(use_ros2_service=False)
assignment = allocator.allocate(initial_positions, goal_positions)
```

### 할당 결과 파일 읽기

```python
import re

assignment_map = {}
with open('results/CATORA_25/assignment_25_drones.txt', 'r') as f:
    for line in f:
        match = re.search(r'Drone\s+(\d+)\s*->\s*Target\s+(\d+)', line)
        if match:
            drone_id = int(match.group(1))
            target_id = int(match.group(2))
            assignment_map[drone_id] = target_id
```

## 🎯 Integration

이 모듈은 다음 파일들과 통합되어 있습니다:

- `scripts/generate_assignment.py` - 통합 할당 생성기
- `scripts/formations_large.py` - 형상 정의
- `src/planner/plan_manage/launch/scenario_swarm_*.launch.py` - Hungarian 할당 사용
- `src/planner/catora_planner/launch/scenario_catora_*.launch.py` - CAT-ORA 할당 사용

## 📝 Notes

- 할당 결과는 자동으로 `results/` 폴더에 저장됩니다
- 이미 존재하는 할당은 재사용됩니다 (`--force`로 재생성 가능)
- Python 캐시(`__pycache__`)는 자동으로 `.gitignore`에 포함됩니다

---

**Updated**: 2025-11-21 (Reorganized structure)
