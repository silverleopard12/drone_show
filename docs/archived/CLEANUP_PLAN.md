# Folder Cleanup Plan

**작성일**: 2025-11-19

---

## 🗑️ 삭제할 파일 목록

### 1. 중복/타임스탬프 할당 파일 (scripts/)

**이유**: Allocation 폴더로 통합됨

```bash
scripts/assignment_25_drones_20251112_220118.txt  # ❌ DELETE
scripts/assignment_25_drones_20251112_220923.txt  # ❌ DELETE
scripts/assignment_25_drones_20251112_221443.txt  # ❌ DELETE
scripts/assignment_25_drones_20251112_232824.txt  # ❌ DELETE
scripts/assignment_36_drones_20251113_161958.txt  # ❌ DELETE
scripts/assignment_36_drones_20251113_162628.txt  # ❌ DELETE
scripts/assignment_36_drones_20251113_171307.txt  # ❌ DELETE
```

**대체**: `scripts/Allocation/Fair_Hungarian_{25,36}/assignment_{25,36}_drones.txt`

### 2. 중복 스크립트 (scripts/)

**이유**: `generate_assignment.py`로 통합됨

```bash
scripts/generate_25_assignment.py  # ❌ DELETE
scripts/generate_36_assignment.py  # ❌ DELETE
```

**대체**:
```bash
python3 generate_assignment.py --algorithm hungarian --num_drones 25
python3 generate_assignment.py --algorithm hungarian --num_drones 36
```

### 3. 중복 스크립트 (scripts/)

**이유**: catora_planner/scripts/로 이동됨

```bash
scripts/catora_assignment_calculator.py  # ❌ DELETE
```

**대체**: `src/planner/catora_planner/scripts/catora_assignment_calculator.py`

### 4. 중복 문서 (root/)

**이유**: CATORA_INTEGRATION_SUMMARY.md로 통합

```bash
./CATORA_INTEGRATION.md  # ❌ DELETE (중복)
```

**대체**: `CATORA_INTEGRATION_SUMMARY.md`

---

## 📁 정리할 폴더 구조

### Before (현재)

```
ego-planner-swarm/
├── scripts/
│   ├── assignment_25_drones_*.txt (7개 중복 파일)
│   ├── catora_assignment_calculator.py (중복)
│   ├── generate_25_assignment.py (중복)
│   ├── generate_36_assignment.py (중복)
│   ├── generate_assignment.py ✅ KEEP
│   └── Allocation/
│       ├── Fair_Hungarian_25/ ✅ KEEP
│       ├── Fair_Hungarian_36/ ✅ KEEP
│       ├── CATORA_25/ ✅ KEEP
│       └── CATORA_36/ ✅ KEEP
├── CATORA_INTEGRATION.md ❌ DELETE
├── CATORA_INTEGRATION_SUMMARY.md ✅ KEEP
└── ...
```

### After (정리 후)

```
ego-planner-swarm/
├── scripts/
│   ├── generate_assignment.py ✅
│   ├── Allocation/
│   │   ├── Fair_Hungarian_25/ ✅
│   │   ├── Fair_Hungarian_36/ ✅
│   │   ├── CATORA_25/ ✅
│   │   ├── CATORA_36/ ✅
│   │   ├── Fair_Hungarian_Allocator.py ✅
│   │   ├── CATORA_Allocator.py ✅
│   │   └── README.md ✅
│   ├── auto_formation_pipeline.py ✅
│   ├── formation_planner_pipeline.py ✅
│   ├── mission_timer.py ✅
│   ├── swarm_synchronizer.py ✅
│   └── formations_large.py ✅
├── src/planner/
│   ├── catora_planner/ ✅
│   │   └── scripts/
│   │       └── catora_assignment_calculator.py ✅
│   └── ...
├── CATORA_INTEGRATION_SUMMARY.md ✅
├── CATORA_PLANNER_GUIDE.md ✅
└── README.md ✅
```

---

## ✅ 유지할 파일 (scripts/)

### 핵심 스크립트
- ✅ `generate_assignment.py` - 통합 할당 생성기
- ✅ `formations_large.py` - 형상 정의
- ✅ `auto_formation_pipeline.py` - 자동 형상 파이프라인
- ✅ `formation_planner_pipeline.py` - 형상 플래너 파이프라인
- ✅ `mission_timer.py` - 미션 타이머
- ✅ `swarm_synchronizer.py` - 스웜 동기화
- ✅ `analyze_mission.py` - 미션 분석
- ✅ `sample_trajectories_launcher.sh` - 궤적 샘플링
- ✅ `run_and_sample_25.sh` - 실행 및 샘플링

### Allocation 시스템
- ✅ `Allocation/Fair_Hungarian_Allocator.py`
- ✅ `Allocation/CATORA_Allocator.py`
- ✅ `Allocation/README.md`
- ✅ `Allocation/Fair_Hungarian_{25,36}/`
- ✅ `Allocation/CATORA_{25,36}/`

### 문서
- ✅ `AUTO_PIPELINE_GUIDE.md`
- ✅ `README_PIPELINE.md`

---

## ✅ 유지할 파일 (root/)

### 핵심 문서
- ✅ `README.md` - 메인 README
- ✅ `CATORA_INTEGRATION_SUMMARY.md` - CAT-ORA 통합 요약
- ✅ `CATORA_PLANNER_GUIDE.md` - CAT-ORA 플래너 가이드
- ✅ `TODO.md` - TODO 리스트
- ✅ `KEY_FILES_REFERENCE.md` - 핵심 파일 참조
- ✅ `TRAJECTORY_SAMPLING.md` - 궤적 샘플링 가이드
- ✅ `OFFLINE_TRAJECTORY_PLANNING.md` - 오프라인 계획
- ✅ `QUICK_SAMPLING_GUIDE.md` - 빠른 샘플링 가이드

---

## 🔧 정리 스크립트

```bash
#!/bin/bash
# cleanup.sh - Cleanup unused files

cd /home/pjh/ego_swarm/ego-planner-swarm

echo "======================================"
echo "Cleaning up redundant files..."
echo "======================================"

# 1. Remove timestamped assignment files
echo "Removing old assignment files..."
rm -v scripts/assignment_25_drones_20251112_*.txt
rm -v scripts/assignment_36_drones_20251113_*.txt

# 2. Remove duplicate assignment scripts
echo "Removing duplicate scripts..."
rm -v scripts/generate_25_assignment.py
rm -v scripts/generate_36_assignment.py

# 3. Remove duplicate catora calculator (moved to catora_planner/scripts)
echo "Removing moved script..."
rm -v scripts/catora_assignment_calculator.py

# 4. Remove duplicate documentation
echo "Removing duplicate documentation..."
rm -v CATORA_INTEGRATION.md

echo "======================================"
echo "Cleanup complete!"
echo "======================================"

# Show remaining structure
echo ""
echo "Remaining scripts/:"
ls -lh scripts/*.py scripts/*.sh 2>/dev/null | head -20

echo ""
echo "Allocation structure:"
ls -lh scripts/Allocation/

echo ""
echo "Root documentation:"
ls -lh *.md
```

---

## 📊 공간 절약

### 삭제 예상

```
assignment_*_drones_*.txt   ~45 KB  (7개 파일)
generate_{25,36}_assignment  ~7 KB  (2개 파일)
catora_assignment_calculator ~5 KB  (1개 파일)
CATORA_INTEGRATION.md        ~10 KB (1개 파일)
─────────────────────────────────────
Total:                      ~67 KB
```

### 정리 효과

- ✅ 중복 파일 제거
- ✅ 명확한 파일 구조
- ✅ 유지보수 용이
- ✅ 혼란 감소

---

## ⚠️ 주의사항

### 백업 권장

```bash
# 삭제 전 백업 생성
cd /home/pjh/ego_swarm/ego-planner-swarm
tar -czf backup_before_cleanup_$(date +%Y%m%d).tar.gz \
    scripts/assignment_*.txt \
    scripts/generate_{25,36}_assignment.py \
    scripts/catora_assignment_calculator.py \
    CATORA_INTEGRATION.md
```

### 확인 사항

1. ✅ `Allocation/Fair_Hungarian_{25,36}/` 폴더에 할당 파일 존재 확인
2. ✅ `generate_assignment.py`가 정상 작동 확인
3. ✅ `catora_planner/scripts/catora_assignment_calculator.py` 존재 확인
4. ✅ `CATORA_INTEGRATION_SUMMARY.md` 존재 확인

---

## 실행 순서

```bash
# 1. 백업 생성
cd ~/ego_swarm/ego-planner-swarm
tar -czf backup_before_cleanup_$(date +%Y%m%d).tar.gz \
    scripts/assignment_*.txt \
    scripts/generate_{25,36}_assignment.py \
    scripts/catora_assignment_calculator.py \
    CATORA_INTEGRATION.md

# 2. 정리 스크립트 실행 (위의 cleanup.sh 내용 복사하여 실행)
chmod +x cleanup.sh
./cleanup.sh

# 3. 확인
python3 scripts/generate_assignment.py --list
ls -la scripts/Allocation/
```

---

**작성**: Claude Code
**날짜**: 2025-11-19
