# Scripts Directory

Organized scripts for ego-planner-swarm drone formation control and trajectory management.

## 📁 Directory Structure

```
scripts/
├── Allocation/          # Drone-to-target assignment algorithms
│   ├── algorithms/      # Hungarian, CAT-ORA implementations
│   ├── results/         # Generated assignments
│   └── docs/            # Detailed algorithm documentation
├── core/               # Core runtime scripts
│   ├── swarm_synchronizer.py
│   ├── swarm_synchronizer_with_planning.py
│   └── mission_timer.py
├── trajectory/         # Trajectory processing
│   ├── launchers/      # Shell scripts for launching
│   ├── sample_trajectories.py
│   └── pad_trajectories.py
├── planning/           # Formation planning pipelines
│   ├── auto_formation_pipeline.py
│   ├── formation_planner_pipeline.py
│   ├── formations_large.py
│   ├── AUTO_PIPELINE_GUIDE.md
│   └── README_PIPELINE.md
└── analysis/           # Analysis and testing
    ├── analyze_mission.py
    └── test_bspline_evaluation.py
```

## 🚀 Quick Start

### 1. Generate Drone Assignment

```bash
# Hungarian algorithm (min-max distance optimization)
python3 generate_assignment.py --algorithm hungarian --num_drones 36

# CAT-ORA algorithm (collision-aware time-optimal)
python3 generate_assignment.py --algorithm catora --num_drones 25

# List existing assignments
python3 generate_assignment.py --list
```

### 2. Run Formation Planning Pipeline

```bash
# Automated pipeline (36 drones, Grid → Triangle)
cd planning/
python3 auto_formation_pipeline.py --num_drones 36

# Custom formations
python3 formation_planner_pipeline.py \
    --current custom_formation_a.txt \
    --target custom_formation_b.txt
```

### 3. Sample Trajectories

```bash
# Sample from running simulation
cd trajectory/
python3 sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 50.0 \
    --duration 60.0

# Or use launcher script
cd trajectory/launchers/
./sample_trajectories_launcher.sh 36 50.0 60.0
```

### 4. Run Complete Mission (Launch + Sample)

```bash
# 25-drone scenario with automatic sampling
cd trajectory/launchers/
./run_and_sample_25.sh
```

## 📂 Module Details

### Allocation/

**Purpose**: Drone-to-target assignment using optimization algorithms

**Key Files**:
- `algorithms/Fair_Hungarian_Allocator.py` - Min-max Hungarian algorithm
- `algorithms/CATORA_Allocator.py` - CAT-ORA interface (Standalone + ROS2)
- `algorithms/CATORA_Allocator_Standalone.py` - Pure Python implementation

**Usage**:
```bash
python3 generate_assignment.py --algorithm hungarian --num_drones 36
python3 generate_assignment.py --algorithm catora --num_drones 25
```

**Outputs**: `Allocation/results/{algorithm}_{num_drones}/assignment_{num_drones}_drones.txt`

See [Allocation/README.md](Allocation/README.md) for detailed documentation.

---

### core/

**Purpose**: Runtime coordination for swarm missions

**Files**:
- **swarm_synchronizer.py** - ROS2 node for multi-drone synchronization
- **swarm_synchronizer_with_planning.py** - Extended synchronizer with planning integration
- **mission_timer.py** - Mission timing and logging utilities

**Usage**:
```bash
# Standalone synchronizer
python3 core/swarm_synchronizer.py --ros-args -p num_drones:=36 -p wait_time:=5.0

# With planning
python3 core/swarm_synchronizer_with_planning.py
```

Used internally by launch files - typically not run directly.

---

### trajectory/

**Purpose**: Trajectory sampling and post-processing

**Files**:
- **sample_trajectories.py** - Extract and sample B-spline trajectories from ROS2 topics
- **pad_trajectories.py** - Pad trajectory files to equal lengths for DroneShow
- **launchers/** - Shell scripts for convenient launching

**Usage**:
```bash
# Sample trajectories from running simulation
python3 trajectory/sample_trajectories.py \
    --num_drones 36 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format all \
    --output_dir trajectories_36_drones

# Pad trajectories to equal length
python3 trajectory/pad_trajectories.py trajectories_36_drones/

# Use launcher script
./trajectory/launchers/sample_trajectories_launcher.sh 36 50.0 60.0
```

**Output Formats**:
- DroneShow format (.csv)
- Raw position data (.txt)
- Metadata and statistics

---

### planning/

**Purpose**: Formation planning and automated pipelines

**Files**:
- **formations_large.py** - Predefined formation definitions (25, 36 drones)
- **auto_formation_pipeline.py** - Fully automated pipeline (assignment → launch → trajectories)
- **formation_planner_pipeline.py** - Manual pipeline with custom formations
- **AUTO_PIPELINE_GUIDE.md** - Detailed guide for automated pipeline
- **README_PIPELINE.md** - Pipeline architecture documentation

**Usage**:
```bash
# Automated pipeline
python3 planning/auto_formation_pipeline.py --num_drones 36

# Custom formations
python3 planning/formation_planner_pipeline.py \
    --current my_formation_a.txt \
    --target my_formation_b.txt \
    --num_drones 36
```

See [planning/AUTO_PIPELINE_GUIDE.md](planning/AUTO_PIPELINE_GUIDE.md) for complete guide.

---

### analysis/

**Purpose**: Mission analysis and testing utilities

**Files**:
- **analyze_mission.py** - Post-mission trajectory analysis and statistics
- **test_bspline_evaluation.py** - B-spline evaluation testing and validation

**Usage**:
```bash
# Analyze mission trajectories
python3 analysis/analyze_mission.py trajectories_36_drones/

# Test B-spline evaluation
python3 analysis/test_bspline_evaluation.py
```

---

## 🔧 Common Workflows

### Workflow 1: Generate and Run Formation Mission

```bash
# 1. Generate assignment
python3 generate_assignment.py --algorithm hungarian --num_drones 36

# 2. Launch ego-swarm (copy assignment to launch file first)
ros2 launch ego_planner scenario_swarm_36.launch.py

# 3. Sample trajectories (in separate terminal)
python3 trajectory/sample_trajectories.py --num_drones 36 --duration 60.0
```

### Workflow 2: Fully Automated Pipeline

```bash
# Everything automated: assignment → launch → trajectories
python3 planning/auto_formation_pipeline.py --num_drones 36
```

### Workflow 3: Custom Formation Development

```bash
# 1. Create formation files (x,y,z per line)
echo "0.0,0.0,10.0" > formation_a.txt
echo "5.0,5.0,15.0" >> formation_a.txt
# ... (add all positions)

# 2. Generate assignment
python3 generate_assignment.py \
    --current formation_a.txt \
    --target formation_b.txt

# 3. Run pipeline
python3 planning/formation_planner_pipeline.py \
    --current formation_a.txt \
    --target formation_b.txt
```

### Workflow 4: Compare Allocation Algorithms

```bash
# Generate both assignments
python3 generate_assignment.py --algorithm hungarian --num_drones 36 --force
python3 generate_assignment.py --algorithm catora --num_drones 36 --force

# Compare results
diff Allocation/results/Fair_Hungarian_36/assignment_36_drones.txt \
     Allocation/results/CATORA_36/assignment_36_drones.txt

# Check statistics in each file
grep "Max distance" Allocation/results/*/assignment_36_drones.txt
```

---

## 📖 Documentation

- **[Allocation/README.md](Allocation/README.md)** - Assignment algorithms
- **[Allocation/docs/COMPARISON.md](Allocation/docs/COMPARISON.md)** - Hungarian vs CAT-ORA comparison
- **[planning/AUTO_PIPELINE_GUIDE.md](planning/AUTO_PIPELINE_GUIDE.md)** - Automated pipeline guide
- **[planning/README_PIPELINE.md](planning/README_PIPELINE.md)** - Pipeline architecture

---

## 🎯 Integration with ego-planner-swarm

These scripts integrate with:

- **Launch files**: `src/planner/plan_manage/launch/scenario_swarm_*.launch.py`
- **CATORA planner**: `src/planner/catora_planner/`
- **Trajectory recorder**: `src/planner/traj_recorder/`

Assignment files are copied into launch files as `drone_configs` lists.

---

## 💡 Tips

1. **Always use --force to regenerate**: If you modify formations, regenerate assignments with `--force`
2. **Check assignment statistics**: Look at max distance to optimize bottleneck
3. **Sample early**: Start sampling ~5-10 seconds after launch to capture full trajectory
4. **Use launchers**: Shell scripts in `trajectory/launchers/` handle ROS2 sourcing automatically
5. **Pad for DroneShow**: Always run `pad_trajectories.py` before importing to DroneShow

---

**Updated**: 2025-11-21 (Reorganized structure)
