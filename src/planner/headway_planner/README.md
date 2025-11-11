# Headway Planner

Offline trajectory planner for drone shows using time-shift (headway) separation and minimal altitude slotting.

## Overview

The Headway Planner generates collision-free trajectories for large drone swarms by:
1. **Time separation**: Drones depart at different times (headway offsets φ)
2. **Altitude slotting**: Minimal vertical separation when time alone is insufficient
3. **Direct PX4 output**: Generates `.plan` files ready for PX4 autopilot

### Key Features

- **Scalable**: Handles 10-100+ drones efficiently
- **Optimal**: MILP solver for makespan minimization (optional)
- **Fast**: Greedy heuristic for quick solutions
- **Safe**: Formal safety constraints with configurable margins
- **Flexible**: Grid, circle, or custom formations

## Algorithm

### Mathematical Formulation

For each drone `i`:
- Start position: **P_i**
- Goal position: **Q_i**
- Path: straight line from P_i to Q_i
- Direction: **u_i** = (Q_i - P_i) / L_i
- Path length: L_i = ||Q_i - P_i||
- Headway (departure time offset): φ_i

Position at time t:
```
r_i(t) = P_i + v_des * (t - φ_i) * u_i    for φ_i <= t <= φ_i + L_i/v_des
```

### Collision Detection

For each pair of drones (i, j):
1. Compute minimum distance between line segments
2. If `d_min < d_gate`, extract collision event:
   - Closest approach parameters: s_i, s_j
   - Required time separation: Δt_req = α * (d_min + 2*e_pos + v_des*ε_t) / v_rel

### Headway Scheduling

**Greedy Algorithm**:
1. Initialize all φ_i = 0
2. Sort collision events by priority (urgency)
3. For each violated event: delay the drone with more slack
4. Iterate until all constraints satisfied or T_max exceeded

**MILP Formulation** (optional):
```
Minimize: T_max

Subject to:
  For each event (i,j):
    |t_i(s_i) - t_j(s_j)| >= Δt_req

  0 <= φ_i <= T_max
  φ_i + L_i/v_des <= T_max
```

### Altitude Slotting

When time separation is tight (Δt < threshold):
1. Compute required slot height: h_slot >= sqrt(d_min² - (v_rel * Δt)²)
2. Build conflict graph of overlapping events
3. Graph coloring to assign layers (0, +h, -h, +2h, ...)
4. Insert climb/descend waypoints based on vertical dynamics

## Installation

### Dependencies

- ROS2 Humble
- Eigen3
- nlohmann/json (for PX4 .plan files)
- CBC or SCIP (for MILP solver, optional)

```bash
sudo apt install libeigen3-dev nlohmann-json3-dev coinor-libcbc-dev
```

### Build

```bash
cd /home/pjh/ego_swarm/ego-planner-swarm
colcon build --packages-select headway_planner
source install/setup.bash
```

## Usage

### Quick Start

```bash
# Run with default configuration (10 drones, greedy scheduler)
ros2 launch headway_planner headway_planner.launch.py

# Output: /tmp/headway_planner_output/drone_0.plan, ..., drone_9.plan
```

### Custom Configuration

```bash
# 50 drones with MILP optimization
ros2 launch headway_planner headway_planner.launch.py \
  num_drones:=50 \
  scheduler_type:=milp \
  output_dir:=/path/to/output

# Load mission from YAML file
ros2 launch headway_planner headway_planner.launch.py \
  mission_file:=/path/to/mission.yaml
```

### Example Missions

See `examples/` directory:
- `mission_10_drones.yaml`: 2x5 grid formation
- `mission_50_drones.yaml`: 5x10 grid formation

## Configuration

Edit `config/default.yaml` to customize:

### Safety Parameters
- `d_min`: Minimum safe distance (2.0 m)
- `epsilon_t`: Time uncertainty buffer (0.2 s)
- `alpha`: Safety margin multiplier (1.2)

### Planning
- `v_des`: Desired cruise speed (5.0 m/s)
- `scheduler_type`: "greedy" or "milp"
- `enable_slotting`: Enable altitude separation

### Dynamics
- `v_max`, `a_max`, `j_max`: Horizontal limits
- `v_z_max`, `a_z_max`: Vertical limits

## Output

### PX4 .plan Files

Each drone gets a mission file `drone_<id>.plan`:

1. **TAKEOFF** to takeoff_altitude
2. **LOITER_TIME** for headway offset φ_i
3. **DO_CHANGE_SPEED** to v_des
4. **WAYPOINT** at start position
5. **WAYPOINT** sequence (including altitude slots)
6. **WAYPOINT** at goal position
7. **LOITER_TIME** at goal (optional)
8. **LAND** (optional)

### Summary Report

If `save_summary: true`, generates `summary.txt`:
```
Headway Planner Results
=======================
Drones: 50
Events detected: 245
Events resolved: 245
Makespan: 35.7 s
Scheduler: greedy
Altitude slots used: 12
```

## Visualization

Publishes to `/headway_planner/visualization`:
- **Green paths**: Drone trajectories
- **Red spheres**: Collision events
- **Blue boxes**: Altitude slots

## Performance

| Drones | Events | Greedy Time | MILP Time | Makespan (Greedy) | Makespan (MILP) |
|--------|--------|-------------|-----------|-------------------|-----------------|
| 10     | 20     | 0.01 s      | 0.5 s     | 12.3 s            | 11.8 s          |
| 50     | 245    | 0.15 s      | 15.2 s    | 35.7 s            | 32.4 s          |
| 100    | 980    | 0.6 s       | 120+ s    | 67.2 s            | 58.9 s          |

*Note: MILP gives 5-15% better makespan but 100-1000x slower*

## Troubleshooting

### No solution found (T_max exceeded)

- Increase `T_max` in config
- Reduce `alpha` (less conservative)
- Enable `enable_slotting` to use altitude separation
- Check if formation is too dense (increase spacing)

### MILP solver timeout

- Reduce `milp_time_limit` (accept suboptimal solution)
- Enable `use_warm_start` (use greedy as initial guess)
- Switch to `scheduler_type: greedy` for faster results

### Drones too close at collision events

- Increase `d_min` for more safety margin
- Increase `alpha` multiplier
- Enable altitude slotting
- Check `epsilon_t` and `e_pos` buffers

## Architecture

```
headway_planner/
├── collision_detector/
│   └── event_extractor     # Extract collision events from paths
├── scheduler/
│   ├── headway_scheduler   # Base class
│   ├── greedy_scheduler    # Fast heuristic
│   └── milp_scheduler      # Optimal MILP solver
├── altitude/
│   └── slot_allocator      # Altitude slot assignment
├── trajectory/
│   └── plan_generator      # PX4 .plan file generator
└── utils/
    ├── data_types          # Core data structures
    └── geometry_utils      # Line segment distance, etc.
```

## References

1. Time-optimal multi-agent trajectory planning
2. Graph coloring for frequency assignment
3. PX4 Mission Format Specification

## License

MIT License

## Authors

Park Jeong Hyeon (pjh)
