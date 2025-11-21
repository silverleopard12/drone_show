# CAT-ORA Planner

**Collision-Aware Time-Optimal formation Reshaping Algorithm** planner for drone swarms, integrated into ego-planner-swarm.

## Overview

CAT-ORA Planner is a formation reshaping planner that computes optimal robot-to-goal assignments and collision-free trajectories for multi-robot swarms. It minimizes the maximum trajectory duration (makespan) while avoiding inter-robot collisions.

## Features

- **Collision-Aware Assignment**: Considers robot trajectories during assignment computation
- **Time-Optimal**: Minimizes the maximum trajectory duration (bottleneck optimization)
- **ROS2 Native**: Built for ROS2 Humble without external dependencies (except Eigen3)
- **Service-Based Interface**: Easy integration with existing planners

## Installation

Already installed as part of ego-planner-swarm:

```bash
cd ~/ego_swarm/ego-planner-swarm
colcon build --packages-select catora_planner
source install/setup.bash
```

## Usage

### Launch the Planner

```bash
ros2 launch catora_planner catora_planner.launch.py
```

With custom parameters:

```bash
ros2 launch catora_planner catora_planner.launch.py \
    max_velocity:=3.0 \
    max_acceleration:=2.5 \
    trajectory_dt:=0.1
```

### Call Services

#### 1. Get Assignment Only

```bash
ros2 service call /catora_planner/get_assignment catora_planner/srv/GetAssignment \
  "{initial_configurations: [{x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 1}],
    goal_configurations: [{x: 1, y: 1, z: 1}, {x: 0, y: 1, z: 1}]}"
```

#### 2. Get Reshaping Trajectories

```bash
ros2 service call /catora_planner/get_reshaping_trajectories \
  catora_planner/srv/GetReshapingTrajectories \
  "{initial_configurations: [{x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 1}],
    goal_configurations: [{x: 1, y: 1, z: 1}, {x: 0, y: 1, z: 1}],
    max_velocity: 2.0, max_acceleration: 2.0, trajectory_dt: 0.2}"
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_velocity` | 2.0 | Maximum velocity (m/s) |
| `max_acceleration` | 2.0 | Maximum acceleration (m/s²) |
| `trajectory_dt` | 0.2 | Trajectory time step (s) |

## Services

### `/catora_planner/get_assignment`

Computes optimal robot-to-goal assignment.

**Type**: `catora_planner/srv/GetAssignment`

**Request**:
- `geometry_msgs/Point[] initial_configurations`
- `geometry_msgs/Point[] goal_configurations`

**Response**:
- `int32[] mapping` - mapping[i] = j means robot i → goal j
- `bool success`
- `string message`

### `/catora_planner/get_reshaping_trajectories`

Computes complete trajectories for formation reshaping.

**Type**: `catora_planner/srv/GetReshapingTrajectories`

**Request**:
- `geometry_msgs/Point[] initial_configurations`
- `geometry_msgs/Point[] goal_configurations`
- `float32 trajectory_dt`
- `float32 max_velocity`
- `float32 max_acceleration`

**Response**:
- `catora_planner/Trajectory[] trajectories` - one per robot
- `bool success`
- `string message`

## Algorithm

CAT-ORA uses a branch-and-bound approach to find collision-free assignments:

1. **Bottleneck Assignment**: Minimize max trajectory time using modified Hungarian algorithm
2. **Collision Detection**: Check for inter-robot collisions along assigned trajectories
3. **Branching**: If collisions detected, explore alternative assignments
4. **Trajectory Generation**: Generate time-optimal trajectories respecting kinematic constraints

For details, see: [CAT-ORA Paper](https://arxiv.org/pdf/2412.00603)

## Integration with ego-planner-swarm

This planner is designed to work alongside other planners in the swarm:

- **layer_planner**: Layer-based collision avoidance
- **dl_planner**: Deep learning-based planning
- **orca_planner**: ORCA-based velocity obstacles
- **catora_planner**: Time-optimal formation reshaping ← NEW

## Comparison with Other Methods

| Method | Objective | Collision Avoidance | Time Complexity |
|--------|-----------|-------------------|-----------------|
| Hungarian (LSAP) | Min total distance | ❌ | O(n³) |
| **CAT-ORA** | Min makespan | ✅ | O(n³) ~ O(n⁴) |
| Greedy | Fast assignment | ❌ | O(n²) |

## Example Use Cases

1. **Formation Change**: Transform from grid to triangle
2. **Swarm Reconfiguration**: Dynamic role assignment
3. **Multi-Goal Assignment**: Optimal task allocation

## Citation

```bibtex
@ARTICLE{kratky2025catora,
  author={Kratky, Vit and Penicka, Robert and Horyna, Jiri and Stibinger, Petr and Baca, Tomas and Petrlik, Matej and Stepan, Petr and Saska, Martin},
  journal={IEEE Transactions on Robotics},
  title={CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping for Efficient Robot Coordination in 3-D Environments},
  year={2025},
  volume={41},
  pages={2950-2969},
  doi={10.1109/TRO.2025.3547296}
}
```

## License

GPL-3.0 License

---

**Original Repository**: https://github.com/ctu-mrs/catora
**Ported to ROS2 Humble** for ego-planner-swarm integration
