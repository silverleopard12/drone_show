# Key Files Reference for Collision Avoidance Implementation

## File Structure and Key Functions

### 1. STATIC OBSTACLE HANDLING

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_env/include/plan_env/grid_map.h`
**Key Classes/Methods**:
- `GridMap::getInflateOccupancy()` - Query inflated occupancy (line 357)
- `GridMap::getOccupancy()` - Query raw occupancy (line 346)
- `GridMap::inflatePoint()` - Inflate obstacles (line 419)
- `MappingParameters::obstacles_inflation_` - Inflation parameter (line 58)

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp`
- Depth image projection and raycasting
- Local map updates and inflation

---

### 2. DYNAMIC OBSTACLE HANDLING

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_env/include/plan_env/obj_predictor.h`
**Key Classes**:
- `ObjPredictor` - Main obstacle predictor (line 146)
  - `evaluateConstVel()` - Constant velocity evaluation (line 182)
  - `evaluatePoly()` - Polynomial evaluation (line 181)
- `PolynomialPrediction` - Trajectory polynomial (line 49)
  - `evaluate()` - Polynomial position at time t (line 84)
  - `evaluateConstVel()` - Linear motion model (line 95)

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_env/src/obj_predictor.cpp`
- `predictPolyFit()` - Fit 5th-order polynomials
- `predictConstVel()` - Constant velocity prediction

---

### 3. MAIN COLLISION AVOIDANCE OPTIMIZER

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`
**Key Classes/Methods**:
- `BsplineOptimizer` - Main optimizer class (line 85)
- `ControlPoints` - Control points with collision info (line 21)
  - `base_point[]` - Points where collision detected (line 27)
  - `direction[]` - Push-away direction vectors (line 28)
- `setEnvironment()` - Set grid map and obstacles (line 93-94)
- `setSwarmTrajs()` - Set other drones' trajectories (line 104)
- `setDroneId()` - Set current drone ID (line 105)

**Parameters**:
- `dist0_` - Minimum safe distance (line 165)
- `swarm_clearance_` - Inter-drone safety distance (line 165)
- `lambda1_`, `lambda2_`, `lambda3_`, `lambda4_` - Cost weights (line 158-161)

---

### 4. COLLISION COST FUNCTIONS

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp`

#### Static Obstacle Cost (Distance Cost)
- **Function**: `calcDistanceCostRebound()` (line 962)
- **Purpose**: Penalizes control points near obstacles
- **Method**: Elastic band deformation
- **Output**: Cost and gradient for L-BFGS optimization

#### Drone-to-Drone Avoidance Cost
- **Function**: `calcSwarmCost()` (line 874)
- **Key Algorithm**:
  - Ellipsoidal safety constraint (lines 878-900)
  - Checks drone positions at discrete time intervals
  - Iterates through all other drones in swarm buffer
  - Quadratic penalty: `cost += (CLEARANCE - ellip_dist)^2`

#### Dynamic Obstacle Cost
- **Function**: `calcMovingObjCost()` (line 927)
- **CLEARANCE**: 1.5m (line 931)
- **Method**: Quadratic penalty for distance violations

#### Check and Rebound (Emergency Collision Handling)
- **Function**: `check_collision_and_rebound()` (line 1298)
- **Steps**:
  1. Detect collision points via `getInflateOccupancy()` (line 1311)
  2. Find collision segments (lines 1308-1370)
  3. Use A-star to find escape path (line 1379)
  4. Calculate push-away directions (lines 1456, 1475)
  5. Store as base_point and direction for gradient (line 1456)

---

### 5. SWARM COORDINATION AND COMMUNICATION

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp`

**Swarm Trajectory Publishing**:
- Publisher topic: `/drone_X_planning/swarm_trajs` (line 105)
- Message type: `traj_utils::msg::MultiBsplines`

**Swarm Trajectory Subscription**:
- Drone i subscribes to: `/drone_(i-1)_planning/swarm_trajs` (line 83)
- Callback: `swarmTrajsCallback()` (line 357)

**Swarm Buffer Management**:
- Data structure: `planner_manager_->swarm_trajs_buf_` (vector of `OneTrajDataOfSwarm`)
- Usage in optimizer: `bspline_optimizer_->setSwarmTrajs()` (line 41)

**Key Callback Functions**:
- `swarmTrajsCallback()` - Process received trajectories (line 357)
  - Stores trajectory as UniformBspline (line 344)
  - Maintains start_time and drone_id (lines 327, 348)

---

### 6. TRAJECTORY DATA STRUCTURE

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/traj_utils/include/traj_utils/plan_container.hpp`

**Swarm Trajectory Data**:
```cpp
struct OneTrajDataOfSwarm (line 218)
  - drone_id
  - duration_
  - start_time_
  - start_pos_
  - position_traj_ (UniformBspline)

typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData (line 229)
```

---

### 7. PATH SEARCHING AND REBOUND

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/path_searching/include/path_searching/dyn_a_star.h`

**AStar Class**:
- `AstarSearch()` - Find collision-free path (line 86)
- `getPath()` - Get computed path points (line 88)
- `checkOccupancy()` - Check occupancy via grid map (line 62)

**Usage in bspline_optimizer**:
- Member: `a_star_` (line 117 in bspline_optimizer.h)
- Used in `check_collision_and_rebound()` (line 1379)

---

### 8. LAYER-BASED SWARM AVOIDANCE

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/layer_planner/include/layer_planner/layer_planner_node.hpp`

**Configuration**:
```cpp
struct LayerConfig (line 28)
  - base_height
  - layer_spacing
  - safety_clearance
  - num_layers
```

**Key Methods**:
- `assignLayer()` - Assign drone to vertical layer (line 54)
- `isLayerAvailable()` - Check layer occupancy (line 55)
- `releaseLayer()` - Release assigned layer (line 56)
- `checkCollisions()` - Distance-based collision detection (line 90)

**Collision Detection** (cpp file, line 736):
- `collision_threshold_` - Distance threshold (1.0m)
- Pairwise distance check: `distance < collision_threshold_`
- Hysteresis: `distance > collision_threshold_ * 1.5` to clear

---

### 9. PLANNER MANAGER

**File**: `/home/pjh/ego_swarm/ego-planner-swarm/src/planner/plan_manage/include/ego_planner/planner_manager.h`

**Key Methods**:
- `reboundReplan()` - Main planning entry point (line 31)
- `checkCollision()` - Collision validation (line 47)

**Members**:
- `bspline_optimizer_` - The optimizer instance
- `grid_map_` - Occupancy grid
- `swarm_trajs_buf_` - Swarm trajectory buffer
- `obj_predictor_` - Dynamic obstacle predictor

---

### 10. UNIFIED B-SPLINE MESSAGE FORMAT

**Message Definition**: `src/planner/traj_utils/msg/Bspline.msg`
- Contains B-spline control points and knots
- Includes drone_id and start_time
- Transmitted between drones for collision avoidance

---

## Quick Reference: Finding Specific Implementations

| What | Where | Line |
|-----|-------|------|
| Inflated occupancy check | grid_map.h | 357 |
| Swarm collision cost | bspline_optimizer.cpp | 874 |
| Moving obstacle cost | bspline_optimizer.cpp | 927 |
| Static obstacle cost | bspline_optimizer.cpp | 962 |
| Check and rebound | bspline_optimizer.cpp | 1298 |
| Swarm trajectory pub | ego_replan_fsm.cpp | 105 |
| Swarm trajectory sub | ego_replan_fsm.cpp | 83-90 |
| Layer assignment | layer_planner_node.cpp | 54 |
| Collision detection | layer_planner_node.cpp | 736 |
| A-star search | dyn_a_star.h | 86 |
| Ellipsoidal constraint | bspline_optimizer.cpp | 899 |

