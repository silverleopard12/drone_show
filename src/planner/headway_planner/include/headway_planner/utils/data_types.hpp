#ifndef HEADWAY_PLANNER_DATA_TYPES_HPP
#define HEADWAY_PLANNER_DATA_TYPES_HPP

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace headway_planner {

/**
 * @brief Configuration for headway planner
 */
struct PlannerConfig {
  // Dynamics limits
  double v_max = 3.0;           // Maximum horizontal velocity (m/s)
  double a_max = 2.0;           // Maximum horizontal acceleration (m/s²)
  double j_max = 3.0;           // Maximum jerk (m/s³)
  double v_z_max = 1.5;         // Maximum vertical velocity (m/s)
  double a_z_max = 1.5;         // Maximum vertical acceleration (m/s²)

  // Safety requirements
  double d_min = 2.0;           // Minimum horizontal distance (m)
  double epsilon_t = 0.5;       // Start synchronization error (s)
  double e_pos = 0.3;           // Tracking/positioning error (m)

  // Planning parameters
  double v_des = 2.1;           // Desired planning speed (0.7~0.8 * v_max)
  double alpha = 1.4;           // Safety factor for time separation [1.3, 1.5]

  // Gate for collision detection
  double d_gate = 0.0;          // Auto-computed: d_min + 2*e_pos + v_max*epsilon_t
  double parallel_threshold = 5.0;  // Parallel path threshold (degrees)

  // Altitude slotting
  double h_slot = 1.0;          // Altitude slot height (m)
  bool enable_slotting = true;  // Enable altitude slotting
  double slot_threshold = 1.0;  // Time gap threshold for slotting (s)

  // Time limits
  double T_max = 300.0;         // Maximum mission duration (s)

  // Solver selection
  std::string scheduler_type = "greedy";  // "greedy" or "milp"
  int max_iterations = 100;     // Maximum iterations for greedy
  double tolerance = 0.01;      // Convergence tolerance (s)

  // PX4 Mission Generation
  double takeoff_altitude = 5.0;    // Takeoff altitude (m)
  double goal_hover_time = 2.0;     // Hover time at goal (s)
  bool enable_landing = false;      // Enable landing at end
  double waypoint_spacing = 5.0;    // Waypoint spacing for slots (m)

  // Mission Configuration
  std::string mission_source = "formation";  // "formation", "yaml", or "params"

  // Output configuration
  std::string plan_file_prefix = "drone_";
  std::string summary_file = "summary.txt";
  bool save_summary = true;

  // Visualization
  bool enable_visualization = true;
  std::string vis_topic = "/headway_planner/visualization";
  std::string status_topic = "/headway_planner/status";

  // Debugging
  bool verbose = true;
  bool print_events = true;
  bool print_statistics = true;

  void computeDerivedParams() {
    d_gate = d_min + 2.0 * e_pos + v_max * epsilon_t;
    if (v_des <= 0) {
      v_des = 0.75 * v_max;  // Default to 75%
    }
  }
};

/**
 * @brief Drone information
 */
struct DroneInfo {
  int id;

  // Path (straight line)
  Eigen::Vector3d P;            // Start position
  Eigen::Vector3d Q;            // Goal position
  Eigen::Vector3d u_hat;        // Unit direction vector
  double L;                     // Path length

  // Timing
  double phi;                   // Headway (departure offset)
  double t_start;               // Actual start time = phi
  double t_end;                 // Actual end time = phi + L/v_des

  // Altitude slots
  struct AltitudeSlot {
    double s_start;             // Start position along path
    double s_end;               // End position along path
    double h_offset;            // Altitude offset (+/- h_slot)

    AltitudeSlot() : s_start(0), s_end(0), h_offset(0) {}
    AltitudeSlot(double s1, double s2, double h)
      : s_start(s1), s_end(s2), h_offset(h) {}
  };

  std::vector<AltitudeSlot> slots;

  // Slack (余裕)
  double slack;                 // T_max - (phi + L/v_des)

  DroneInfo()
    : id(-1),
      P(Eigen::Vector3d::Zero()),
      Q(Eigen::Vector3d::Zero()),
      u_hat(Eigen::Vector3d::Zero()),
      L(0.0),
      phi(0.0),
      t_start(0.0),
      t_end(0.0),
      slack(0.0) {}

  void computeDerivedValues(double v_des, double T_max) {
    u_hat = (Q - P).normalized();
    L = (Q - P).norm();
    t_start = phi;
    t_end = phi + L / v_des;
    slack = T_max - t_end;
  }

  Eigen::Vector3d getPositionAtS(double s) const {
    return P + u_hat * s;
  }

  Eigen::Vector3d getPositionAtTime(double t, double v_des) const {
    double s = (t - phi) * v_des;
    s = std::max(0.0, std::min(L, s));
    return getPositionAtS(s);
  }
};

/**
 * @brief Collision event
 */
struct CollisionEvent {
  int drone_i;
  int drone_j;

  // Closest point parameters
  double s_i;                   // Position on path i
  double s_j;                   // Position on path j
  double dist_min;              // Minimum distance

  // Ideal passing times (with phi=0)
  double t_i_ideal;
  double t_j_ideal;

  // Actual passing times (with phi)
  double t_i_actual;
  double t_j_actual;

  // Geometry
  double theta;                 // Crossing angle (radians)
  double v_rel;                 // Relative velocity

  // Required time separation
  double delta_t_req;

  // Priority (for greedy)
  double priority;              // Smaller = more urgent

  CollisionEvent()
    : drone_i(-1), drone_j(-1),
      s_i(0), s_j(0), dist_min(0),
      t_i_ideal(0), t_j_ideal(0),
      t_i_actual(0), t_j_actual(0),
      theta(0), v_rel(0),
      delta_t_req(0), priority(0) {}

  bool operator<(const CollisionEvent& other) const {
    return priority < other.priority;  // Min-heap
  }
};

/**
 * @brief Planning result
 */
struct PlanningResult {
  bool success;
  std::string error_message;

  std::vector<DroneInfo> drones;
  std::vector<CollisionEvent> events;

  // Statistics
  double computation_time;      // seconds
  double makespan;              // Total mission duration
  int num_drones;
  int num_events;
  int num_slots_used;
  double max_headway;
  std::string scheduler_type;

  PlanningResult()
    : success(false),
      computation_time(0.0),
      makespan(0.0),
      num_drones(0),
      num_events(0),
      num_slots_used(0),
      max_headway(0.0),
      scheduler_type("unknown") {}
};

/**
 * @brief PX4 Mission Item (QGroundControl format)
 */
struct PX4MissionItem {
  bool autoContinue;
  int command;
  int doJumpId;
  int frame;
  std::vector<double> params;
  std::string type;

  PX4MissionItem()
    : autoContinue(true),
      command(16),  // MAV_CMD_NAV_WAYPOINT
      doJumpId(0),
      frame(3),     // MAV_FRAME_GLOBAL_RELATIVE_ALT
      params(7, 0.0),
      type("SimpleItem") {}
};

/**
 * @brief PX4 .plan file (QGroundControl format)
 */
struct PX4Plan {
  std::string fileType = "Plan";
  int version = 1;
  std::string groundStation = "QGroundControl";
  nlohmann::json geoFence;
  nlohmann::json rallyPoints;

  struct Mission {
    double cruiseSpeed = 3.0;
    double hoverSpeed = 2.0;
    int firmwareType = 12;  // PX4 Pro
    int vehicleType = 2;    // Quadcopter
    std::vector<double> plannedHomePosition;
    std::vector<PX4MissionItem> items;
  } mission;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_DATA_TYPES_HPP
