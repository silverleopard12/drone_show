#ifndef DL_PLANNER_DATA_TYPES_HPP
#define DL_PLANNER_DATA_TYPES_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dl_planner {

/**
 * @brief Drone state structure
 */
struct DroneState {
  int id;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double timestamp;

  DroneState()
    : id(-1),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()),
      timestamp(0.0) {}
};

/**
 * @brief Swarm state structure
 */
struct SwarmState {
  std::vector<DroneState> drones;
  double timestamp;

  int size() const { return drones.size(); }
  void clear() { drones.clear(); }
};

/**
 * @brief Waypoint structure
 */
struct Waypoint {
  Eigen::Vector3d position;
  double time;
  double yaw;

  Waypoint() : position(Eigen::Vector3d::Zero()), time(0.0), yaw(0.0) {}
  Waypoint(const Eigen::Vector3d& pos, double t, double y = 0.0)
    : position(pos), time(t), yaw(y) {}
};

/**
 * @brief Mission structure for drone show
 */
struct DroneMission {
  int drone_id;
  std::vector<Waypoint> waypoints;
  Eigen::Vector3d start_position;
  Eigen::Vector3d goal_position;
  double start_time;
  double end_time;

  void clear() {
    waypoints.clear();
    start_time = 0.0;
    end_time = 0.0;
  }

  size_t size() const { return waypoints.size(); }
};

/**
 * @brief Swarm mission structure
 */
struct SwarmMission {
  std::vector<DroneMission> missions;
  double total_duration;
  std::string description;

  int size() const { return missions.size(); }
  void clear() {
    missions.clear();
    total_duration = 0.0;
    description = "";
  }
};

/**
 * @brief Planning statistics
 */
struct PlanningStatistics {
  double planning_time;        // seconds
  int iterations;
  int collision_checks;
  int collisions_found;
  bool success;
  std::string failure_reason;

  PlanningStatistics()
    : planning_time(0.0),
      iterations(0),
      collision_checks(0),
      collisions_found(0),
      success(false),
      failure_reason("") {}

  void reset() {
    planning_time = 0.0;
    iterations = 0;
    collision_checks = 0;
    collisions_found = 0;
    success = false;
    failure_reason = "";
  }
};

/**
 * @brief Bounding box structure
 */
struct BoundingBox {
  Eigen::Vector3d min_bound;
  Eigen::Vector3d max_bound;

  BoundingBox()
    : min_bound(Eigen::Vector3d::Zero()),
      max_bound(Eigen::Vector3d::Zero()) {}

  BoundingBox(const Eigen::Vector3d& min_b, const Eigen::Vector3d& max_b)
    : min_bound(min_b), max_bound(max_b) {}

  bool contains(const Eigen::Vector3d& point) const {
    return (point.x() >= min_bound.x() && point.x() <= max_bound.x() &&
            point.y() >= min_bound.y() && point.y() <= max_bound.y() &&
            point.z() >= min_bound.z() && point.z() <= max_bound.z());
  }

  Eigen::Vector3d getSize() const {
    return max_bound - min_bound;
  }

  Eigen::Vector3d getCenter() const {
    return (min_bound + max_bound) / 2.0;
  }
};

} // namespace dl_planner

#endif // DL_PLANNER_DATA_TYPES_HPP
