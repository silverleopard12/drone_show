#ifndef ORCA_PLANNER_TRAJECTORY_GENERATOR_HPP
#define ORCA_PLANNER_TRAJECTORY_GENERATOR_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>

#include "orca_planner/orca_solver/orca_3d.hpp"
#include "orca_planner/spatial_hash/spatial_hash_3d.hpp"

namespace orca_planner {

/**
 * @brief Waypoint for trajectory
 */
struct Waypoint {
  Eigen::Vector3d position;
  double time;
  double yaw;

  Waypoint() : position(Eigen::Vector3d::Zero()), time(0.0), yaw(0.0) {}
  Waypoint(const Eigen::Vector3d& pos, double t = 0.0, double y = 0.0)
    : position(pos), time(t), yaw(y) {}
};

/**
 * @brief Formation definition
 */
struct Formation {
  std::string name;
  std::vector<Eigen::Vector3d> positions;  // Position for each drone
  double start_time;
  double duration;
  double transition_time;  // Time to transition to this formation

  Formation() : start_time(0.0), duration(5.0), transition_time(3.0) {}
};

/**
 * @brief Trajectory for a single drone
 */
struct DroneTrajectory {
  int drone_id;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<double> timestamps;
  std::vector<double> yaws;

  void clear() {
    positions.clear();
    velocities.clear();
    timestamps.clear();
    yaws.clear();
  }

  size_t size() const { return positions.size(); }

  bool empty() const { return positions.empty(); }
};

/**
 * @brief Configuration for trajectory generation
 */
struct TrajectoryGeneratorConfig {
  // Time parameters
  double dt = 0.1;                  // Time step
  double total_duration = 60.0;     // Total mission duration

  // ORCA parameters
  ORCAConfig orca_config;

  // Goal reaching
  double goal_tolerance = 0.5;      // Distance to consider goal reached
  double hover_time = 1.0;          // Time to hover at goal

  // Smoothing (optional)
  bool enable_smoothing = true;
  int smoothing_window = 5;
};

/**
 * @brief ORCA-based trajectory generator
 *
 * Generates collision-free trajectories for swarms using ORCA algorithm.
 * Simulates all drones forward in time while ensuring no collisions.
 */
class ORCATrajectoryGenerator {
public:
  using Ptr = std::shared_ptr<ORCATrajectoryGenerator>;

  ORCATrajectoryGenerator();
  explicit ORCATrajectoryGenerator(const TrajectoryGeneratorConfig& config);
  ~ORCATrajectoryGenerator();

  /**
   * @brief Set configuration
   */
  void setConfig(const TrajectoryGeneratorConfig& config);

  /**
   * @brief Generate trajectories for simple point-to-point mission
   *
   * @param start_positions Starting positions for all drones
   * @param goal_positions Goal positions for all drones
   * @param trajectories Output trajectories
   * @return true if successful
   */
  bool generateTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Generate trajectories through multiple formations
   *
   * @param start_positions Starting positions
   * @param formations Sequence of formations to visit
   * @param trajectories Output trajectories
   * @return true if successful
   */
  bool generateFormationTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Formation>& formations,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Generate trajectories through waypoints
   *
   * @param waypoints_per_drone Waypoints for each drone
   * @param trajectories Output trajectories
   * @return true if successful
   */
  bool generateWaypointTrajectories(
    const std::vector<std::vector<Waypoint>>& waypoints_per_drone,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Save trajectories to file
   */
  bool saveTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    const std::string& filename
  ) const;

  /**
   * @brief Load trajectories from file
   */
  bool loadTrajectories(
    const std::string& filename,
    std::vector<DroneTrajectory>& trajectories
  ) const;

  /**
   * @brief Get statistics from last generation
   */
  struct Statistics {
    double computation_time;     // seconds
    int total_steps;
    int collision_checks;
    double average_speed;
    double max_speed;
    bool success;
    std::string error_message;

    Statistics() : computation_time(0.0), total_steps(0),
                   collision_checks(0), average_speed(0.0),
                   max_speed(0.0), success(false) {}
  };

  const Statistics& getStatistics() const { return statistics_; }

private:
  TrajectoryGeneratorConfig config_;
  ORCA3D::Ptr orca_solver_;
  SpatialHash3D::Ptr spatial_hash_;
  Statistics statistics_;

  /**
   * @brief Main simulation loop
   *
   * Simulates all drones forward in time using ORCA
   */
  bool simulateORCA(
    std::vector<AgentState>& agents,
    const std::vector<std::vector<Waypoint>>& waypoints_per_drone,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Compute preferred velocity towards next waypoint
   */
  Eigen::Vector3d computePreferredVelocity(
    const AgentState& agent,
    const std::vector<Waypoint>& waypoints,
    size_t& current_waypoint_idx
  ) const;

  /**
   * @brief Check if agent reached waypoint
   */
  bool reachedWaypoint(
    const Eigen::Vector3d& position,
    const Waypoint& waypoint
  ) const;

  /**
   * @brief Apply smoothing to trajectory
   */
  void smoothTrajectory(DroneTrajectory& trajectory) const;

  /**
   * @brief Compute yaw from velocity
   */
  double computeYawFromVelocity(const Eigen::Vector3d& velocity) const;

  /**
   * @brief Validate trajectories for safety
   */
  bool validateTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    std::string& error_message
  ) const;
};

} // namespace orca_planner

#endif // ORCA_PLANNER_TRAJECTORY_GENERATOR_HPP
