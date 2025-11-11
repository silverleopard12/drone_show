#ifndef ORCA_PLANNER_ORCA_3D_HPP
#define ORCA_PLANNER_ORCA_3D_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace orca_planner {

/**
 * @brief Agent state in 3D space
 */
struct AgentState {
  int id;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double radius;

  AgentState()
    : id(-1),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      radius(0.5) {}
};

/**
 * @brief ORCA plane (half-space constraint) in 3D
 *
 * Represents a constraint: (v - point) · normal >= 0
 * All velocities on the positive side of this plane are collision-free
 */
struct ORCAPlane {
  Eigen::Vector3d point;   // A point on the plane
  Eigen::Vector3d normal;  // Normal vector pointing to safe half-space

  ORCAPlane()
    : point(Eigen::Vector3d::Zero()),
      normal(Eigen::Vector3d::Zero()) {}

  ORCAPlane(const Eigen::Vector3d& pt, const Eigen::Vector3d& n)
    : point(pt), normal(n) {}
};

/**
 * @brief ORCA solver configuration
 */
struct ORCAConfig {
  double time_horizon = 3.0;        // Time horizon for collision avoidance
  double neighbor_dist = 5.0;       // Maximum distance to consider neighbors
  int max_neighbors = 10;           // Maximum number of neighbors to consider
  double time_step = 0.1;           // Simulation time step
  double max_speed = 2.0;           // Maximum speed
  double preferred_speed = 1.5;     // Preferred cruising speed
  double goal_radius = 0.2;         // Goal reached threshold
};

/**
 * @brief ORCA 3D solver
 *
 * Implements Optimal Reciprocal Collision Avoidance in 3D space.
 * Each agent computes a collision-free velocity by solving a linear program
 * that respects ORCA constraints from all neighbors.
 */
class ORCA3D {
public:
  using Ptr = std::shared_ptr<ORCA3D>;

  explicit ORCA3D(const ORCAConfig& config = ORCAConfig());
  ~ORCA3D();

  /**
   * @brief Set configuration
   */
  void setConfig(const ORCAConfig& config);
  const ORCAConfig& getConfig() const { return config_; }

  /**
   * @brief Compute ORCA velocity for a single agent
   *
   * @param agent_id ID of the agent
   * @param agents States of all agents
   * @param preferred_velocity Desired velocity towards goal
   * @return Safe velocity that avoids collisions
   */
  Eigen::Vector3d computeVelocity(
    int agent_id,
    const std::vector<AgentState>& agents,
    const Eigen::Vector3d& preferred_velocity
  );

  /**
   * @brief Compute ORCA velocities for all agents
   *
   * @param agents States of all agents
   * @param preferred_velocities Desired velocities for each agent
   * @return Safe velocities for all agents
   */
  std::vector<Eigen::Vector3d> computeVelocities(
    const std::vector<AgentState>& agents,
    const std::vector<Eigen::Vector3d>& preferred_velocities
  );

  /**
   * @brief Get ORCA planes for debugging/visualization
   */
  const std::vector<ORCAPlane>& getORCAPlanes() const { return orca_planes_; }

  /**
   * @brief Get neighbor IDs for debugging
   */
  const std::vector<int>& getNeighbors() const { return neighbor_ids_; }

private:
  ORCAConfig config_;
  std::vector<ORCAPlane> orca_planes_;  // For debugging
  std::vector<int> neighbor_ids_;       // For debugging

  /**
   * @brief Find neighbors within range
   */
  std::vector<int> findNeighbors(
    int agent_id,
    const std::vector<AgentState>& agents
  ) const;

  /**
   * @brief Compute ORCA plane for a pair of agents
   *
   * @param agent Agent A
   * @param neighbor Agent B
   * @return ORCA plane representing the velocity obstacle
   */
  ORCAPlane computeORCAPlane(
    const AgentState& agent,
    const AgentState& neighbor
  ) const;

  /**
   * @brief Solve linear program to find optimal velocity
   *
   * Minimize: ||v - v_pref||^2
   * Subject to: (v - p_i) · n_i >= 0 for all ORCA planes i
   *             ||v|| <= max_speed
   *
   * @param preferred_velocity Desired velocity
   * @param planes ORCA constraint planes
   * @return Optimal collision-free velocity
   */
  Eigen::Vector3d linearProgram3D(
    const Eigen::Vector3d& preferred_velocity,
    const std::vector<ORCAPlane>& planes
  ) const;

  /**
   * @brief Project velocity onto plane if it violates constraint
   */
  Eigen::Vector3d projectOntoPlane(
    const Eigen::Vector3d& velocity,
    const ORCAPlane& plane
  ) const;

  /**
   * @brief Compute time to collision
   */
  double computeTimeToCollision(
    const Eigen::Vector3d& relative_position,
    const Eigen::Vector3d& relative_velocity,
    double combined_radius
  ) const;
};

} // namespace orca_planner

#endif // ORCA_PLANNER_ORCA_3D_HPP
