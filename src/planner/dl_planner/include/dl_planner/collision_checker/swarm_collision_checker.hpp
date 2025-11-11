#ifndef DL_PLANNER_SWARM_COLLISION_CHECKER_HPP
#define DL_PLANNER_SWARM_COLLISION_CHECKER_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace dl_planner {

// Forward declaration
struct DroneTrajectory;

/**
 * @brief Collision information structure
 */
struct CollisionInfo {
  bool has_collision;
  int drone_id_1;
  int drone_id_2;
  double time;
  Eigen::Vector3d position;
  double distance;
};

/**
 * @brief Swarm collision checker
 *
 * This class checks for collisions between drones in a swarm
 * and provides collision information for trajectory optimization.
 */
class SwarmCollisionChecker {
public:
  using Ptr = std::shared_ptr<SwarmCollisionChecker>;

  SwarmCollisionChecker();
  ~SwarmCollisionChecker();

  /**
   * @brief Set safety distance threshold
   */
  void setSafetyDistance(double distance);

  /**
   * @brief Check collision between two positions
   * @return true if collision detected
   */
  bool checkCollision(
    const Eigen::Vector3d& pos1,
    const Eigen::Vector3d& pos2,
    double safety_distance = -1.0
  );

  /**
   * @brief Check collision between two trajectories
   * @param traj1 First trajectory
   * @param traj2 Second trajectory
   * @param collisions Output vector of collision information
   * @return true if any collision detected
   */
  bool checkTrajectoryCollision(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    std::vector<CollisionInfo>& collisions
  );

  /**
   * @brief Check collision for one trajectory against multiple others
   * @param trajectory Trajectory to check
   * @param other_trajectories Other trajectories in the swarm
   * @param collisions Output vector of collision information
   * @return true if any collision detected
   */
  bool checkSwarmCollision(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories,
    std::vector<CollisionInfo>& collisions
  );

  /**
   * @brief Check all trajectories for collisions
   * @param trajectories All drone trajectories
   * @param collisions Output vector of all collision information
   * @return true if any collision detected
   */
  bool checkAllCollisions(
    const std::vector<DroneTrajectory>& trajectories,
    std::vector<CollisionInfo>& collisions
  );

  /**
   * @brief Get minimum distance between two trajectories
   */
  double getMinimumDistance(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    double& time_at_min_distance
  );

  /**
   * @brief Get distance at specific time
   */
  double getDistanceAtTime(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    double time
  );

  /**
   * @brief Get collision-free segments
   * @return Vector of time intervals without collisions
   */
  std::vector<std::pair<double, double>> getCollisionFreeSegments(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories
  );

  /**
   * @brief Compute collision risk map
   * @return 3D grid of collision probabilities
   */
  Eigen::MatrixXd computeCollisionRiskMap(
    const std::vector<DroneTrajectory>& trajectories,
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    double resolution
  );

  /**
   * @brief Get statistics
   */
  int getTotalCollisionChecks() const { return total_checks_; }
  int getCollisionCount() const { return collision_count_; }
  void resetStatistics();

private:
  double safety_distance_;
  int total_checks_;
  int collision_count_;

  /**
   * @brief Interpolate position at given time
   */
  Eigen::Vector3d interpolatePosition(
    const DroneTrajectory& trajectory,
    double time
  ) const;

  /**
   * @brief Find closest point of approach
   */
  void findClosestPointOfApproach(
    const Eigen::Vector3d& pos1,
    const Eigen::Vector3d& vel1,
    const Eigen::Vector3d& pos2,
    const Eigen::Vector3d& vel2,
    double& time_to_cpa,
    double& distance_at_cpa
  );
};

} // namespace dl_planner

#endif // DL_PLANNER_SWARM_COLLISION_CHECKER_HPP
