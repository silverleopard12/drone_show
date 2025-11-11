#ifndef DL_PLANNER_OFFLINE_TRAJECTORY_GENERATOR_HPP
#define DL_PLANNER_OFFLINE_TRAJECTORY_GENERATOR_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "dl_planner/neural_network/neural_collision_net.hpp"
#include "dl_planner/collision_checker/swarm_collision_checker.hpp"

namespace dl_planner {

/**
 * @brief Trajectory structure for a single drone
 */
struct DroneTrajectory {
  int drone_id;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<Eigen::Vector3d> accelerations;
  std::vector<double> timestamps;
  double total_time;

  void clear() {
    positions.clear();
    velocities.clear();
    accelerations.clear();
    timestamps.clear();
    total_time = 0.0;
  }

  size_t size() const { return positions.size(); }
};

/**
 * @brief Configuration for trajectory generation
 */
struct TrajectoryConfig {
  // Time parameters
  double dt = 0.1;           // Time step
  double max_duration = 60.0; // Maximum trajectory duration

  // Dynamic constraints
  double max_velocity = 2.0;     // m/s
  double max_acceleration = 2.0; // m/s^2

  // Safety parameters
  double safety_distance = 1.0;  // Minimum distance between drones
  double collision_threshold = 0.5; // Collision probability threshold

  // Optimization parameters
  int max_iterations = 1000;
  double convergence_threshold = 0.01;
  double learning_rate = 0.1;
};

/**
 * @brief Offline trajectory generator for drone swarms
 *
 * This class generates collision-free trajectories for multiple drones
 * using deep learning-based collision prediction and optimization.
 */
class OfflineTrajectoryGenerator {
public:
  using Ptr = std::shared_ptr<OfflineTrajectoryGenerator>;

  OfflineTrajectoryGenerator();
  ~OfflineTrajectoryGenerator();

  /**
   * @brief Initialize the generator
   */
  bool initialize(const TrajectoryConfig& config);

  /**
   * @brief Set the neural network model
   */
  void setNeuralNetwork(NeuralCollisionNet::Ptr network);

  /**
   * @brief Set the collision checker
   */
  void setCollisionChecker(std::shared_ptr<SwarmCollisionChecker> checker);

  /**
   * @brief Generate trajectories for all drones
   * @param start_positions Starting positions for each drone
   * @param goal_positions Goal positions for each drone
   * @param trajectories Output trajectories
   * @return true if successful
   */
  bool generateTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Generate trajectory for a single drone
   * @param drone_id Drone identifier
   * @param start Starting position
   * @param goal Goal position
   * @param other_trajectories Trajectories of other drones (for collision checking)
   * @param trajectory Output trajectory
   * @return true if successful
   */
  bool generateSingleTrajectory(
    int drone_id,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    const std::vector<DroneTrajectory>& other_trajectories,
    DroneTrajectory& trajectory
  );

  /**
   * @brief Optimize trajectory to minimize collision risk
   */
  bool optimizeTrajectory(
    DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories
  );

  /**
   * @brief Validate trajectory for safety
   */
  bool validateTrajectory(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories
  );

  /**
   * @brief Save trajectories to file
   */
  bool saveTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    const std::string& filename
  );

  /**
   * @brief Load trajectories from file
   */
  bool loadTrajectories(
    const std::string& filename,
    std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Get configuration
   */
  const TrajectoryConfig& getConfig() const { return config_; }

private:
  TrajectoryConfig config_;
  NeuralCollisionNet::Ptr neural_network_;
  std::shared_ptr<SwarmCollisionChecker> collision_checker_;

  /**
   * @brief Generate initial trajectory using simple interpolation
   */
  void generateInitialTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    DroneTrajectory& trajectory
  );

  /**
   * @brief Compute collision cost
   */
  double computeCollisionCost(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories
  );

  /**
   * @brief Compute smoothness cost
   */
  double computeSmoothnessCost(const DroneTrajectory& trajectory);

  /**
   * @brief Apply dynamic constraints
   */
  void applyDynamicConstraints(DroneTrajectory& trajectory);
};

} // namespace dl_planner

#endif // DL_PLANNER_OFFLINE_TRAJECTORY_GENERATOR_HPP
