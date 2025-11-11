#ifndef RL_PLANNER_SWARM_ENV_HPP
#define RL_PLANNER_SWARM_ENV_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>
#include <unordered_map>

namespace rl_planner {

/**
 * @brief Configuration for RL environment
 */
struct EnvConfig {
  // World bounds
  Eigen::Vector3d world_min = Eigen::Vector3d(-50, -50, 0);
  Eigen::Vector3d world_max = Eigen::Vector3d(50, 50, 20);

  // Drone dynamics
  double max_velocity = 3.0;      // m/s
  double max_acceleration = 2.0;  // m/s^2
  double dt = 0.1;                // Time step

  // Safety
  double safety_distance = 2.0;   // Minimum distance between drones
  double collision_penalty = -100.0;
  double goal_reward = 100.0;
  double goal_tolerance = 0.5;

  // Episode
  int max_steps = 600;  // 60 seconds at 0.1s timestep

  // Multi-agent
  int num_drones = 10;
  bool centralized_training = true;  // CTDE (Centralized Training Decentralized Execution)
  bool use_communication = true;     // Allow drone-to-drone communication
  double communication_range = 10.0; // meters

  // Observation
  int num_neighbors = 5;  // Number of nearest neighbors to observe
  bool use_relative_coords = true;

  // Reward shaping
  double distance_reward_scale = 1.0;
  double collision_reward_scale = 1.0;
  double smoothness_reward_scale = 0.1;
  double formation_reward_scale = 0.5;
};

/**
 * @brief State of a single drone
 */
struct DroneState {
  int id;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d goal;
  bool reached_goal;
  bool collided;
  double time_in_air;

  DroneState()
    : id(-1),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      goal(Eigen::Vector3d::Zero()),
      reached_goal(false),
      collided(false),
      time_in_air(0.0) {}
};

/**
 * @brief Action for a single drone (velocity command)
 */
struct DroneAction {
  Eigen::Vector3d velocity_command;  // Desired velocity

  DroneAction() : velocity_command(Eigen::Vector3d::Zero()) {}
  explicit DroneAction(const Eigen::Vector3d& vel) : velocity_command(vel) {}
};

/**
 * @brief Observation for a single drone
 */
struct DroneObservation {
  // Ego state
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d goal;
  double distance_to_goal;

  // Neighbor states (relative to ego)
  std::vector<Eigen::Vector3d> neighbor_positions;
  std::vector<Eigen::Vector3d> neighbor_velocities;
  std::vector<double> neighbor_distances;

  // Communication (if enabled)
  std::vector<Eigen::Vector3d> neighbor_goals;

  /**
   * @brief Convert to feature vector for neural network
   */
  Eigen::VectorXd toFeatureVector() const;

  /**
   * @brief Get feature dimension
   */
  static int getFeatureDim(int num_neighbors, bool use_communication);
};

/**
 * @brief Step result containing observation, reward, done flag
 */
struct StepResult {
  DroneObservation observation;
  double reward;
  bool done;
  std::unordered_map<std::string, double> info;  // Additional info (e.g., collision, goal_reached)
};

/**
 * @brief Multi-Agent Swarm Environment
 *
 * This environment simulates a swarm of drones navigating to goals
 * while avoiding collisions with each other.
 */
class SwarmEnv {
public:
  using Ptr = std::shared_ptr<SwarmEnv>;

  explicit SwarmEnv(const EnvConfig& config);
  ~SwarmEnv();

  /**
   * @brief Reset environment to initial state
   * @return Initial observations for all drones
   */
  std::vector<DroneObservation> reset();

  /**
   * @brief Reset with specific start and goal positions
   */
  std::vector<DroneObservation> reset(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions
  );

  /**
   * @brief Step environment with actions for all drones
   * @param actions Actions for each drone
   * @return Step results for each drone
   */
  std::vector<StepResult> step(const std::vector<DroneAction>& actions);

  /**
   * @brief Get observation for a single drone
   */
  DroneObservation getObservation(int drone_id) const;

  /**
   * @brief Get all drone states
   */
  const std::vector<DroneState>& getDroneStates() const { return drone_states_; }

  /**
   * @brief Get current timestep
   */
  int getCurrentStep() const { return current_step_; }

  /**
   * @brief Check if episode is done
   */
  bool isDone() const;

  /**
   * @brief Get environment configuration
   */
  const EnvConfig& getConfig() const { return config_; }

  /**
   * @brief Set random seed
   */
  void setSeed(int seed);

  /**
   * @brief Get statistics
   */
  struct Statistics {
    int total_collisions;
    int drones_reached_goal;
    double average_time_to_goal;
    double total_distance_traveled;
    double episode_reward;
  };
  Statistics getStatistics() const;

private:
  EnvConfig config_;
  std::vector<DroneState> drone_states_;
  int current_step_;
  std::mt19937 rng_;

  Statistics stats_;

  /**
   * @brief Initialize drone states randomly
   */
  void initializeDrones();

  /**
   * @brief Update drone physics
   */
  void updateDronePhysics(int drone_id, const DroneAction& action);

  /**
   * @brief Check collisions
   */
  bool checkCollision(int drone_id);

  /**
   * @brief Check if drone reached goal
   */
  bool checkGoalReached(int drone_id);

  /**
   * @brief Compute reward for a drone
   */
  double computeReward(int drone_id, const DroneAction& action);

  /**
   * @brief Get nearest neighbors
   */
  std::vector<int> getNearestNeighbors(int drone_id, int k) const;

  /**
   * @brief Generate random position within bounds
   */
  Eigen::Vector3d randomPosition();

  /**
   * @brief Check if position is valid (within bounds)
   */
  bool isValidPosition(const Eigen::Vector3d& pos) const;
};

} // namespace rl_planner

#endif // RL_PLANNER_SWARM_ENV_HPP
