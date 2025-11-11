#include "dl_planner/trajectory_generator/offline_trajectory_generator.hpp"
#include <iostream>
#include <fstream>
#include <chrono>

namespace dl_planner {

OfflineTrajectoryGenerator::OfflineTrajectoryGenerator() {
}

OfflineTrajectoryGenerator::~OfflineTrajectoryGenerator() {
}

bool OfflineTrajectoryGenerator::initialize(const TrajectoryConfig& config) {
  config_ = config;
  std::cout << "[OfflineTrajectoryGenerator] Initialized with config:" << std::endl;
  std::cout << "  dt: " << config_.dt << std::endl;
  std::cout << "  max_duration: " << config_.max_duration << std::endl;
  std::cout << "  max_velocity: " << config_.max_velocity << std::endl;
  std::cout << "  safety_distance: " << config_.safety_distance << std::endl;
  return true;
}

void OfflineTrajectoryGenerator::setNeuralNetwork(NeuralCollisionNet::Ptr network) {
  neural_network_ = network;
}

void OfflineTrajectoryGenerator::setCollisionChecker(
    std::shared_ptr<SwarmCollisionChecker> checker) {
  collision_checker_ = checker;
}

bool OfflineTrajectoryGenerator::generateTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions,
    std::vector<DroneTrajectory>& trajectories) {

  if (start_positions.size() != goal_positions.size()) {
    std::cerr << "[OfflineTrajectoryGenerator] Start and goal size mismatch!" << std::endl;
    return false;
  }

  int num_drones = start_positions.size();
  std::cout << "[OfflineTrajectoryGenerator] Generating trajectories for "
            << num_drones << " drones" << std::endl;

  trajectories.clear();
  trajectories.resize(num_drones);

  // Sequential planning: plan for each drone in order
  for (int i = 0; i < num_drones; ++i) {
    std::cout << "[OfflineTrajectoryGenerator] Planning for drone " << i << std::endl;

    // Get previous trajectories for collision checking
    std::vector<DroneTrajectory> prev_trajectories(trajectories.begin(),
                                                     trajectories.begin() + i);

    bool success = generateSingleTrajectory(
        i, start_positions[i], goal_positions[i], prev_trajectories, trajectories[i]);

    if (!success) {
      std::cerr << "[OfflineTrajectoryGenerator] Failed to plan for drone " << i << std::endl;
      return false;
    }
  }

  std::cout << "[OfflineTrajectoryGenerator] All trajectories generated successfully" << std::endl;
  return true;
}

bool OfflineTrajectoryGenerator::generateSingleTrajectory(
    int drone_id,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    const std::vector<DroneTrajectory>& other_trajectories,
    DroneTrajectory& trajectory) {

  trajectory.drone_id = drone_id;
  trajectory.clear();

  // Step 1: Generate initial trajectory
  generateInitialTrajectory(start, goal, trajectory);

  // Step 2: Optimize trajectory
  bool success = optimizeTrajectory(trajectory, other_trajectories);

  if (!success) {
    std::cerr << "[OfflineTrajectoryGenerator] Optimization failed for drone "
              << drone_id << std::endl;
    return false;
  }

  // Step 3: Validate trajectory
  if (!validateTrajectory(trajectory, other_trajectories)) {
    std::cerr << "[OfflineTrajectoryGenerator] Validation failed for drone "
              << drone_id << std::endl;
    return false;
  }

  std::cout << "[OfflineTrajectoryGenerator] Trajectory for drone " << drone_id
            << " completed with " << trajectory.size() << " waypoints" << std::endl;

  return true;
}

bool OfflineTrajectoryGenerator::optimizeTrajectory(
    DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories) {

  // TODO: Implement gradient-based optimization using neural network predictions

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    // Compute costs
    double collision_cost = computeCollisionCost(trajectory, other_trajectories);
    double smoothness_cost = computeSmoothnessCost(trajectory);
    double total_cost = collision_cost + smoothness_cost;

    // Check convergence
    if (total_cost < config_.convergence_threshold) {
      std::cout << "[OfflineTrajectoryGenerator] Converged at iteration " << iter << std::endl;
      break;
    }

    // Update trajectory (gradient descent)
    // TODO: Implement actual optimization step
  }

  // Apply dynamic constraints
  applyDynamicConstraints(trajectory);

  return true;
}

bool OfflineTrajectoryGenerator::validateTrajectory(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories) {

  if (trajectory.positions.empty()) {
    return false;
  }

  // Check dynamic constraints
  for (size_t i = 0; i < trajectory.velocities.size(); ++i) {
    if (trajectory.velocities[i].norm() > config_.max_velocity) {
      std::cerr << "[OfflineTrajectoryGenerator] Velocity constraint violated at " << i << std::endl;
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.accelerations.size(); ++i) {
    if (trajectory.accelerations[i].norm() > config_.max_acceleration) {
      std::cerr << "[OfflineTrajectoryGenerator] Acceleration constraint violated at " << i << std::endl;
      return false;
    }
  }

  // Check collisions
  if (collision_checker_) {
    std::vector<CollisionInfo> collisions;
    for (const auto& other_traj : other_trajectories) {
      if (collision_checker_->checkTrajectoryCollision(trajectory, other_traj, collisions)) {
        std::cerr << "[OfflineTrajectoryGenerator] Collision detected!" << std::endl;
        return false;
      }
    }
  }

  return true;
}

bool OfflineTrajectoryGenerator::saveTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    const std::string& filename) {

  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[OfflineTrajectoryGenerator] Failed to open file: " << filename << std::endl;
    return false;
  }

  // Write header
  file << "# Drone Show Trajectories" << std::endl;
  file << "# Number of drones: " << trajectories.size() << std::endl;
  file << std::endl;

  // Write each trajectory
  for (const auto& traj : trajectories) {
    file << "DRONE " << traj.drone_id << std::endl;
    file << "POINTS " << traj.size() << std::endl;

    for (size_t i = 0; i < traj.positions.size(); ++i) {
      file << traj.timestamps[i] << " "
           << traj.positions[i].x() << " "
           << traj.positions[i].y() << " "
           << traj.positions[i].z() << " "
           << traj.velocities[i].x() << " "
           << traj.velocities[i].y() << " "
           << traj.velocities[i].z() << std::endl;
    }
    file << std::endl;
  }

  file.close();
  std::cout << "[OfflineTrajectoryGenerator] Trajectories saved to: " << filename << std::endl;
  return true;
}

bool OfflineTrajectoryGenerator::loadTrajectories(
    const std::string& filename,
    std::vector<DroneTrajectory>& trajectories) {

  // TODO: Implement trajectory loading
  std::cout << "[OfflineTrajectoryGenerator] Loading trajectories from: " << filename << std::endl;

  return true;
}

void OfflineTrajectoryGenerator::generateInitialTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    DroneTrajectory& trajectory) {

  // Simple linear interpolation for initial trajectory
  double distance = (goal - start).norm();
  double duration = distance / (config_.max_velocity * 0.5); // Use 50% of max velocity
  int num_points = static_cast<int>(duration / config_.dt) + 1;

  trajectory.positions.resize(num_points);
  trajectory.velocities.resize(num_points);
  trajectory.accelerations.resize(num_points);
  trajectory.timestamps.resize(num_points);

  for (int i = 0; i < num_points; ++i) {
    double t = i * config_.dt;
    double s = t / duration; // Normalized time [0, 1]

    trajectory.timestamps[i] = t;
    trajectory.positions[i] = start + s * (goal - start);
    trajectory.velocities[i] = (goal - start) / duration;
    trajectory.accelerations[i] = Eigen::Vector3d::Zero();
  }

  trajectory.total_time = duration;
}

double OfflineTrajectoryGenerator::computeCollisionCost(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories) {

  double cost = 0.0;

  // Use neural network for collision prediction
  if (neural_network_ && neural_network_->isModelLoaded()) {
    // TODO: Implement neural network-based collision cost
  }

  // Fallback: use geometric collision checking
  if (collision_checker_) {
    for (const auto& other_traj : other_trajectories) {
      double min_time;
      double min_dist = collision_checker_->getMinimumDistance(
          trajectory, other_traj, min_time);

      if (min_dist < config_.safety_distance) {
        cost += (config_.safety_distance - min_dist) * 10.0;
      }
    }
  }

  return cost;
}

double OfflineTrajectoryGenerator::computeSmoothnessCost(
    const DroneTrajectory& trajectory) {

  double cost = 0.0;

  // Penalize high accelerations
  for (const auto& acc : trajectory.accelerations) {
    cost += acc.squaredNorm();
  }

  // Penalize high jerk (change in acceleration)
  for (size_t i = 1; i < trajectory.accelerations.size(); ++i) {
    Eigen::Vector3d jerk = trajectory.accelerations[i] - trajectory.accelerations[i-1];
    cost += jerk.squaredNorm() * 0.1;
  }

  return cost;
}

void OfflineTrajectoryGenerator::applyDynamicConstraints(DroneTrajectory& trajectory) {
  // Clamp velocities
  for (auto& vel : trajectory.velocities) {
    double vel_norm = vel.norm();
    if (vel_norm > config_.max_velocity) {
      vel = vel / vel_norm * config_.max_velocity;
    }
  }

  // Clamp accelerations
  for (auto& acc : trajectory.accelerations) {
    double acc_norm = acc.norm();
    if (acc_norm > config_.max_acceleration) {
      acc = acc / acc_norm * config_.max_acceleration;
    }
  }
}

} // namespace dl_planner
