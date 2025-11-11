#include "orca_planner/trajectory_generator/orca_trajectory_generator.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>

namespace orca_planner {

ORCATrajectoryGenerator::ORCATrajectoryGenerator()
  : config_() {
  orca_solver_ = std::make_shared<ORCA3D>(config_.orca_config);
  spatial_hash_ = std::make_shared<SpatialHash3D>(config_.orca_config.neighbor_dist);
}

ORCATrajectoryGenerator::ORCATrajectoryGenerator(
    const TrajectoryGeneratorConfig& config)
  : config_(config) {
  orca_solver_ = std::make_shared<ORCA3D>(config_.orca_config);
  spatial_hash_ = std::make_shared<SpatialHash3D>(config_.orca_config.neighbor_dist);
}

ORCATrajectoryGenerator::~ORCATrajectoryGenerator() {
}

void ORCATrajectoryGenerator::setConfig(const TrajectoryGeneratorConfig& config) {
  config_ = config;
  orca_solver_->setConfig(config_.orca_config);
}

bool ORCATrajectoryGenerator::generateTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions,
    std::vector<DroneTrajectory>& trajectories) {

  if (start_positions.size() != goal_positions.size()) {
    std::cerr << "[ORCATrajectoryGenerator] Start and goal size mismatch!" << std::endl;
    statistics_.success = false;
    statistics_.error_message = "Size mismatch";
    return false;
  }

  int num_agents = start_positions.size();
  std::cout << "[ORCATrajectoryGenerator] Generating trajectories for "
            << num_agents << " drones" << std::endl;

  // Convert to waypoints
  std::vector<std::vector<Waypoint>> waypoints_per_drone(num_agents);
  for (int i = 0; i < num_agents; ++i) {
    waypoints_per_drone[i].push_back(Waypoint(start_positions[i], 0.0));
    waypoints_per_drone[i].push_back(Waypoint(goal_positions[i],
                                                config_.total_duration));
  }

  return generateWaypointTrajectories(waypoints_per_drone, trajectories);
}

bool ORCATrajectoryGenerator::generateFormationTrajectories(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Formation>& formations,
    std::vector<DroneTrajectory>& trajectories) {

  int num_agents = start_positions.size();

  // Build waypoint sequence from formations
  std::vector<std::vector<Waypoint>> waypoints_per_drone(num_agents);

  // Start positions
  for (int i = 0; i < num_agents; ++i) {
    waypoints_per_drone[i].push_back(Waypoint(start_positions[i], 0.0));
  }

  // Formations
  double current_time = 0.0;
  for (const auto& formation : formations) {
    current_time += formation.transition_time;

    if (static_cast<int>(formation.positions.size()) != num_agents) {
      std::cerr << "[ORCATrajectoryGenerator] Formation size mismatch!" << std::endl;
      return false;
    }

    for (int i = 0; i < num_agents; ++i) {
      waypoints_per_drone[i].push_back(
          Waypoint(formation.positions[i], current_time));
    }

    current_time += formation.duration;
  }

  return generateWaypointTrajectories(waypoints_per_drone, trajectories);
}

bool ORCATrajectoryGenerator::generateWaypointTrajectories(
    const std::vector<std::vector<Waypoint>>& waypoints_per_drone,
    std::vector<DroneTrajectory>& trajectories) {

  auto start_time = std::chrono::high_resolution_clock::now();

  statistics_ = Statistics();

  int num_agents = waypoints_per_drone.size();

  // Initialize agent states
  std::vector<AgentState> agents(num_agents);
  for (int i = 0; i < num_agents; ++i) {
    agents[i].id = i;
    agents[i].position = waypoints_per_drone[i][0].position;
    agents[i].velocity = Eigen::Vector3d::Zero();
    agents[i].radius = config_.orca_config.max_speed * config_.dt;  // Dynamic radius
  }

  // Initialize trajectories
  trajectories.clear();
  trajectories.resize(num_agents);
  for (int i = 0; i < num_agents; ++i) {
    trajectories[i].drone_id = i;
  }

  // Run ORCA simulation
  bool success = simulateORCA(agents, waypoints_per_drone, trajectories);

  if (!success) {
    statistics_.success = false;
    return false;
  }

  // Apply smoothing if enabled
  if (config_.enable_smoothing) {
    for (auto& traj : trajectories) {
      smoothTrajectory(traj);
    }
  }

  // Validate
  std::string error_msg;
  if (!validateTrajectories(trajectories, error_msg)) {
    std::cerr << "[ORCATrajectoryGenerator] Validation failed: "
              << error_msg << std::endl;
    statistics_.success = false;
    statistics_.error_message = error_msg;
    return false;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  statistics_.computation_time =
      std::chrono::duration<double>(end_time - start_time).count();
  statistics_.success = true;

  std::cout << "[ORCATrajectoryGenerator] Successfully generated trajectories in "
            << statistics_.computation_time << " seconds" << std::endl;

  return true;
}

bool ORCATrajectoryGenerator::simulateORCA(
    std::vector<AgentState>& agents,
    const std::vector<std::vector<Waypoint>>& waypoints_per_drone,
    std::vector<DroneTrajectory>& trajectories) {

  int num_agents = agents.size();
  int num_steps = static_cast<int>(config_.total_duration / config_.dt);

  // Track current waypoint for each agent
  std::vector<size_t> current_waypoint_idx(num_agents, 1);  // Start at second waypoint

  std::cout << "[ORCATrajectoryGenerator] Simulating " << num_steps
            << " steps..." << std::endl;

  // Simulation loop
  for (int step = 0; step < num_steps; ++step) {
    double t = step * config_.dt;

    // Compute preferred velocities
    std::vector<Eigen::Vector3d> preferred_velocities(num_agents);
    for (int i = 0; i < num_agents; ++i) {
      preferred_velocities[i] = computePreferredVelocity(
          agents[i],
          waypoints_per_drone[i],
          current_waypoint_idx[i]
      );
    }

    // Compute ORCA velocities
    std::vector<Eigen::Vector3d> safe_velocities =
        orca_solver_->computeVelocities(agents, preferred_velocities);

    // Update agent states and record trajectories
    for (int i = 0; i < num_agents; ++i) {
      agents[i].velocity = safe_velocities[i];
      agents[i].position += agents[i].velocity * config_.dt;

      trajectories[i].positions.push_back(agents[i].position);
      trajectories[i].velocities.push_back(agents[i].velocity);
      trajectories[i].timestamps.push_back(t);
      trajectories[i].yaws.push_back(computeYawFromVelocity(agents[i].velocity));

      // Track statistics
      double speed = agents[i].velocity.norm();
      if (speed > statistics_.max_speed) {
        statistics_.max_speed = speed;
      }
    }

    // Progress update
    if (step % 100 == 0) {
      std::cout << "  Progress: " << (100 * step / num_steps) << "%\r" << std::flush;
    }

    statistics_.total_steps++;
  }

  std::cout << "  Progress: 100%" << std::endl;

  return true;
}

Eigen::Vector3d ORCATrajectoryGenerator::computePreferredVelocity(
    const AgentState& agent,
    const std::vector<Waypoint>& waypoints,
    size_t& current_waypoint_idx) const {

  if (current_waypoint_idx >= waypoints.size()) {
    // Reached all waypoints, hover
    return Eigen::Vector3d::Zero();
  }

  const Waypoint& target = waypoints[current_waypoint_idx];

  // Check if reached
  if (reachedWaypoint(agent.position, target)) {
    current_waypoint_idx++;
    if (current_waypoint_idx >= waypoints.size()) {
      return Eigen::Vector3d::Zero();
    }
  }

  // Compute desired velocity towards target
  Eigen::Vector3d to_target = target.position - agent.position;
  double distance = to_target.norm();

  if (distance < 1e-3) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d direction = to_target / distance;

  // Desired speed (slow down near target)
  double desired_speed = config_.orca_config.preferred_speed;
  if (distance < 2.0) {
    desired_speed = std::min(desired_speed, distance / config_.dt);
  }

  return direction * desired_speed;
}

bool ORCATrajectoryGenerator::reachedWaypoint(
    const Eigen::Vector3d& position,
    const Waypoint& waypoint) const {
  return (position - waypoint.position).norm() < config_.goal_tolerance;
}

void ORCATrajectoryGenerator::smoothTrajectory(DroneTrajectory& trajectory) const {
  if (trajectory.positions.size() < static_cast<size_t>(config_.smoothing_window)) {
    return;
  }

  // Moving average filter
  std::vector<Eigen::Vector3d> smoothed_positions;
  int window = config_.smoothing_window;
  int half_window = window / 2;

  for (size_t i = 0; i < trajectory.positions.size(); ++i) {
    Eigen::Vector3d avg = Eigen::Vector3d::Zero();
    int count = 0;

    for (int j = -half_window; j <= half_window; ++j) {
      int idx = static_cast<int>(i) + j;
      if (idx >= 0 && idx < static_cast<int>(trajectory.positions.size())) {
        avg += trajectory.positions[idx];
        count++;
      }
    }

    smoothed_positions.push_back(avg / count);
  }

  trajectory.positions = smoothed_positions;

  // Recompute velocities
  for (size_t i = 0; i < trajectory.positions.size() - 1; ++i) {
    trajectory.velocities[i] =
        (trajectory.positions[i + 1] - trajectory.positions[i]) / config_.dt;
  }
  if (!trajectory.velocities.empty()) {
    trajectory.velocities.back() = trajectory.velocities[trajectory.velocities.size() - 2];
  }
}

double ORCATrajectoryGenerator::computeYawFromVelocity(
    const Eigen::Vector3d& velocity) const {
  if (velocity.head<2>().norm() < 0.1) {
    return 0.0;  // Not moving horizontally
  }
  return std::atan2(velocity.y(), velocity.x());
}

bool ORCATrajectoryGenerator::validateTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    std::string& error_message) const {

  // Check for collisions
  for (size_t i = 0; i < trajectories.size(); ++i) {
    for (size_t j = i + 1; j < trajectories.size(); ++j) {
      size_t min_size = std::min(trajectories[i].size(), trajectories[j].size());

      for (size_t t = 0; t < min_size; ++t) {
        double dist = (trajectories[i].positions[t] -
                       trajectories[j].positions[t]).norm();

        if (dist < config_.orca_config.max_speed * config_.dt * 2.0) {
          error_message = "Collision detected between drone " +
                         std::to_string(i) + " and " + std::to_string(j) +
                         " at t=" + std::to_string(trajectories[i].timestamps[t]);
          return false;
        }
      }
    }
  }

  return true;
}

bool ORCATrajectoryGenerator::saveTrajectories(
    const std::vector<DroneTrajectory>& trajectories,
    const std::string& filename) const {

  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[ORCATrajectoryGenerator] Failed to open: " << filename << std::endl;
    return false;
  }

  file << "# ORCA Planner Trajectories\n";
  file << "# Drones: " << trajectories.size() << "\n\n";

  for (const auto& traj : trajectories) {
    file << "DRONE " << traj.drone_id << "\n";
    file << "POINTS " << traj.size() << "\n";

    for (size_t i = 0; i < traj.size(); ++i) {
      file << traj.timestamps[i] << " "
           << traj.positions[i].x() << " "
           << traj.positions[i].y() << " "
           << traj.positions[i].z() << " "
           << traj.velocities[i].x() << " "
           << traj.velocities[i].y() << " "
           << traj.velocities[i].z() << " "
           << traj.yaws[i] << "\n";
    }
    file << "\n";
  }

  file.close();
  std::cout << "[ORCATrajectoryGenerator] Saved to: " << filename << std::endl;
  return true;
}

bool ORCATrajectoryGenerator::loadTrajectories(
    const std::string& filename,
    std::vector<DroneTrajectory>& trajectories) const {

  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[ORCATrajectoryGenerator] Failed to open: " << filename << std::endl;
    return false;
  }

  // TODO: Implement loading

  return true;
}

} // namespace orca_planner
