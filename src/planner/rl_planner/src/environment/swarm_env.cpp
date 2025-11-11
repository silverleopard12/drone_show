#include "rl_planner/environment/swarm_env.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace rl_planner {

Eigen::VectorXd DroneObservation::toFeatureVector() const {
  int num_neighbors = neighbor_positions.size();
  bool use_comm = !neighbor_goals.empty();

  int dim = getFeatureDim(num_neighbors, use_comm);
  Eigen::VectorXd features(dim);

  int idx = 0;

  // Ego state (10D)
  features.segment<3>(idx) = position; idx += 3;
  features.segment<3>(idx) = velocity; idx += 3;
  features.segment<3>(idx) = goal; idx += 3;
  features(idx++) = distance_to_goal;

  // Neighbor states (7D per neighbor)
  for (size_t i = 0; i < neighbor_positions.size(); ++i) {
    features.segment<3>(idx) = neighbor_positions[i]; idx += 3;
    features.segment<3>(idx) = neighbor_velocities[i]; idx += 3;
    features(idx++) = neighbor_distances[i];
  }

  // Communication (3D per neighbor)
  if (use_comm) {
    for (size_t i = 0; i < neighbor_goals.size(); ++i) {
      features.segment<3>(idx) = neighbor_goals[i]; idx += 3;
    }
  }

  return features;
}

int DroneObservation::getFeatureDim(int num_neighbors, bool use_communication) {
  int dim = 10;  // Ego: pos(3) + vel(3) + goal(3) + dist_to_goal(1)
  dim += num_neighbors * 7;  // Neighbors: rel_pos(3) + rel_vel(3) + dist(1)
  if (use_communication) {
    dim += num_neighbors * 3;  // Neighbor goals(3)
  }
  return dim;
}

SwarmEnv::SwarmEnv(const EnvConfig& config)
  : config_(config),
    current_step_(0),
    rng_(std::random_device{}())
{
  drone_states_.resize(config_.num_drones);
  for (int i = 0; i < config_.num_drones; ++i) {
    drone_states_[i].id = i;
  }

  std::cout << "[SwarmEnv] Environment created with " << config_.num_drones
            << " drones" << std::endl;
}

SwarmEnv::~SwarmEnv() {
}

std::vector<DroneObservation> SwarmEnv::reset() {
  current_step_ = 0;

  // Reset statistics
  stats_ = Statistics();

  // Initialize drones randomly
  initializeDrones();

  // Get initial observations
  std::vector<DroneObservation> observations;
  observations.reserve(config_.num_drones);
  for (int i = 0; i < config_.num_drones; ++i) {
    observations.push_back(getObservation(i));
  }

  return observations;
}

std::vector<DroneObservation> SwarmEnv::reset(
    const std::vector<Eigen::Vector3d>& start_positions,
    const std::vector<Eigen::Vector3d>& goal_positions) {

  if (start_positions.size() != static_cast<size_t>(config_.num_drones) ||
      goal_positions.size() != static_cast<size_t>(config_.num_drones)) {
    std::cerr << "[SwarmEnv] Invalid start/goal positions size!" << std::endl;
    return reset();
  }

  current_step_ = 0;
  stats_ = Statistics();

  // Set positions
  for (int i = 0; i < config_.num_drones; ++i) {
    drone_states_[i].position = start_positions[i];
    drone_states_[i].goal = goal_positions[i];
    drone_states_[i].velocity = Eigen::Vector3d::Zero();
    drone_states_[i].reached_goal = false;
    drone_states_[i].collided = false;
    drone_states_[i].time_in_air = 0.0;
  }

  // Get initial observations
  std::vector<DroneObservation> observations;
  observations.reserve(config_.num_drones);
  for (int i = 0; i < config_.num_drones; ++i) {
    observations.push_back(getObservation(i));
  }

  return observations;
}

std::vector<StepResult> SwarmEnv::step(const std::vector<DroneAction>& actions) {
  if (actions.size() != static_cast<size_t>(config_.num_drones)) {
    std::cerr << "[SwarmEnv] Invalid actions size!" << std::endl;
    return {};
  }

  current_step_++;

  // Update all drones
  for (int i = 0; i < config_.num_drones; ++i) {
    if (!drone_states_[i].reached_goal && !drone_states_[i].collided) {
      updateDronePhysics(i, actions[i]);
      drone_states_[i].time_in_air += config_.dt;
    }
  }

  // Check collisions and goals
  for (int i = 0; i < config_.num_drones; ++i) {
    if (!drone_states_[i].collided) {
      if (checkCollision(i)) {
        drone_states_[i].collided = true;
        stats_.total_collisions++;
      }
    }

    if (!drone_states_[i].reached_goal) {
      if (checkGoalReached(i)) {
        drone_states_[i].reached_goal = true;
        stats_.drones_reached_goal++;
        stats_.average_time_to_goal += drone_states_[i].time_in_air;
      }
    }
  }

  // Compute average time to goal
  if (stats_.drones_reached_goal > 0) {
    stats_.average_time_to_goal /= stats_.drones_reached_goal;
  }

  // Generate step results
  std::vector<StepResult> results;
  results.reserve(config_.num_drones);

  for (int i = 0; i < config_.num_drones; ++i) {
    StepResult result;
    result.observation = getObservation(i);
    result.reward = computeReward(i, actions[i]);
    result.done = isDone();

    // Additional info
    result.info["collision"] = drone_states_[i].collided ? 1.0 : 0.0;
    result.info["goal_reached"] = drone_states_[i].reached_goal ? 1.0 : 0.0;
    result.info["distance_to_goal"] =
      (drone_states_[i].position - drone_states_[i].goal).norm();

    stats_.episode_reward += result.reward;
    results.push_back(result);
  }

  return results;
}

DroneObservation SwarmEnv::getObservation(int drone_id) const {
  if (drone_id < 0 || drone_id >= config_.num_drones) {
    return DroneObservation();
  }

  const auto& ego = drone_states_[drone_id];
  DroneObservation obs;

  // Ego state
  if (config_.use_relative_coords) {
    obs.position = Eigen::Vector3d::Zero();  // Relative to self
    obs.velocity = ego.velocity;
    obs.goal = ego.goal - ego.position;  // Relative goal
  } else {
    obs.position = ego.position;
    obs.velocity = ego.velocity;
    obs.goal = ego.goal;
  }
  obs.distance_to_goal = (ego.position - ego.goal).norm();

  // Get nearest neighbors
  auto neighbor_ids = getNearestNeighbors(drone_id, config_.num_neighbors);

  for (int neighbor_id : neighbor_ids) {
    const auto& neighbor = drone_states_[neighbor_id];

    // Relative position and velocity
    Eigen::Vector3d rel_pos = neighbor.position - ego.position;
    Eigen::Vector3d rel_vel = neighbor.velocity - ego.velocity;
    double distance = rel_pos.norm();

    obs.neighbor_positions.push_back(rel_pos);
    obs.neighbor_velocities.push_back(rel_vel);
    obs.neighbor_distances.push_back(distance);

    // Communication
    if (config_.use_communication && distance <= config_.communication_range) {
      Eigen::Vector3d rel_goal = neighbor.goal - ego.position;
      obs.neighbor_goals.push_back(rel_goal);
    }
  }

  // Pad if fewer neighbors
  while (obs.neighbor_positions.size() < static_cast<size_t>(config_.num_neighbors)) {
    obs.neighbor_positions.push_back(Eigen::Vector3d::Zero());
    obs.neighbor_velocities.push_back(Eigen::Vector3d::Zero());
    obs.neighbor_distances.push_back(1000.0);  // Large distance
    if (config_.use_communication) {
      obs.neighbor_goals.push_back(Eigen::Vector3d::Zero());
    }
  }

  return obs;
}

bool SwarmEnv::isDone() const {
  // Episode done if:
  // 1. Max steps reached
  // 2. All drones reached goal or collided

  if (current_step_ >= config_.max_steps) {
    return true;
  }

  int active_drones = 0;
  for (const auto& drone : drone_states_) {
    if (!drone.reached_goal && !drone.collided) {
      active_drones++;
    }
  }

  return active_drones == 0;
}

void SwarmEnv::setSeed(int seed) {
  rng_.seed(seed);
}

SwarmEnv::Statistics SwarmEnv::getStatistics() const {
  return stats_;
}

void SwarmEnv::initializeDrones() {
  for (int i = 0; i < config_.num_drones; ++i) {
    // Random start position
    drone_states_[i].position = randomPosition();

    // Random goal position (ensure minimum distance from start)
    Eigen::Vector3d goal;
    do {
      goal = randomPosition();
    } while ((goal - drone_states_[i].position).norm() < 10.0);

    drone_states_[i].goal = goal;
    drone_states_[i].velocity = Eigen::Vector3d::Zero();
    drone_states_[i].reached_goal = false;
    drone_states_[i].collided = false;
    drone_states_[i].time_in_air = 0.0;
  }
}

void SwarmEnv::updateDronePhysics(int drone_id, const DroneAction& action) {
  auto& drone = drone_states_[drone_id];

  // Desired velocity from action
  Eigen::Vector3d desired_vel = action.velocity_command;

  // Clip to max velocity
  if (desired_vel.norm() > config_.max_velocity) {
    desired_vel = desired_vel.normalized() * config_.max_velocity;
  }

  // Compute acceleration needed
  Eigen::Vector3d acc = (desired_vel - drone.velocity) / config_.dt;

  // Clip to max acceleration
  if (acc.norm() > config_.max_acceleration) {
    acc = acc.normalized() * config_.max_acceleration;
  }

  // Update velocity
  drone.velocity += acc * config_.dt;

  // Update position
  drone.position += drone.velocity * config_.dt;

  // Enforce world bounds
  drone.position = drone.position.cwiseMax(config_.world_min);
  drone.position = drone.position.cwiseMin(config_.world_max);

  // Update statistics
  stats_.total_distance_traveled += (drone.velocity * config_.dt).norm();
}

bool SwarmEnv::checkCollision(int drone_id) {
  const auto& ego = drone_states_[drone_id];

  for (int i = 0; i < config_.num_drones; ++i) {
    if (i == drone_id) continue;

    const auto& other = drone_states_[i];
    double distance = (ego.position - other.position).norm();

    if (distance < config_.safety_distance) {
      return true;
    }
  }

  return false;
}

bool SwarmEnv::checkGoalReached(int drone_id) {
  const auto& drone = drone_states_[drone_id];
  double distance = (drone.position - drone.goal).norm();
  return distance < config_.goal_tolerance;
}

double SwarmEnv::computeReward(int drone_id, const DroneAction& action) {
  const auto& drone = drone_states_[drone_id];
  double reward = 0.0;

  // Goal reached reward
  if (drone.reached_goal) {
    return config_.goal_reward;
  }

  // Collision penalty
  if (drone.collided) {
    return config_.collision_penalty;
  }

  // Distance to goal reward (negative to encourage progress)
  double distance_to_goal = (drone.position - drone.goal).norm();
  reward += -distance_to_goal * config_.distance_reward_scale;

  // Smoothness reward (penalize large accelerations)
  Eigen::Vector3d acc = (action.velocity_command - drone.velocity) / config_.dt;
  reward += -acc.squaredNorm() * config_.smoothness_reward_scale;

  // Collision avoidance reward (soft constraint)
  double min_distance = 1000.0;
  for (int i = 0; i < config_.num_drones; ++i) {
    if (i == drone_id) continue;
    double dist = (drone.position - drone_states_[i].position).norm();
    min_distance = std::min(min_distance, dist);
  }

  if (min_distance < config_.safety_distance * 2.0) {
    double safety_reward = -(config_.safety_distance * 2.0 - min_distance);
    reward += safety_reward * config_.collision_reward_scale;
  }

  return reward;
}

std::vector<int> SwarmEnv::getNearestNeighbors(int drone_id, int k) const {
  if (drone_id < 0 || drone_id >= config_.num_drones) {
    return {};
  }

  const auto& ego = drone_states_[drone_id];

  // Compute distances to all other drones
  std::vector<std::pair<double, int>> distances;
  distances.reserve(config_.num_drones - 1);

  for (int i = 0; i < config_.num_drones; ++i) {
    if (i == drone_id) continue;

    double dist = (ego.position - drone_states_[i].position).norm();
    distances.emplace_back(dist, i);
  }

  // Sort by distance
  std::sort(distances.begin(), distances.end());

  // Return k nearest
  std::vector<int> neighbors;
  int num_neighbors = std::min(k, static_cast<int>(distances.size()));
  for (int i = 0; i < num_neighbors; ++i) {
    neighbors.push_back(distances[i].second);
  }

  return neighbors;
}

Eigen::Vector3d SwarmEnv::randomPosition() {
  std::uniform_real_distribution<double> dist_x(config_.world_min.x(), config_.world_max.x());
  std::uniform_real_distribution<double> dist_y(config_.world_min.y(), config_.world_max.y());
  std::uniform_real_distribution<double> dist_z(config_.world_min.z(), config_.world_max.z());

  return Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
}

bool SwarmEnv::isValidPosition(const Eigen::Vector3d& pos) const {
  return (pos.array() >= config_.world_min.array()).all() &&
         (pos.array() <= config_.world_max.array()).all();
}

} // namespace rl_planner
