#include "dl_planner/collision_checker/swarm_collision_checker.hpp"
#include "dl_planner/trajectory_generator/offline_trajectory_generator.hpp"
#include <iostream>
#include <cmath>
#include <limits>

namespace dl_planner {

SwarmCollisionChecker::SwarmCollisionChecker()
  : safety_distance_(1.0),
    total_checks_(0),
    collision_count_(0) {
}

SwarmCollisionChecker::~SwarmCollisionChecker() {
}

void SwarmCollisionChecker::setSafetyDistance(double distance) {
  safety_distance_ = distance;
}

bool SwarmCollisionChecker::checkCollision(
    const Eigen::Vector3d& pos1,
    const Eigen::Vector3d& pos2,
    double safety_distance) {

  total_checks_++;

  double dist_threshold = (safety_distance > 0) ? safety_distance : safety_distance_;
  double distance = (pos2 - pos1).norm();

  bool collision = distance < dist_threshold;
  if (collision) {
    collision_count_++;
  }

  return collision;
}

bool SwarmCollisionChecker::checkTrajectoryCollision(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    std::vector<CollisionInfo>& collisions) {

  collisions.clear();

  if (traj1.positions.empty() || traj2.positions.empty()) {
    return false;
  }

  // Check at discrete time steps
  size_t min_size = std::min(traj1.positions.size(), traj2.positions.size());

  for (size_t i = 0; i < min_size; ++i) {
    double distance = (traj1.positions[i] - traj2.positions[i]).norm();

    if (distance < safety_distance_) {
      CollisionInfo info;
      info.has_collision = true;
      info.drone_id_1 = traj1.drone_id;
      info.drone_id_2 = traj2.drone_id;
      info.time = traj1.timestamps[i];
      info.position = (traj1.positions[i] + traj2.positions[i]) / 2.0;
      info.distance = distance;
      collisions.push_back(info);
    }
  }

  return !collisions.empty();
}

bool SwarmCollisionChecker::checkSwarmCollision(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories,
    std::vector<CollisionInfo>& collisions) {

  collisions.clear();

  for (const auto& other_traj : other_trajectories) {
    std::vector<CollisionInfo> pair_collisions;
    if (checkTrajectoryCollision(trajectory, other_traj, pair_collisions)) {
      collisions.insert(collisions.end(), pair_collisions.begin(), pair_collisions.end());
    }
  }

  return !collisions.empty();
}

bool SwarmCollisionChecker::checkAllCollisions(
    const std::vector<DroneTrajectory>& trajectories,
    std::vector<CollisionInfo>& collisions) {

  collisions.clear();

  for (size_t i = 0; i < trajectories.size(); ++i) {
    for (size_t j = i + 1; j < trajectories.size(); ++j) {
      std::vector<CollisionInfo> pair_collisions;
      if (checkTrajectoryCollision(trajectories[i], trajectories[j], pair_collisions)) {
        collisions.insert(collisions.end(), pair_collisions.begin(), pair_collisions.end());
      }
    }
  }

  return !collisions.empty();
}

double SwarmCollisionChecker::getMinimumDistance(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    double& time_at_min_distance) {

  if (traj1.positions.empty() || traj2.positions.empty()) {
    time_at_min_distance = 0.0;
    return std::numeric_limits<double>::max();
  }

  double min_distance = std::numeric_limits<double>::max();
  size_t min_size = std::min(traj1.positions.size(), traj2.positions.size());

  for (size_t i = 0; i < min_size; ++i) {
    double distance = (traj1.positions[i] - traj2.positions[i]).norm();
    if (distance < min_distance) {
      min_distance = distance;
      time_at_min_distance = traj1.timestamps[i];
    }
  }

  return min_distance;
}

double SwarmCollisionChecker::getDistanceAtTime(
    const DroneTrajectory& traj1,
    const DroneTrajectory& traj2,
    double time) {

  Eigen::Vector3d pos1 = interpolatePosition(traj1, time);
  Eigen::Vector3d pos2 = interpolatePosition(traj2, time);

  return (pos2 - pos1).norm();
}

std::vector<std::pair<double, double>> SwarmCollisionChecker::getCollisionFreeSegments(
    const DroneTrajectory& trajectory,
    const std::vector<DroneTrajectory>& other_trajectories) {

  std::vector<std::pair<double, double>> free_segments;

  // TODO: Implement collision-free segment detection
  // This would identify time intervals where trajectory is collision-free

  return free_segments;
}

Eigen::MatrixXd SwarmCollisionChecker::computeCollisionRiskMap(
    const std::vector<DroneTrajectory>& trajectories,
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    double resolution) {

  // TODO: Implement 3D collision risk map
  // This would create a 3D grid showing collision probability at each cell

  int grid_size = 10; // Placeholder
  Eigen::MatrixXd risk_map = Eigen::MatrixXd::Zero(grid_size, grid_size);

  return risk_map;
}

void SwarmCollisionChecker::resetStatistics() {
  total_checks_ = 0;
  collision_count_ = 0;
}

Eigen::Vector3d SwarmCollisionChecker::interpolatePosition(
    const DroneTrajectory& trajectory,
    double time) const {

  if (trajectory.positions.empty()) {
    return Eigen::Vector3d::Zero();
  }

  // Find the two points to interpolate between
  for (size_t i = 0; i < trajectory.timestamps.size() - 1; ++i) {
    if (time >= trajectory.timestamps[i] && time <= trajectory.timestamps[i+1]) {
      double dt = trajectory.timestamps[i+1] - trajectory.timestamps[i];
      double alpha = (time - trajectory.timestamps[i]) / dt;

      return (1.0 - alpha) * trajectory.positions[i] +
             alpha * trajectory.positions[i+1];
    }
  }

  // If time is beyond trajectory, return last position
  return trajectory.positions.back();
}

void SwarmCollisionChecker::findClosestPointOfApproach(
    const Eigen::Vector3d& pos1,
    const Eigen::Vector3d& vel1,
    const Eigen::Vector3d& pos2,
    const Eigen::Vector3d& vel2,
    double& time_to_cpa,
    double& distance_at_cpa) {

  // Relative position and velocity
  Eigen::Vector3d rel_pos = pos2 - pos1;
  Eigen::Vector3d rel_vel = vel2 - vel1;

  // Time to closest point of approach
  double rel_vel_sq = rel_vel.squaredNorm();
  if (rel_vel_sq < 1e-6) {
    // Objects not moving relative to each other
    time_to_cpa = 0.0;
    distance_at_cpa = rel_pos.norm();
    return;
  }

  time_to_cpa = -rel_pos.dot(rel_vel) / rel_vel_sq;

  // Clamp to non-negative time
  if (time_to_cpa < 0.0) {
    time_to_cpa = 0.0;
  }

  // Distance at CPA
  Eigen::Vector3d pos_at_cpa = rel_pos + time_to_cpa * rel_vel;
  distance_at_cpa = pos_at_cpa.norm();
}

} // namespace dl_planner
