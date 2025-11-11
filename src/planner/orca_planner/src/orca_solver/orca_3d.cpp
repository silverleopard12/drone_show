#include "orca_planner/orca_solver/orca_3d.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

namespace orca_planner {

ORCA3D::ORCA3D(const ORCAConfig& config)
  : config_(config) {
}

ORCA3D::~ORCA3D() {
}

void ORCA3D::setConfig(const ORCAConfig& config) {
  config_ = config;
}

Eigen::Vector3d ORCA3D::computeVelocity(
    int agent_id,
    const std::vector<AgentState>& agents,
    const Eigen::Vector3d& preferred_velocity) {

  if (agent_id < 0 || agent_id >= static_cast<int>(agents.size())) {
    std::cerr << "[ORCA3D] Invalid agent_id: " << agent_id << std::endl;
    return Eigen::Vector3d::Zero();
  }

  const AgentState& agent = agents[agent_id];

  // Find neighbors
  neighbor_ids_ = findNeighbors(agent_id, agents);

  // Build ORCA planes
  orca_planes_.clear();

  for (int neighbor_id : neighbor_ids_) {
    ORCAPlane plane = computeORCAPlane(agent, agents[neighbor_id]);
    orca_planes_.push_back(plane);
  }

  // Solve linear program
  Eigen::Vector3d new_velocity = linearProgram3D(
      preferred_velocity, orca_planes_);

  return new_velocity;
}

std::vector<Eigen::Vector3d> ORCA3D::computeVelocities(
    const std::vector<AgentState>& agents,
    const std::vector<Eigen::Vector3d>& preferred_velocities) {

  if (agents.size() != preferred_velocities.size()) {
    std::cerr << "[ORCA3D] Size mismatch!" << std::endl;
    return std::vector<Eigen::Vector3d>(agents.size(), Eigen::Vector3d::Zero());
  }

  std::vector<Eigen::Vector3d> new_velocities(agents.size());

  // Compute velocities for all agents (can be parallelized)
  #pragma omp parallel for
  for (size_t i = 0; i < agents.size(); ++i) {
    new_velocities[i] = computeVelocity(i, agents, preferred_velocities[i]);
  }

  return new_velocities;
}

std::vector<int> ORCA3D::findNeighbors(
    int agent_id,
    const std::vector<AgentState>& agents) const {

  std::vector<int> neighbors;
  const AgentState& agent = agents[agent_id];

  // Simple O(n) search - can be optimized with spatial hashing
  for (size_t i = 0; i < agents.size(); ++i) {
    if (static_cast<int>(i) == agent_id) continue;

    double dist = (agents[i].position - agent.position).norm();
    if (dist < config_.neighbor_dist) {
      neighbors.push_back(i);
    }
  }

  // Limit to max_neighbors (keep closest ones)
  if (neighbors.size() > static_cast<size_t>(config_.max_neighbors)) {
    // Sort by distance
    std::sort(neighbors.begin(), neighbors.end(),
      [&agent, &agents](int a, int b) {
        double dist_a = (agents[a].position - agent.position).norm();
        double dist_b = (agents[b].position - agent.position).norm();
        return dist_a < dist_b;
      });
    neighbors.resize(config_.max_neighbors);
  }

  return neighbors;
}

ORCAPlane ORCA3D::computeORCAPlane(
    const AgentState& agent,
    const AgentState& neighbor) const {

  // Relative position and velocity
  Eigen::Vector3d rel_pos = neighbor.position - agent.position;
  Eigen::Vector3d rel_vel = agent.velocity - neighbor.velocity;

  double dist = rel_pos.norm();
  double combined_radius = agent.radius + neighbor.radius;

  // Time to collision
  double tau = computeTimeToCollision(rel_pos, rel_vel, combined_radius);

  ORCAPlane plane;

  if (tau < config_.time_horizon && tau > 0) {
    // Collision is imminent

    // Compute collision avoidance direction
    Eigen::Vector3d w = rel_vel - (rel_pos / tau);

    // Normal points away from velocity obstacle
    double w_norm = w.norm();
    if (w_norm < 1e-6) {
      // Velocities are already collision-free
      plane.normal = rel_pos.normalized();
    } else {
      plane.normal = w / w_norm;
    }

    // ORCA line/plane passes through midpoint
    // Each agent takes 50% responsibility
    Eigen::Vector3d u = (w / 2.0);
    plane.point = agent.velocity + u;

  } else {
    // No immediate collision, but maintain separation

    if (dist < combined_radius + 1e-3) {
      // Agents are penetrating - push apart
      plane.normal = rel_pos.normalized();
      plane.point = agent.velocity;
    } else {
      // Project onto velocity obstacle cone
      double leg_length = std::sqrt(dist * dist - combined_radius * combined_radius);

      Eigen::Vector3d direction = rel_pos / dist;

      // Tangent direction (simplified for 3D)
      Eigen::Vector3d tangent;
      if (std::abs(direction.z()) < 0.9) {
        tangent = direction.cross(Eigen::Vector3d(0, 0, 1)).normalized();
      } else {
        tangent = direction.cross(Eigen::Vector3d(1, 0, 0)).normalized();
      }

      // Compute cutoff center
      double cutoff_dist = (dist - combined_radius) / config_.time_horizon;
      Eigen::Vector3d cutoff_center = agent.velocity - rel_vel +
                                       direction * cutoff_dist;

      // Simplifed plane
      plane.normal = direction;
      plane.point = cutoff_center;
    }
  }

  return plane;
}

Eigen::Vector3d ORCA3D::linearProgram3D(
    const Eigen::Vector3d& preferred_velocity,
    const std::vector<ORCAPlane>& planes) const {

  // Start with preferred velocity
  Eigen::Vector3d velocity = preferred_velocity;

  // Clamp to max speed
  double speed = velocity.norm();
  if (speed > config_.max_speed) {
    velocity = velocity * (config_.max_speed / speed);
  }

  // Iteratively project onto each plane if violated
  for (size_t iteration = 0; iteration < 10; ++iteration) {
    bool all_satisfied = true;

    for (const auto& plane : planes) {
      // Check if velocity violates this plane
      double dist = (velocity - plane.point).dot(plane.normal);

      if (dist < -1e-6) {
        // Violates constraint, project onto plane
        all_satisfied = false;
        velocity = projectOntoPlane(velocity, plane);

        // Re-clamp to max speed
        speed = velocity.norm();
        if (speed > config_.max_speed) {
          velocity = velocity * (config_.max_speed / speed);
        }
      }
    }

    if (all_satisfied) break;
  }

  return velocity;
}

Eigen::Vector3d ORCA3D::projectOntoPlane(
    const Eigen::Vector3d& velocity,
    const ORCAPlane& plane) const {

  // Project velocity onto plane
  // v_new = v + d * n, where d makes (v_new - p) · n = 0

  double dist = (velocity - plane.point).dot(plane.normal);
  return velocity - dist * plane.normal;
}

double ORCA3D::computeTimeToCollision(
    const Eigen::Vector3d& relative_position,
    const Eigen::Vector3d& relative_velocity,
    double combined_radius) const {

  // Solve: ||p + t*v|| = r
  // (p + tv) · (p + tv) = r²
  // t²(v·v) + 2t(p·v) + (p·p - r²) = 0

  double a = relative_velocity.squaredNorm();
  if (a < 1e-6) {
    // No relative motion
    return std::numeric_limits<double>::infinity();
  }

  double b = 2.0 * relative_position.dot(relative_velocity);
  double c = relative_position.squaredNorm() - combined_radius * combined_radius;

  double discriminant = b * b - 4.0 * a * c;

  if (discriminant < 0) {
    // No collision
    return std::numeric_limits<double>::infinity();
  }

  double t = (-b - std::sqrt(discriminant)) / (2.0 * a);

  if (t < 0) {
    // Collision in the past or already colliding
    return 0.0;
  }

  return t;
}

} // namespace orca_planner
