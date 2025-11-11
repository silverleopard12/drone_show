#include "headway_planner/collision_detector/event_extractor.hpp"
#include "headway_planner/utils/geometry_utils.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace headway_planner {

EventExtractor::EventExtractor(const PlannerConfig& config)
  : config_(config)
{
}

EventExtractor::~EventExtractor() {
}

std::vector<CollisionEvent> EventExtractor::extractEvents(
    const std::vector<DroneInfo>& drones)
{
  std::vector<CollisionEvent> events;

  // O(n^2) pair-wise collision detection
  for (size_t i = 0; i < drones.size(); ++i) {
    for (size_t j = i + 1; j < drones.size(); ++j) {
      CollisionEvent event;
      if (detectCollision(drones[i], drones[j], event)) {
        events.push_back(event);
      }
    }
  }

  return events;
}

bool EventExtractor::detectCollision(
    const DroneInfo& drone_i,
    const DroneInfo& drone_j,
    CollisionEvent& event)
{
  // Compute minimum distance between line segments
  double s_i, s_j;
  double dist_min = GeometryUtils::lineSegmentMinDistance(
    drone_i.P, drone_i.u_hat, drone_i.L,
    drone_j.P, drone_j.u_hat, drone_j.L,
    s_i, s_j
  );

  // Check if collision event exists
  if (dist_min >= config_.d_gate) {
    return false;  // Too far apart
  }

  // Fill event information
  event.drone_i = drone_i.id;
  event.drone_j = drone_j.id;
  event.s_i = s_i;
  event.s_j = s_j;
  event.dist_min = dist_min;

  // Compute crossing angle
  event.theta = GeometryUtils::crossingAngle(drone_i.u_hat, drone_j.u_hat);

  // Compute relative velocity
  event.v_rel = GeometryUtils::relativeVelocity(
    drone_i.u_hat,
    drone_j.u_hat,
    config_.v_des
  );

  // Handle parallel paths (v_rel ≈ 0)
  if (event.v_rel < 1e-6 || event.theta < config_.parallel_threshold) {
    // Parallel paths - use perpendicular distance
    event.v_rel = config_.v_des;  // Assume worst case
    event.delta_t_req = computeParallelSeparation(drone_i, drone_j);
  } else {
    // Crossing paths - compute required time separation
    event.delta_t_req = computeRequiredTimeSeparation(event);
  }

  return true;
}

double EventExtractor::computeRequiredTimeSeparation(
    const CollisionEvent& event) const
{
  // Formula: Δt_req = α * (d_min + 2*e_pos + v_des*ε_t) / v_rel

  double numerator = config_.d_min + 2.0 * config_.e_pos + config_.v_des * config_.epsilon_t;
  double delta_t = config_.alpha * numerator / event.v_rel;

  return delta_t;
}

double EventExtractor::computeParallelSeparation(
    const DroneInfo& drone_i,
    const DroneInfo& drone_j) const
{
  // For parallel paths, compute time based on overlap length
  // Conservative approach: use maximum of both path lengths

  double max_length = std::max(drone_i.L, drone_j.L);
  double safety_distance = config_.d_min + 2.0 * config_.e_pos;

  // Time to clear the entire length with safety margin
  double delta_t = config_.alpha * (max_length + safety_distance) / config_.v_des;

  return delta_t;
}

void EventExtractor::printEvent(const CollisionEvent& event) const {
  std::cout << "Event: Drone " << event.drone_i << " <-> Drone " << event.drone_j
            << ", s_i=" << event.s_i << ", s_j=" << event.s_j
            << ", d_min=" << event.dist_min << " m"
            << ", theta=" << event.theta << " deg"
            << ", v_rel=" << event.v_rel << " m/s"
            << ", Δt_req=" << event.delta_t_req << " s"
            << std::endl;
}

void EventExtractor::updateEventTimes(
    const std::vector<DroneInfo>& drones,
    std::vector<CollisionEvent>& events)
{
  // Update actual passing times based on current headways
  for (auto& event : events) {
    const auto& drone_i = drones[event.drone_i];
    const auto& drone_j = drones[event.drone_j];

    // Ideal time to reach the event point (without headway)
    double t_i_ideal = event.s_i / config_.v_des;
    double t_j_ideal = event.s_j / config_.v_des;

    // Actual time including headway (phi is the headway)
    event.t_i_actual = drone_i.phi + t_i_ideal;
    event.t_j_actual = drone_j.phi + t_j_ideal;
  }
}

bool EventExtractor::isViolated(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones) const
{
  // Get actual passing times
  auto [t_i, t_j] = getPassingTimes(event, drones);

  // Check if time separation is sufficient
  double time_gap = std::abs(t_i - t_j);

  return time_gap < event.delta_t_req;
}

std::pair<double, double> EventExtractor::getPassingTimes(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones) const
{
  const auto& drone_i = drones[event.drone_i];
  const auto& drone_j = drones[event.drone_j];

  // Time to reach event point = headway (phi) + travel_time
  double t_i = drone_i.phi + event.s_i / config_.v_des;
  double t_j = drone_j.phi + event.s_j / config_.v_des;

  return {t_i, t_j};
}

void EventExtractor::printStatistics(
    const std::vector<CollisionEvent>& events) const
{
  if (events.empty()) {
    std::cout << "No collision events detected." << std::endl;
    return;
  }

  std::cout << "\n=== Collision Event Statistics ===" << std::endl;
  std::cout << "Total events: " << events.size() << std::endl;

  // Compute statistics
  double min_dist = std::numeric_limits<double>::max();
  double max_dist = 0.0;
  double avg_dist = 0.0;
  double min_theta = std::numeric_limits<double>::max();
  double max_theta = 0.0;
  double avg_theta = 0.0;
  double min_delta_t = std::numeric_limits<double>::max();
  double max_delta_t = 0.0;
  double avg_delta_t = 0.0;

  for (const auto& event : events) {
    min_dist = std::min(min_dist, event.dist_min);
    max_dist = std::max(max_dist, event.dist_min);
    avg_dist += event.dist_min;

    min_theta = std::min(min_theta, event.theta);
    max_theta = std::max(max_theta, event.theta);
    avg_theta += event.theta;

    min_delta_t = std::min(min_delta_t, event.delta_t_req);
    max_delta_t = std::max(max_delta_t, event.delta_t_req);
    avg_delta_t += event.delta_t_req;
  }

  avg_dist /= events.size();
  avg_theta /= events.size();
  avg_delta_t /= events.size();

  std::cout << "Distance (m): min=" << min_dist
            << ", max=" << max_dist
            << ", avg=" << avg_dist << std::endl;
  std::cout << "Crossing angle (deg): min=" << min_theta
            << ", max=" << max_theta
            << ", avg=" << avg_theta << std::endl;
  std::cout << "Required Δt (s): min=" << min_delta_t
            << ", max=" << max_delta_t
            << ", avg=" << avg_delta_t << std::endl;
  std::cout << "==================================\n" << std::endl;
}

} // namespace headway_planner
