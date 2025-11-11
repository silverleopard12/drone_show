#include "headway_planner/scheduler/greedy_scheduler.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace headway_planner {

GreedyScheduler::GreedyScheduler(const PlannerConfig& config)
  : HeadwayScheduler(config)
{
}

GreedyScheduler::~GreedyScheduler() {
}

bool GreedyScheduler::solve(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events)
{
  // Initialize all headways to 0
  for (auto& drone : drones) {
    drone.phi = 0.0;
  }

  if (events.empty()) {
    std::cout << "[GreedyScheduler] No collision events to resolve." << std::endl;
    updateDroneSlacks(drones);
    return true;
  }

  std::cout << "[GreedyScheduler] Solving " << events.size() << " events..." << std::endl;

  // Iterative conflict resolution
  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    // Sort events by priority
    auto sorted_events = sortEventsByPriority(events, drones);

    bool all_satisfied = true;
    int num_violations = 0;

    // Check and resolve each event
    for (const auto& event : sorted_events) {
      if (isViolated(event, drones)) {
        all_satisfied = false;
        num_violations++;
        resolveEvent(event, drones);
      }
    }

    // Update slacks after each iteration
    updateDroneSlacks(drones);

    if (config_.verbose && iter % 10 == 0) {
      std::cout << "[GreedyScheduler] Iteration " << iter
                << ": violations=" << num_violations
                << ", makespan=" << getMakespan(drones) << " s"
                << std::endl;
    }

    // Check convergence
    if (all_satisfied) {
      std::cout << "[GreedyScheduler] Converged in " << iter << " iterations."
                << std::endl;
      return true;
    }

    // Check if T_max is exceeded
    if (checkTimeLimitViolation(drones)) {
      std::cerr << "[GreedyScheduler] ERROR: T_max exceeded. No solution found."
                << std::endl;
      return false;
    }
  }

  std::cerr << "[GreedyScheduler] WARNING: Max iterations reached. Solution may not be optimal."
            << std::endl;

  // Check if solution is feasible
  bool feasible = true;
  for (const auto& event : events) {
    if (isViolated(event, drones)) {
      feasible = false;
      break;
    }
  }

  return feasible;
}

std::vector<CollisionEvent> GreedyScheduler::sortEventsByPriority(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones) const
{
  auto sorted = events;

  std::sort(sorted.begin(), sorted.end(),
    [this, &drones](const CollisionEvent& a, const CollisionEvent& b) {
      // Priority: events with larger required time separation first
      // Tie-break: events with smaller minimum distance
      if (std::abs(a.delta_t_req - b.delta_t_req) > config_.tolerance) {
        return a.delta_t_req > b.delta_t_req;
      }
      return a.dist_min < b.dist_min;
    });

  return sorted;
}

void GreedyScheduler::resolveEvent(
    const CollisionEvent& event,
    std::vector<DroneInfo>& drones) const
{
  // Find drones
  DroneInfo* drone_i = nullptr;
  DroneInfo* drone_j = nullptr;

  for (auto& drone : drones) {
    if (drone.id == event.drone_i) drone_i = &drone;
    if (drone.id == event.drone_j) drone_j = &drone;
  }

  if (!drone_i || !drone_j) {
    return;
  }

  // Select which drone to delay
  int drone_to_delay = selectDroneToDelay(event, *drone_i, *drone_j);

  // Compute required delay
  double delay_amount = computeDelayAmount(event, *drone_i, *drone_j, drone_to_delay);

  // Apply delay
  if (drone_to_delay == event.drone_i) {
    drone_i->phi += delay_amount;
  } else {
    drone_j->phi += delay_amount;
  }
}

int GreedyScheduler::selectDroneToDelay(
    const CollisionEvent& event,
    const DroneInfo& drone_i,
    const DroneInfo& drone_j) const
{
  // Delay the drone with more slack
  double slack_i = computeSlack(drone_i);
  double slack_j = computeSlack(drone_j);

  if (slack_i > slack_j) {
    return event.drone_i;
  } else if (slack_j > slack_i) {
    return event.drone_j;
  } else {
    // Equal slack: delay the one arriving first
    double t_i = computeArrivalTime(drone_i, event.s_i);
    double t_j = computeArrivalTime(drone_j, event.s_j);

    return (t_i < t_j) ? event.drone_i : event.drone_j;
  }
}

double GreedyScheduler::computeDelayAmount(
    const CollisionEvent& event,
    const DroneInfo& drone_i,
    const DroneInfo& drone_j,
    int drone_to_delay) const
{
  // Compute current arrival times
  double t_i = computeArrivalTime(drone_i, event.s_i);
  double t_j = computeArrivalTime(drone_j, event.s_j);

  double current_delta_t = std::abs(t_i - t_j);
  double required_delta_t = event.delta_t_req;

  // Compute how much more separation is needed
  double deficit = required_delta_t - current_delta_t;

  if (deficit <= 0.0) {
    return 0.0;  // Already satisfied
  }

  // Add small margin to avoid numerical issues
  return deficit + config_.tolerance;
}

void GreedyScheduler::applyDelay(
    DroneInfo& drone,
    double delay) const
{
  drone.phi += delay;
}

bool GreedyScheduler::checkTimeLimitViolation(
    const std::vector<DroneInfo>& drones) const
{
  for (const auto& drone : drones) {
    double completion_time = drone.phi + drone.L / config_.v_des;
    if (completion_time > config_.T_max) {
      return true;
    }
  }
  return false;
}

PlanningResult GreedyScheduler::getResult(
    const std::vector<DroneInfo>& drones) const
{
  PlanningResult result = HeadwayScheduler::getResult(drones);
  result.scheduler_type = "greedy";
  return result;
}

} // namespace headway_planner
