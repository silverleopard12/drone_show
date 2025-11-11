#include "headway_planner/scheduler/headway_scheduler.hpp"
#include <cmath>

namespace headway_planner {

HeadwayScheduler::HeadwayScheduler(const PlannerConfig& config)
  : config_(config)
{
}

HeadwayScheduler::~HeadwayScheduler() {
}

double HeadwayScheduler::computeArrivalTime(
    const DroneInfo& drone,
    double s) const
{
  // t = phi + s / v_des
  return drone.phi + s / config_.v_des;
}

double HeadwayScheduler::computeSlack(const DroneInfo& drone) const {
  // slack = T_max - (phi + L / v_des)
  double completion_time = drone.phi + drone.L / config_.v_des;
  return config_.T_max - completion_time;
}

double HeadwayScheduler::getMakespan(
    const std::vector<DroneInfo>& drones) const
{
  double makespan = 0.0;

  for (const auto& drone : drones) {
    double completion_time = drone.phi + drone.L / config_.v_des;
    makespan = std::max(makespan, completion_time);
  }

  return makespan;
}

bool HeadwayScheduler::isViolated(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones) const
{
  // Find drones
  const DroneInfo* drone_i = nullptr;
  const DroneInfo* drone_j = nullptr;

  for (const auto& drone : drones) {
    if (drone.id == event.drone_i) drone_i = &drone;
    if (drone.id == event.drone_j) drone_j = &drone;
  }

  if (!drone_i || !drone_j) {
    return false;  // Drone not found
  }

  // Compute arrival times at collision point
  double t_i = computeArrivalTime(*drone_i, event.s_i);
  double t_j = computeArrivalTime(*drone_j, event.s_j);

  // Check if time separation is sufficient
  double actual_delta_t = std::abs(t_i - t_j);

  return actual_delta_t < event.delta_t_req;
}

double HeadwayScheduler::getActualTimeSeparation(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones) const
{
  const DroneInfo* drone_i = nullptr;
  const DroneInfo* drone_j = nullptr;

  for (const auto& drone : drones) {
    if (drone.id == event.drone_i) drone_i = &drone;
    if (drone.id == event.drone_j) drone_j = &drone;
  }

  if (!drone_i || !drone_j) {
    return 0.0;
  }

  double t_i = computeArrivalTime(*drone_i, event.s_i);
  double t_j = computeArrivalTime(*drone_j, event.s_j);

  return std::abs(t_i - t_j);
}

void HeadwayScheduler::updateDroneSlacks(std::vector<DroneInfo>& drones) const {
  for (auto& drone : drones) {
    drone.slack = computeSlack(drone);
  }
}

PlanningResult HeadwayScheduler::getResult(
    const std::vector<DroneInfo>& drones) const
{
  PlanningResult result;
  result.success = true;
  result.makespan = getMakespan(drones);
  result.num_drones = drones.size();
  result.scheduler_type = "base";

  return result;
}

} // namespace headway_planner
