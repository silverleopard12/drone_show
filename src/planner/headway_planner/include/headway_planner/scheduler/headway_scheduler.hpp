#ifndef HEADWAY_PLANNER_HEADWAY_SCHEDULER_HPP
#define HEADWAY_PLANNER_HEADWAY_SCHEDULER_HPP

#include "headway_planner/utils/data_types.hpp"
#include "headway_planner/collision_detector/event_extractor.hpp"
#include <vector>
#include <memory>

namespace headway_planner {

/**
 * @brief Base class for headway schedulers
 *
 * Computes departure offsets (phi_i) for each drone to avoid collisions.
 */
class HeadwayScheduler {
public:
  using Ptr = std::shared_ptr<HeadwayScheduler>;

  explicit HeadwayScheduler(const PlannerConfig& config);
  virtual ~HeadwayScheduler();

  /**
   * @brief Solve for headways
   *
   * @param drones Drone information (input/output: phi will be set)
   * @param events Collision events to satisfy
   * @return true if successful
   */
  virtual bool solve(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events
  ) = 0;

  /**
   * @brief Get computation time of last solve
   */
  double getComputationTime() const { return computation_time_; }

  /**
   * @brief Get makespan (maximum end time)
   */
  double getMakespan(const std::vector<DroneInfo>& drones) const;

  /**
   * @brief Verify solution (check all constraints)
   */
  bool verifySolution(
    const std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events,
    std::string& error_message
  ) const;

protected:
  PlannerConfig config_;
  double computation_time_;

  /**
   * @brief Compute arrival time at position s
   */
  double computeArrivalTime(const DroneInfo& drone, double s) const;

  /**
   * @brief Compute slack time for drone
   */
  double computeSlack(const DroneInfo& drone) const;

  /**
   * @brief Check if event constraint is violated
   */
  bool isViolated(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Get actual time separation for an event
   */
  double getActualTimeSeparation(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Update drone slacks
   */
  void updateDroneSlacks(std::vector<DroneInfo>& drones) const;

  /**
   * @brief Get planning result
   */
  PlanningResult getResult(const std::vector<DroneInfo>& drones) const;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_HEADWAY_SCHEDULER_HPP
