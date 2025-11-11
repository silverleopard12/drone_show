#ifndef HEADWAY_PLANNER_GREEDY_SCHEDULER_HPP
#define HEADWAY_PLANNER_GREEDY_SCHEDULER_HPP

#include "headway_planner/scheduler/headway_scheduler.hpp"
#include <queue>

namespace headway_planner {

/**
 * @brief Greedy headway scheduler
 *
 * Algorithm:
 * 1. Sort events by urgency (|t_i_ideal - t_j_ideal|)
 * 2. For each event:
 *    - Check if constraint is violated
 *    - If yes, delay the drone with more slack
 *    - Update all affected events
 * 3. Iterate until all constraints satisfied or max iterations
 *
 * Fast and practical for most cases (< 1 second for 100 drones).
 */
class GreedyScheduler : public HeadwayScheduler {
public:
  explicit GreedyScheduler(const PlannerConfig& config);
  ~GreedyScheduler() override;

  /**
   * @brief Solve using greedy heuristic
   */
  bool solve(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events
  ) override;

  /**
   * @brief Set maximum iterations
   */
  void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }

  /**
   * @brief Get iteration count from last solve
   */
  int getIterationCount() const { return iteration_count_; }

private:
  int max_iterations_;
  int iteration_count_;

  /**
   * @brief Sort events by priority
   */
  std::vector<CollisionEvent> sortEventsByPriority(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Resolve a single violated event
   */
  void resolveEvent(
    const CollisionEvent& event,
    std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Select which drone to delay
   */
  int selectDroneToDelay(
    const CollisionEvent& event,
    const DroneInfo& drone_i,
    const DroneInfo& drone_j
  ) const;

  /**
   * @brief Compute delay amount
   */
  double computeDelayAmount(
    const CollisionEvent& event,
    const DroneInfo& drone_i,
    const DroneInfo& drone_j,
    int drone_to_delay
  ) const;

  /**
   * @brief Apply delay to drone
   */
  void applyDelay(
    DroneInfo& drone,
    double delay
  ) const;

  /**
   * @brief Check if T_max is violated
   */
  bool checkTimeLimitViolation(
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Get planning result
   */
  PlanningResult getResult(const std::vector<DroneInfo>& drones) const;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_GREEDY_SCHEDULER_HPP
