#ifndef HEADWAY_PLANNER_EVENT_EXTRACTOR_HPP
#define HEADWAY_PLANNER_EVENT_EXTRACTOR_HPP

#include "headway_planner/utils/data_types.hpp"
#include "headway_planner/utils/geometry_utils.hpp"
#include <vector>
#include <memory>

namespace headway_planner {

/**
 * @brief Extract collision events from drone paths
 *
 * For each pair of drones, computes:
 * - Minimum distance point
 * - Crossing angle
 * - Relative velocity
 * - Required time separation
 */
class EventExtractor {
public:
  using Ptr = std::shared_ptr<EventExtractor>;

  explicit EventExtractor(const PlannerConfig& config);
  ~EventExtractor();

  /**
   * @brief Extract collision events from drones
   *
   * @param drones Vector of drone information
   * @return Vector of collision events (pairs that need separation)
   */
  std::vector<CollisionEvent> extractEvents(
    const std::vector<DroneInfo>& drones
  );

  /**
   * @brief Update events after headway changes
   *
   * Recomputes actual passing times based on current headways.
   * Used by scheduler to check constraint violations.
   *
   * @param drones Updated drone information (with headways)
   * @param events Events to update
   */
  void updateEventTimes(
    const std::vector<DroneInfo>& drones,
    std::vector<CollisionEvent>& events
  );

  /**
   * @brief Check if an event violates time separation constraint
   *
   * @param event Event to check
   * @param drones Drones with current headways
   * @return true if |t_i - t_j| < delta_t_req
   */
  bool isViolated(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Get actual passing times for an event
   */
  std::pair<double, double> getPassingTimes(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Get configuration
   */
  const PlannerConfig& getConfig() const { return config_; }

  /**
   * @brief Print event information
   */
  void printEvent(const CollisionEvent& event) const;

  /**
   * @brief Print statistics about events
   */
  void printStatistics(const std::vector<CollisionEvent>& events) const;

  /**
   * @brief Get statistics
   */
  struct Statistics {
    int total_pairs;
    int collision_candidates;
    int parallel_events;
    double avg_crossing_angle;
    double avg_relative_velocity;

    Statistics() : total_pairs(0), collision_candidates(0),
                   parallel_events(0), avg_crossing_angle(0.0),
                   avg_relative_velocity(0.0) {}
  };

  const Statistics& getStatistics() const { return stats_; }

private:
  PlannerConfig config_;
  Statistics stats_;

  /**
   * @brief Detect collision between two drones
   */
  bool detectCollision(
    const DroneInfo& drone_i,
    const DroneInfo& drone_j,
    CollisionEvent& event
  );

  /**
   * @brief Compute required time separation for an event
   */
  double computeRequiredTimeSeparation(const CollisionEvent& event) const;

  /**
   * @brief Compute separation for parallel paths
   */
  double computeParallelSeparation(
    const DroneInfo& drone_i,
    const DroneInfo& drone_j
  ) const;

  /**
   * @brief Handle special case: parallel paths
   *
   * When crossing angle is very small, v_rel → 0
   * Use different strategy (spatial separation or large time gap)
   */
  void handleParallelCase(
    CollisionEvent& event,
    const DroneInfo& drone_i,
    const DroneInfo& drone_j
  );
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_EVENT_EXTRACTOR_HPP
