#ifndef HEADWAY_PLANNER_SLOT_ALLOCATOR_HPP
#define HEADWAY_PLANNER_SLOT_ALLOCATOR_HPP

#include "headway_planner/utils/data_types.hpp"
#include <vector>
#include <memory>
#include <map>

namespace headway_planner {

/**
 * @brief Altitude slot allocator
 *
 * Assigns altitude offsets to drones at collision events where
 * time separation alone is insufficient or to reduce makespan.
 *
 * Strategy:
 * 1. Identify events that still have small time gaps
 * 2. Build conflict graph (events overlapping in time)
 * 3. Graph coloring to assign layers (0, +h, -h, +2h, ...)
 * 4. Compute waypoint insertions for vertical maneuvers
 */
class SlotAllocator {
public:
  using Ptr = std::shared_ptr<SlotAllocator>;

  explicit SlotAllocator(const PlannerConfig& config);
  ~SlotAllocator();

  /**
   * @brief Allocate altitude slots
   *
   * @param drones Drone information (slots will be added)
   * @param events Collision events
   * @return Number of slots allocated
   */
  int allocateSlots(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events
  );

  /**
   * @brief Compute required slot height for an event
   *
   * Formula: h_slot >= sqrt(d_min² - (v_rel * Δt)²)
   * If Δt >= d_min/v_rel, returns 0 (time alone is sufficient)
   *
   * @param event Collision event
   * @param actual_delta_t Actual time separation achieved
   * @return Required h_slot (meters), or 0 if not needed
   */
  double computeRequiredSlotHeight(
    const CollisionEvent& event,
    double actual_delta_t
  ) const;

  /**
   * @brief Filter events that need slotting
   *
   * Events where time separation is tight (< safety threshold)
   */
  std::vector<CollisionEvent> filterEventsNeedingSlots(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Compute slot waypoints
   *
   * Determines where to insert climb/descend waypoints
   * based on vertical dynamics constraints.
   *
   * @param drone_id Drone to add slot
   * @param event Event causing the slot
   * @param h_offset Altitude offset (+/- h_slot)
   * @return Slot information (s_start, s_end, h_offset)
   */
  DroneInfo::AltitudeSlot computeSlotWaypoints(
    int drone_id,
    const CollisionEvent& event,
    double h_offset,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Set slot height
   */
  void setSlotHeight(double h) { h_slot_ = h; }

  /**
   * @brief Get statistics
   */
  struct Statistics {
    int events_checked;
    int events_needing_slots;
    int slots_allocated;
    int num_layers_used;

    Statistics() : events_checked(0), events_needing_slots(0),
                   slots_allocated(0), num_layers_used(0) {}
  };

  const Statistics& getStatistics() const { return stats_; }

private:
  PlannerConfig config_;
  double h_slot_;
  Statistics stats_;

  /**
   * @brief Build conflict graph
   *
   * Two events conflict if they overlap in time for the same drones
   * or involve drones that are spatially close.
   *
   * @return Adjacency list representation
   */
  std::map<int, std::vector<int>> buildConflictGraph(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Greedy graph coloring
   *
   * Assign colors (layers) to events such that
   * conflicting events get different colors.
   *
   * @param graph Conflict graph
   * @return Color assignment (event_idx -> color)
   */
  std::vector<int> greedyColoring(
    const std::map<int, std::vector<int>>& graph,
    int num_events
  ) const;

  /**
   * @brief Compute climb distance
   *
   * Distance needed to climb h_slot with constraints v_z_max, a_z_max
   *
   * @return s_pre (meters before event center)
   */
  double computeClimbDistance(double h_offset) const;

  /**
   * @brief Select which drone in an event gets the slot
   *
   * Prefer drone with:
   * - More slack
   * - Longer path (absorbs vertical maneuver better)
   *
   * @return drone_i or drone_j
   */
  int selectDroneForSlot(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones
  ) const;

  /**
   * @brief Check if drone already has slot at this location
   */
  bool hasOverlappingSlot(
    const DroneInfo& drone,
    double s_start,
    double s_end
  ) const;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_SLOT_ALLOCATOR_HPP
