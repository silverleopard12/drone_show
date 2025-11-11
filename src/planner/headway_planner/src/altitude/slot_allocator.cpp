#include "headway_planner/altitude/slot_allocator.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>

namespace headway_planner {

SlotAllocator::SlotAllocator(const PlannerConfig& config)
  : config_(config),
    h_slot_(config.h_slot)
{
}

SlotAllocator::~SlotAllocator() {
}

int SlotAllocator::allocateSlots(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events)
{
  stats_ = Statistics();

  if (!config_.enable_slotting) {
    std::cout << "[SlotAllocator] Altitude slotting disabled." << std::endl;
    return 0;
  }

  std::cout << "[SlotAllocator] Allocating altitude slots..." << std::endl;

  // Filter events that need slotting
  auto events_needing_slots = filterEventsNeedingSlots(events, drones);

  stats_.events_checked = events.size();
  stats_.events_needing_slots = events_needing_slots.size();

  if (events_needing_slots.empty()) {
    std::cout << "[SlotAllocator] No events need altitude slotting." << std::endl;
    return 0;
  }

  std::cout << "[SlotAllocator] " << events_needing_slots.size()
            << " events need slotting (out of " << events.size() << ")" << std::endl;

  // Build conflict graph
  auto conflict_graph = buildConflictGraph(events_needing_slots, drones);

  // Graph coloring to assign layers
  auto colors = greedyColoring(conflict_graph, events_needing_slots.size());

  // Assign slots based on colors
  int num_slots_allocated = 0;
  std::set<int> layers_used;

  for (size_t idx = 0; idx < events_needing_slots.size(); ++idx) {
    const auto& event = events_needing_slots[idx];
    int layer = colors[idx];
    layers_used.insert(layer);

    // Compute altitude offset
    // Layer 0: no offset
    // Layer 1: +h_slot
    // Layer 2: -h_slot
    // Layer 3: +2*h_slot, etc.
    double h_offset = 0.0;
    if (layer > 0) {
      int sign = (layer % 2 == 1) ? 1 : -1;
      int magnitude = (layer + 1) / 2;
      h_offset = sign * magnitude * h_slot_;
    }

    // Select which drone gets the slot
    int drone_to_slot = selectDroneForSlot(event, drones);

    // Find drone
    DroneInfo* drone = nullptr;
    for (auto& d : drones) {
      if (d.id == drone_to_slot) {
        drone = &d;
        break;
      }
    }

    if (!drone) continue;

    // Check if drone already has overlapping slot
    auto slot = computeSlotWaypoints(drone_to_slot, event, h_offset, drones);

    if (!hasOverlappingSlot(*drone, slot.s_start, slot.s_end)) {
      drone->slots.push_back(slot);
      num_slots_allocated++;
    }
  }

  stats_.slots_allocated = num_slots_allocated;
  stats_.num_layers_used = layers_used.size();

  std::cout << "[SlotAllocator] Allocated " << num_slots_allocated << " slots using "
            << stats_.num_layers_used << " layers." << std::endl;

  return num_slots_allocated;
}

double SlotAllocator::computeRequiredSlotHeight(
    const CollisionEvent& event,
    double actual_delta_t) const
{
  // Formula: h_slot >= sqrt(d_min² - (v_rel * Δt)²)
  // If Δt >= d_min/v_rel, time alone is sufficient (return 0)

  double time_component = event.v_rel * actual_delta_t;

  if (time_component >= config_.d_min) {
    return 0.0;  // Time separation sufficient
  }

  double h_required_sq = config_.d_min * config_.d_min - time_component * time_component;

  if (h_required_sq <= 0.0) {
    return 0.0;
  }

  return std::sqrt(h_required_sq);
}

std::vector<CollisionEvent> SlotAllocator::filterEventsNeedingSlots(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones) const
{
  std::vector<CollisionEvent> filtered;

  for (const auto& event : events) {
    // Find drones
    const DroneInfo* drone_i = nullptr;
    const DroneInfo* drone_j = nullptr;

    for (const auto& drone : drones) {
      if (drone.id == event.drone_i) drone_i = &drone;
      if (drone.id == event.drone_j) drone_j = &drone;
    }

    if (!drone_i || !drone_j) continue;

    // Compute actual time separation
    double t_i = drone_i->phi + event.s_i / config_.v_des;
    double t_j = drone_j->phi + event.s_j / config_.v_des;
    double actual_delta_t = std::abs(t_i - t_j);

    // Check if time gap is tight
    if (actual_delta_t < event.delta_t_req + config_.slot_threshold) {
      filtered.push_back(event);
    }
  }

  return filtered;
}

DroneInfo::AltitudeSlot SlotAllocator::computeSlotWaypoints(
    int drone_id,
    const CollisionEvent& event,
    double h_offset,
    const std::vector<DroneInfo>& drones) const
{
  DroneInfo::AltitudeSlot slot;

  // Find drone
  const DroneInfo* drone = nullptr;
  for (const auto& d : drones) {
    if (d.id == drone_id) {
      drone = &d;
      break;
    }
  }

  if (!drone) {
    return slot;
  }

  // Determine s position for slot
  double s_event = (drone->id == event.drone_i) ? event.s_i : event.s_j;

  // Compute climb distance
  double climb_dist = computeClimbDistance(std::abs(h_offset));

  // Slot boundaries
  slot.s_start = std::max(0.0, s_event - climb_dist);
  slot.s_end = std::min(drone->L, s_event + climb_dist);
  slot.h_offset = h_offset;

  return slot;
}

std::map<int, std::vector<int>> SlotAllocator::buildConflictGraph(
    const std::vector<CollisionEvent>& events,
    const std::vector<DroneInfo>& drones) const
{
  std::map<int, std::vector<int>> graph;

  // Initialize graph
  for (size_t i = 0; i < events.size(); ++i) {
    graph[i] = std::vector<int>();
  }

  // Two events conflict if they overlap in time and involve same/nearby drones
  for (size_t i = 0; i < events.size(); ++i) {
    for (size_t j = i + 1; j < events.size(); ++j) {
      const auto& event_i = events[i];
      const auto& event_j = events[j];

      // Check if events share a drone
      bool share_drone = (event_i.drone_i == event_j.drone_i ||
                         event_i.drone_i == event_j.drone_j ||
                         event_i.drone_j == event_j.drone_i ||
                         event_i.drone_j == event_j.drone_j);

      if (share_drone) {
        // Events conflict - add edge
        graph[i].push_back(j);
        graph[j].push_back(i);
      }
    }
  }

  return graph;
}

std::vector<int> SlotAllocator::greedyColoring(
    const std::map<int, std::vector<int>>& graph,
    int num_events) const
{
  std::vector<int> colors(num_events, -1);

  for (int i = 0; i < num_events; ++i) {
    // Find colors used by neighbors
    std::set<int> neighbor_colors;

    if (graph.find(i) != graph.end()) {
      for (int neighbor : graph.at(i)) {
        if (colors[neighbor] != -1) {
          neighbor_colors.insert(colors[neighbor]);
        }
      }
    }

    // Assign smallest available color
    int color = 0;
    while (neighbor_colors.count(color) > 0) {
      color++;
    }

    colors[i] = color;
  }

  return colors;
}

double SlotAllocator::computeClimbDistance(double h_offset) const {
  // Compute distance needed to climb h_offset with constraints v_z_max, a_z_max
  //
  // Simplified model: constant acceleration then constant deceleration
  // t_accel = v_z_max / a_z_max
  // h_accel = 0.5 * a_z_max * t_accel^2 = v_z_max^2 / (2 * a_z_max)
  //
  // If h_offset < 2 * h_accel: triangular profile
  // If h_offset >= 2 * h_accel: trapezoidal profile
  //
  // For simplicity, use conservative estimate based on horizontal travel:
  // s_climb = h_offset * (v_des / v_z_max)

  if (h_offset < 1e-6) {
    return 0.0;
  }

  double climb_time = h_offset / config_.v_z_max;
  double horizontal_distance = climb_time * config_.v_des;

  // Add margin for acceleration/deceleration
  return horizontal_distance * 1.5;
}

int SlotAllocator::selectDroneForSlot(
    const CollisionEvent& event,
    const std::vector<DroneInfo>& drones) const
{
  // Select drone with more slack or longer path
  const DroneInfo* drone_i = nullptr;
  const DroneInfo* drone_j = nullptr;

  for (const auto& drone : drones) {
    if (drone.id == event.drone_i) drone_i = &drone;
    if (drone.id == event.drone_j) drone_j = &drone;
  }

  if (!drone_i) return event.drone_j;
  if (!drone_j) return event.drone_i;

  // Prefer drone with more slack
  if (drone_i->slack > drone_j->slack) {
    return event.drone_i;
  } else if (drone_j->slack > drone_i->slack) {
    return event.drone_j;
  } else {
    // Equal slack: prefer longer path
    return (drone_i->L > drone_j->L) ? event.drone_i : event.drone_j;
  }
}

bool SlotAllocator::hasOverlappingSlot(
    const DroneInfo& drone,
    double s_start,
    double s_end) const
{
  for (const auto& slot : drone.slots) {
    // Check for overlap
    if (!(s_end < slot.s_start || s_start > slot.s_end)) {
      return true;  // Overlap detected
    }
  }

  return false;
}

} // namespace headway_planner
