#ifndef HEADWAY_PLANNER_PLAN_GENERATOR_HPP
#define HEADWAY_PLANNER_PLAN_GENERATOR_HPP

#include "headway_planner/utils/data_types.hpp"
#include <string>
#include <memory>

namespace headway_planner {

/**
 * @brief PX4 .plan file generator
 *
 * Converts planned trajectories to PX4 mission format (.plan JSON).
 *
 * Mission structure:
 * 1. TAKEOFF (to initial altitude)
 * 2. LOITER_TIME (wait for headway offset phi)
 * 3. DO_CHANGE_SPEED (set cruise speed)
 * 4. WAYPOINT (start position)
 * 5. WAYPOINT sequence (including altitude slots)
 * 6. WAYPOINT (goal position)
 * 7. LOITER_TIME (optional: hover at goal)
 * 8. LAND (optional)
 */
class PlanGenerator {
public:
  using Ptr = std::shared_ptr<PlanGenerator>;

  explicit PlanGenerator(const PlannerConfig& config);
  ~PlanGenerator();

  /**
   * @brief Generate .plan file for a single drone
   *
   * @param drone Drone information with trajectory
   * @param output_file Output .plan file path
   * @return true if successful
   */
  bool generatePlan(
    const DroneInfo& drone,
    const std::string& output_file
  );

  /**
   * @brief Generate .plan files for all drones
   *
   * @param drones All drone information
   * @param output_dir Output directory
   * @param prefix File prefix (e.g., "drone_")
   * @return Number of files generated
   */
  int generateAllPlans(
    const std::vector<DroneInfo>& drones,
    const std::string& output_dir,
    const std::string& prefix = "drone_"
  );

  /**
   * @brief Set takeoff altitude
   */
  void setTakeoffAltitude(double alt) { takeoff_altitude_ = alt; }

  /**
   * @brief Enable/disable landing at end
   */
  void enableLanding(bool enable) { enable_landing_ = enable; }

  /**
   * @brief Set hover time at goal
   */
  void setGoalHoverTime(double seconds) { goal_hover_time_ = seconds; }

  /**
   * @brief Set waypoint spacing for slots
   *
   * How densely to sample along path (meters)
   */
  void setWaypointSpacing(double spacing) { waypoint_spacing_ = spacing; }

private:
  PlannerConfig config_;
  double takeoff_altitude_;
  bool enable_landing_;
  double goal_hover_time_;
  double waypoint_spacing_;

  /**
   * @brief Build PX4Plan structure
   */
  PX4Plan buildPlan(const DroneInfo& drone);

  /**
   * @brief Add initial sequence (takeoff, loiter for headway)
   */
  void addInitialSequence(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq
  );

  /**
   * @brief Add main waypoint sequence
   *
   * Includes altitude slot waypoints
   */
  void addMainWaypoints(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq
  );

  /**
   * @brief Add final sequence (goal hover, land)
   */
  void addFinalSequence(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq
  );

  /**
   * @brief Create waypoint item
   */
  PX4MissionItem createWaypoint(
    int seq,
    const Eigen::Vector3d& position,
    double param1 = 0.0  // Hold time
  );

  /**
   * @brief Create LOITER_TIME item
   */
  PX4MissionItem createLoiterTime(
    int seq,
    const Eigen::Vector3d& position,
    double duration
  );

  /**
   * @brief Create DO_CHANGE_SPEED item
   */
  PX4MissionItem createChangeSpeed(
    int seq,
    double speed
  );

  /**
   * @brief Create TAKEOFF item
   */
  PX4MissionItem createTakeoff(
    int seq,
    const Eigen::Vector3d& position,
    double altitude
  );

  /**
   * @brief Create LAND item
   */
  PX4MissionItem createLand(
    int seq,
    const Eigen::Vector3d& position
  );

  /**
   * @brief Write .plan JSON file
   */
  bool writeJSON(const PX4Plan& plan, const std::string& filename);

  /**
   * @brief Sample waypoints along path with slots
   *
   * Generates waypoints considering altitude slots
   */
  std::vector<Eigen::Vector3d> sampleWaypoints(const DroneInfo& drone);

  /**
   * @brief Get altitude offset at position s
   */
  double getAltitudeOffsetAtS(const DroneInfo& drone, double s) const;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_PLAN_GENERATOR_HPP
