#ifndef HEADWAY_PLANNER_NODE_HPP
#define HEADWAY_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>

#include "headway_planner/utils/data_types.hpp"
#include "headway_planner/collision_detector/event_extractor.hpp"
#include "headway_planner/scheduler/greedy_scheduler.hpp"
#include "headway_planner/scheduler/milp_scheduler.hpp"
#include "headway_planner/altitude/slot_allocator.hpp"
#include "headway_planner/trajectory/plan_generator.hpp"

namespace headway_planner {

/**
 * @brief Main ROS2 node for Headway Planner
 *
 * Complete pipeline:
 * 1. Load drone start/goal positions
 * 2. Extract collision events
 * 3. Solve for headways (Greedy or MILP)
 * 4. Allocate altitude slots (optional)
 * 5. Generate PX4 .plan files
 * 6. Visualize results
 */
class HeadwayPlannerNode : public rclcpp::Node {
public:
  HeadwayPlannerNode();
  ~HeadwayPlannerNode();

  /**
   * @brief Initialize and run planner
   */
  bool initialize();
  bool plan();

private:
  // Configuration
  PlannerConfig config_;
  std::string output_dir_;
  int num_drones_;

  // Core components
  EventExtractor::Ptr event_extractor_;
  HeadwayScheduler::Ptr scheduler_;
  SlotAllocator::Ptr slot_allocator_;
  PlanGenerator::Ptr plan_generator_;

  // Data
  std::vector<DroneInfo> drones_;
  std::vector<CollisionEvent> events_;
  PlanningResult result_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> waypoint_pubs_;

  /**
   * @brief Load parameters from ROS
   */
  void loadParameters();

  /**
   * @brief Load mission (start/goal positions)
   *
   * Can load from:
   * - ROS parameters
   * - YAML file
   * - Formation generator
   */
  bool loadMission();

  /**
   * @brief Load from grid formation
   */
  void loadGridFormation(
    int rows, int cols, double spacing,
    const Eigen::Vector3d& center,
    std::vector<Eigen::Vector3d>& positions
  );

  /**
   * @brief Pipeline step 1: Extract events
   */
  bool extractCollisionEvents();

  /**
   * @brief Pipeline step 2: Solve headways
   */
  bool solveHeadways();

  /**
   * @brief Pipeline step 3: Allocate slots
   */
  bool allocateAltitudeSlots();

  /**
   * @brief Publish waypoints to ROS topics for executor nodes
   */
  void publishWaypoints();

  /**
   * @brief Pipeline step 4: Generate .plan files
   */
  bool generatePlanFiles();

  /**
   * @brief Visualize results
   */
  void visualize();

  /**
   * @brief Print statistics
   */
  void printStatistics();

  /**
   * @brief Publish status message
   */
  void publishStatus(const std::string& status);

  /**
   * @brief Save summary report
   */
  bool saveSummary(const std::string& filename);
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_NODE_HPP
