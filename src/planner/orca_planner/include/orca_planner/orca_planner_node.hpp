#ifndef ORCA_PLANNER_NODE_HPP
#define ORCA_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

#include "orca_planner/trajectory_generator/orca_trajectory_generator.hpp"
#include "orca_planner/utils/visualization_utils.hpp"

namespace orca_planner {

/**
 * @brief Main ROS2 node for ORCA-based trajectory planning
 *
 * This node generates collision-free trajectories for drone swarms
 * using the ORCA algorithm, suitable for drone show applications.
 */
class ORCAPlannerNode : public rclcpp::Node {
public:
  ORCAPlannerNode();
  ~ORCAPlannerNode();

  /**
   * @brief Initialize the planner
   */
  bool initialize();

  /**
   * @brief Generate trajectories (main function)
   */
  bool generateTrajectories();

private:
  // Core components
  ORCATrajectoryGenerator::Ptr trajectory_generator_;
  VisualizationUtils::Ptr visualizer_;

  // Configuration
  TrajectoryGeneratorConfig config_;
  std::string mission_type_;      // "point_to_point", "formation", "waypoints"
  std::string output_path_;
  int num_drones_;

  // Mission data
  std::vector<Eigen::Vector3d> start_positions_;
  std::vector<Eigen::Vector3d> goal_positions_;
  std::vector<Formation> formations_;
  std::vector<std::vector<Waypoint>> waypoints_per_drone_;
  std::vector<DroneTrajectory> trajectories_;

  // ROS publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Timer for periodic visualization
  rclcpp::TimerBase::SharedPtr vis_timer_;
  size_t current_vis_step_;

  /**
   * @brief Load parameters from ROS parameter server
   */
  void loadParameters();

  /**
   * @brief Load mission configuration from YAML file
   */
  bool loadMissionConfig(const std::string& mission_file);

  /**
   * @brief Setup mission based on type
   */
  bool setupMission();

  /**
   * @brief Publish status message
   */
  void publishStatus(const std::string& status);

  /**
   * @brief Visualization timer callback
   */
  void visualizationTimerCallback();

  /**
   * @brief Print planning statistics
   */
  void printStatistics(const ORCATrajectoryGenerator::Statistics& stats);

  /**
   * @brief Publish trajectories as ROS paths
   */
  void publishTrajectories();
};

} // namespace orca_planner

#endif // ORCA_PLANNER_NODE_HPP
