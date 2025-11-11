#ifndef DL_PLANNER_NODE_HPP
#define DL_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

#include "dl_planner/trajectory_generator/offline_trajectory_generator.hpp"
#include "dl_planner/neural_network/neural_collision_net.hpp"
#include "dl_planner/collision_checker/swarm_collision_checker.hpp"

namespace dl_planner {

/**
 * @brief Main ROS2 node for DL-based trajectory planning
 *
 * This node provides offline trajectory generation for drone shows
 * using deep learning-based collision avoidance.
 */
class DLPlannerNode : public rclcpp::Node {
public:
  DLPlannerNode();
  ~DLPlannerNode();

  /**
   * @brief Initialize the planner
   */
  bool initialize();

  /**
   * @brief Generate trajectories for all drones
   */
  bool generateTrajectories();

private:
  // Core components
  OfflineTrajectoryGenerator::Ptr trajectory_generator_;
  NeuralCollisionNet::Ptr neural_net_;
  SwarmCollisionChecker::Ptr collision_checker_;

  // Configuration
  std::string model_path_;
  std::string output_path_;
  std::string mission_type_;
  int num_drones_;

  // Trajectory parameters
  double dt_;
  double max_duration_;
  double max_velocity_;
  double max_acceleration_;
  double safety_distance_;
  double collision_threshold_;

  // Mission data
  std::vector<Eigen::Vector3d> start_positions_;
  std::vector<Eigen::Vector3d> goal_positions_;
  std::vector<DroneTrajectory> trajectories_;

  // ROS publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  /**
   * @brief Load parameters from ROS parameter server
   */
  void loadParameters();

  /**
   * @brief Setup mission (start/goal positions)
   */
  bool setupMission();

  /**
   * @brief Publish status message
   */
  void publishStatus(const std::string& status);

  /**
   * @brief Validate all trajectories
   */
  bool validateTrajectories();

  /**
   * @brief Print statistics
   */
  void printStatistics(double computation_time);
};

} // namespace dl_planner

#endif // DL_PLANNER_NODE_HPP
