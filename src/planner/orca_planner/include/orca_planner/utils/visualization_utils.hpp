#ifndef ORCA_PLANNER_VISUALIZATION_UTILS_HPP
#define ORCA_PLANNER_VISUALIZATION_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "orca_planner/trajectory_generator/orca_trajectory_generator.hpp"
#include "orca_planner/orca_solver/orca_3d.hpp"

namespace orca_planner {

/**
 * @brief Visualization helper for ORCA planner
 */
class VisualizationUtils {
public:
  using Ptr = std::shared_ptr<VisualizationUtils>;

  explicit VisualizationUtils(rclcpp::Node::SharedPtr node);
  ~VisualizationUtils();

  /**
   * @brief Initialize publishers
   */
  void initialize();

  /**
   * @brief Visualize all trajectories
   */
  void visualizeTrajectories(
    const std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Visualize single trajectory
   */
  void visualizeTrajectory(
    const DroneTrajectory& trajectory,
    const std::vector<double>& color = {0.0, 1.0, 0.0}
  );

  /**
   * @brief Visualize current drone positions
   */
  void visualizeDronePositions(
    const std::vector<AgentState>& agents
  );

  /**
   * @brief Visualize ORCA planes (for debugging)
   */
  void visualizeORCAPlanes(
    const std::vector<ORCAPlane>& planes,
    const Eigen::Vector3d& agent_position
  );

  /**
   * @brief Visualize formations
   */
  void visualizeFormation(
    const Formation& formation,
    const std::string& ns = "formation"
  );

  /**
   * @brief Visualize waypoints
   */
  void visualizeWaypoints(
    const std::vector<Waypoint>& waypoints,
    const std::string& ns = "waypoints"
  );

  /**
   * @brief Clear all visualizations
   */
  void clearAll();

  /**
   * @brief Set frame ID
   */
  void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string frame_id_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    agent_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    formation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_pub_;

  int marker_id_;

  /**
   * @brief Create trajectory marker (line strip)
   */
  visualization_msgs::msg::Marker createTrajectoryMarker(
    const DroneTrajectory& trajectory,
    const std::vector<double>& color,
    int id
  );

  /**
   * @brief Create sphere marker for agent
   */
  visualization_msgs::msg::Marker createAgentMarker(
    const AgentState& agent,
    int id
  );

  /**
   * @brief Create text marker
   */
  visualization_msgs::msg::Marker createTextMarker(
    const std::string& text,
    const Eigen::Vector3d& position,
    int id
  );

  /**
   * @brief Get color for drone ID
   */
  std::vector<double> getColorForDrone(int drone_id) const;
};

} // namespace orca_planner

#endif // ORCA_PLANNER_VISUALIZATION_UTILS_HPP
