#ifndef DL_PLANNER_VISUALIZATION_HPP
#define DL_PLANNER_VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace dl_planner {

// Forward declarations
struct DroneTrajectory;
struct CollisionInfo;

/**
 * @brief Visualization helper class for RViz
 */
class Visualization {
public:
  using Ptr = std::shared_ptr<Visualization>;

  explicit Visualization(rclcpp::Node::SharedPtr node);
  ~Visualization();

  /**
   * @brief Initialize publishers
   */
  void initialize();

  /**
   * @brief Visualize a single trajectory
   */
  void visualizeTrajectory(
    const DroneTrajectory& trajectory,
    const std::string& ns = "trajectory",
    const std::vector<double>& color = {0.0, 1.0, 0.0}
  );

  /**
   * @brief Visualize all trajectories
   */
  void visualizeTrajectories(
    const std::vector<DroneTrajectory>& trajectories
  );

  /**
   * @brief Visualize collision points
   */
  void visualizeCollisions(
    const std::vector<CollisionInfo>& collisions
  );

  /**
   * @brief Visualize waypoints
   */
  void visualizeWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints,
    const std::string& ns = "waypoints",
    const std::vector<double>& color = {1.0, 0.0, 0.0}
  );

  /**
   * @brief Visualize drone positions
   */
  void visualizeDronePositions(
    const std::vector<Eigen::Vector3d>& positions,
    const std::string& ns = "drones"
  );

  /**
   * @brief Visualize safety spheres around drones
   */
  void visualizeSafetySpheres(
    const std::vector<Eigen::Vector3d>& positions,
    double radius,
    const std::string& ns = "safety_spheres"
  );

  /**
   * @brief Visualize bounding box
   */
  void visualizeBoundingBox(
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    const std::string& ns = "bounds"
  );

  /**
   * @brief Clear all markers
   */
  void clearAll();

  /**
   * @brief Clear specific namespace
   */
  void clearNamespace(const std::string& ns);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr misc_pub_;

  int marker_id_counter_;

  /**
   * @brief Create line strip marker
   */
  visualization_msgs::msg::Marker createLineStripMarker(
    const std::vector<Eigen::Vector3d>& points,
    const std::string& ns,
    int id,
    const std::vector<double>& color,
    double width = 0.05
  );

  /**
   * @brief Create sphere marker
   */
  visualization_msgs::msg::Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const std::string& ns,
    int id,
    const std::vector<double>& color,
    double radius = 0.1
  );

  /**
   * @brief Create delete all marker
   */
  visualization_msgs::msg::Marker createDeleteAllMarker(const std::string& ns);

  /**
   * @brief Get color for drone ID
   */
  std::vector<double> getColorForDrone(int drone_id);
};

} // namespace dl_planner

#endif // DL_PLANNER_VISUALIZATION_HPP
