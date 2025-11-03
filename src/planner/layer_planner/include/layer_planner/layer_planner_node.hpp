#ifndef LAYER_PLANNER_NODE_HPP
#define LAYER_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/msg/multi_bsplines.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>

namespace layer_planner {

struct DroneState {
    int drone_id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d goal;
    Eigen::Vector3d start_position;  // Store initial position when goal is set
    bool has_goal;
    bool has_valid_odom;  // Flag to ensure we received at least one odom message
    bool trajectory_generated;  // Flag to track if trajectory was generated
    traj_utils::msg::Bspline trajectory_msg;  // Store the generated trajectory
    rclcpp::Time last_update;
};

struct LayerConfig {
    double base_height;        // Base height for layer 0
    double layer_spacing;      // Vertical spacing between layers
    double safety_clearance;   // Minimum clearance between drones
    int num_layers;           // Total number of layers available
};

struct TrajectorySegment {
    Eigen::Vector3d start;
    Eigen::Vector3d end;
    double duration;
    rclcpp::Time start_time;
};

class LayerPlannerNode : public rclcpp::Node {
public:
    LayerPlannerNode();
    ~LayerPlannerNode() = default;

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int drone_id);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void planningTimerCallback();

    // Layer assignment
    int assignLayer(int drone_id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
    bool isLayerAvailable(int layer_id, double start_y, double end_y);
    void releaseLayer(int drone_id);

    // Trajectory generation
    traj_utils::msg::Bspline generateLayeredTrajectory(
        int drone_id,
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& goal,
        int layer_id
    );

    std::vector<Eigen::Vector3d> generateWaypoints(
        int drone_id,
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& goal,
        int layer_id
    );

    // Layer offset for collision avoidance
    Eigen::Vector2d getLayerOffset(int drone_id);

    // Scenario file parsing
    bool loadScenarioFiles(const std::string& scenario_dir);
    bool parseScenarioFile(const std::string& file_path, int& drone_id, Eigen::Vector3d& goal);

    // Load targets from launch parameters
    bool loadTargetsFromParameters();

    // Calculate formation boundaries from all goals
    void calculateFormationBoundaries();

    // Utilities
    double calculateDuration(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

    // Collision detection
    void checkCollisions();

    // ROS communication
    std::map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::map<int, rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr> traj_pubs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // State
    std::map<int, DroneState> drone_states_;
    std::map<int, int> drone_to_layer_;  // Maps drone_id to assigned layer
    std::map<int, std::vector<std::pair<double, double>>> layer_occupancy_;  // layer_id -> [(y_start, y_end)]

    // Mission timing
    rclcpp::Time mission_start_time_;
    bool mission_started_;
    int num_drones_reached_goal_;

    // Parameters
    LayerConfig config_;
    int num_drones_;
    double max_vel_;
    double max_acc_;
    double max_jerk_;
    double limit_ratio_;
    double planning_rate_;
    int target_drone_id_;  // For manual goal setting
    std::string scenario_dir_;  // Directory containing scenario files
    double collision_threshold_;  // Distance threshold for collision detection

    // Formation boundaries (for collision-free approach)
    double formation_min_y_;
    double formation_max_y_;
    double formation_buffer_;  // Buffer distance for approach

    // Collision tracking
    std::map<std::pair<int, int>, bool> collision_warned_;  // Track which pairs already warned
};

}  // namespace layer_planner

#endif  // LAYER_PLANNER_NODE_HPP
