/**
 * Per-drone ORCA executor node for real-time collision avoidance
 * Follows Ego-Swarm pattern with namespace and topic remapping
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "orca_planner/orca_solver/orca_3d.hpp"

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace orca_planner {

class ORCAExecutorNode : public rclcpp::Node {
public:
  ORCAExecutorNode() : Node("orca_executor") {
    // Parameters
    drone_id_ = this->declare_parameter("drone_id", 0);
    target_x_ = this->declare_parameter("target_x", 10.0);
    target_y_ = this->declare_parameter("target_y", 10.0);
    target_z_ = this->declare_parameter("target_z", 5.0);
    goal_threshold_ = this->declare_parameter("goal_threshold", 0.5);
    max_velocity_ = this->declare_parameter("max_velocity", 2.0);
    control_rate_ = this->declare_parameter("control_rate", 20.0);  // 20 Hz

    // ORCA parameters
    double time_horizon = this->declare_parameter("time_horizon", 3.0);
    double neighbor_dist = this->declare_parameter("neighbor_dist", 5.0);
    int max_neighbors = this->declare_parameter("max_neighbors", 10);

    target_pos_ << target_x_, target_y_, target_z_;
    current_pos_.setZero();
    current_vel_.setZero();
    goal_reached_ = false;
    mission_started_ = false;

    // ORCA configuration
    ORCAConfig config;
    config.time_horizon = time_horizon;
    config.neighbor_dist = neighbor_dist;
    config.max_neighbors = max_neighbors;
    config.max_speed = max_velocity_;
    config.preferred_speed = max_velocity_ * 0.8;
    config.goal_radius = goal_threshold_;
    config.time_step = 1.0 / control_rate_;

    orca_solver_ = std::make_shared<ORCA3D>(config);

    // Publishers
    vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "position_cmd", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "orca_marker", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ORCAExecutorNode::odomCallback, this, std::placeholders::_1));

    // Subscribe to swarm sync
    sync_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/swarm_sync/start", 10,
      std::bind(&ORCAExecutorNode::syncCallback, this, std::placeholders::_1));

    // Subscribe to neighbor states
    neighbor_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/swarm/agent_states", 10,
      std::bind(&ORCAExecutorNode::neighborCallback, this, std::placeholders::_1));

    // Control timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&ORCAExecutorNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "ORCA Executor initialized for drone %d, target: (%.2f, %.2f, %.2f)",
      drone_id_, target_x_, target_y_, target_z_);
  }

private:
  // Node parameters
  int drone_id_;
  double target_x_, target_y_, target_z_;
  double goal_threshold_;
  double max_velocity_;
  double control_rate_;

  // State
  Eigen::Vector3d current_pos_;
  Eigen::Vector3d current_vel_;
  Eigen::Vector3d target_pos_;
  bool goal_reached_;
  bool mission_started_;

  // Neighbor tracking
  struct NeighborInfo {
    int id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    rclcpp::Time last_update;
  };
  std::map<int, NeighborInfo> neighbors_;
  std::mutex neighbors_mutex_;

  // ORCA solver
  std::shared_ptr<ORCA3D> orca_solver_;

  // ROS interface
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sync_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr neighbor_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ << msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z;

    current_vel_ << msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z;

    // Check if goal reached
    double dist_to_goal = (target_pos_ - current_pos_).norm();
    if (dist_to_goal < goal_threshold_ && !goal_reached_) {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Drone %d reached goal! Distance: %.3f m", drone_id_, dist_to_goal);
    }
  }

  void syncCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !mission_started_) {
      mission_started_ = true;
      RCLCPP_INFO(this->get_logger(), "Drone %d: Mission started!", drone_id_);
    }
  }

  void neighborCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Parse neighbor ID from frame_id (format: "drone_X")
    std::string frame_id = msg->header.frame_id;

    // Extract drone ID from frame_id
    size_t pos = frame_id.find("drone_");
    if (pos != std::string::npos) {
      int neighbor_id = std::stoi(frame_id.substr(pos + 6));

      // Don't track ourselves
      if (neighbor_id == drone_id_) return;

      std::lock_guard<std::mutex> lock(neighbors_mutex_);

      NeighborInfo& neighbor = neighbors_[neighbor_id];
      neighbor.id = neighbor_id;
      neighbor.position << msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z;
      neighbor.velocity << msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z;
      neighbor.last_update = this->now();
    }
  }

  void controlLoop() {
    if (!mission_started_ || goal_reached_) {
      // Publish zero velocity
      auto cmd = geometry_msgs::msg::TwistStamped();
      cmd.header.stamp = this->now();
      cmd.header.frame_id = "world";
      vel_cmd_pub_->publish(cmd);
      return;
    }

    // Compute preferred velocity (towards goal)
    Eigen::Vector3d to_goal = target_pos_ - current_pos_;
    double dist_to_goal = to_goal.norm();

    Eigen::Vector3d preferred_vel;
    if (dist_to_goal < 1e-6) {
      preferred_vel.setZero();
    } else {
      double speed = std::min(max_velocity_, dist_to_goal * 2.0);  // Slow down near goal
      preferred_vel = to_goal.normalized() * speed;
    }

    // Build agent states for ORCA
    std::vector<AgentState> agents;

    // Self
    AgentState self;
    self.id = drone_id_;
    self.position = current_pos_;
    self.velocity = current_vel_;
    self.radius = 0.5;
    agents.push_back(self);

    // Neighbors (remove stale entries)
    {
      std::lock_guard<std::mutex> lock(neighbors_mutex_);
      auto now = this->now();

      for (auto it = neighbors_.begin(); it != neighbors_.end(); ) {
        double age = (now - it->second.last_update).seconds();

        if (age > 1.0) {
          // Remove stale neighbor
          it = neighbors_.erase(it);
        } else {
          // Add to agents
          AgentState neighbor;
          neighbor.id = it->second.id;
          neighbor.position = it->second.position;
          neighbor.velocity = it->second.velocity;
          neighbor.radius = 0.5;
          agents.push_back(neighbor);
          ++it;
        }
      }
    }

    // Compute ORCA velocity
    Eigen::Vector3d orca_vel = orca_solver_->computeVelocity(
      drone_id_, agents, preferred_vel);

    // Publish velocity command
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "world";
    cmd.twist.linear.x = orca_vel.x();
    cmd.twist.linear.y = orca_vel.y();
    cmd.twist.linear.z = orca_vel.z();
    vel_cmd_pub_->publish(cmd);

    // Visualize (every 5 control cycles to reduce overhead)
    static int vis_counter = 0;
    if (++vis_counter >= 5) {
      vis_counter = 0;
      visualizeState();
    }
  }

  void visualizeState() {
    // Visualize goal as a sphere
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "orca_goal";
    marker.id = drone_id_;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = target_pos_.x();
    marker.pose.position.y = target_pos_.y();
    marker.pose.position.z = target_pos_.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    if (goal_reached_) {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    marker.color.a = 0.5f;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    marker_pub_->publish(marker);
  }
};

} // namespace orca_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_planner::ORCAExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
