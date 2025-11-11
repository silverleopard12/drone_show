#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

// Position command message (assuming quadrotor_msgs is available)
// If not, we'll use geometry_msgs/TwistStamped as fallback
namespace quadrotor_msgs {
namespace msg {
struct PositionCommand {
  std_msgs::msg::Header header;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Vector3 velocity;
  geometry_msgs::msg::Vector3 acceleration;
  double yaw;
  double yaw_dot;
};
}
}

class HeadwayExecutorNode : public rclcpp::Node {
public:
  HeadwayExecutorNode() : Node("headway_executor") {
    // Parameters
    drone_id_ = this->declare_parameter("drone_id", 0);
    waypoint_threshold_ = this->declare_parameter("waypoint_threshold", 0.5);
    max_velocity_ = this->declare_parameter("max_velocity", 2.0);

    // Publishers - using TwistStamped as position command
    vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "position_cmd", 10);

    // Subscribers
    // Use transient_local QoS to match publisher
    rclcpp::QoS qos(10);
    qos.transient_local();

    waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/drone_" + std::to_string(drone_id_) + "/headway/waypoints", qos,
      std::bind(&HeadwayExecutorNode::waypointCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&HeadwayExecutorNode::odomCallback, this, std::placeholders::_1));

    sync_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/swarm_sync/start", 10,
      std::bind(&HeadwayExecutorNode::syncCallback, this, std::placeholders::_1));

    // Timer for control loop (50 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&HeadwayExecutorNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Headway Executor for drone_%d initialized", drone_id_);
    RCLCPP_INFO(this->get_logger(), "Waiting for waypoints on topic: /drone_%d/headway/waypoints", drone_id_);
  }

private:
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    waypoints_.clear();

    for (const auto& pose : msg->poses) {
      Eigen::Vector3d wp(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z
      );
      waypoints_.push_back(wp);
    }

    current_waypoint_idx_ = 0;
    waypoints_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints_.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
    );

    current_vel_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z
    );

    odom_received_ = true;
  }

  void syncCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !mission_started_) {
      mission_started_ = true;
      start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Mission started!");
    }
  }

  void controlLoop() {
    // Preparation check
    if (!waypoints_received_ || !odom_received_) {
      return;
    }

    // If not started, hover at current position
    if (!mission_started_) {
      publishHoverCommand();
      return;
    }

    // Mission completion check
    if (current_waypoint_idx_ >= waypoints_.size()) {
      if (!mission_completed_) {
        auto elapsed = (this->now() - start_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "Mission completed in %.2f seconds!", elapsed);
        mission_completed_ = true;
      }

      // Hover at last waypoint
      publishHoverCommand();
      return;
    }

    // Current target waypoint
    const Eigen::Vector3d& target_pos = waypoints_[current_waypoint_idx_];

    // Calculate distance to target
    double dist = (target_pos - current_pos_).norm();

    // Check if waypoint reached
    if (dist < waypoint_threshold_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu (dist: %.2fm)",
                  current_waypoint_idx_ + 1, waypoints_.size(), dist);
      current_waypoint_idx_++;
      return;
    }

    // Generate velocity command
    Eigen::Vector3d direction = (target_pos - current_pos_).normalized();

    // Decelerate when approaching waypoint
    double speed = std::min(max_velocity_, dist * 2.0);
    speed = std::max(speed, 0.2);  // Minimum speed

    Eigen::Vector3d desired_vel = direction * speed;

    // Publish velocity command
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "world";

    cmd.twist.linear.x = desired_vel.x();
    cmd.twist.linear.y = desired_vel.y();
    cmd.twist.linear.z = desired_vel.z();

    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;

    vel_cmd_pub_->publish(cmd);
  }

  void publishHoverCommand() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "world";

    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;

    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;

    vel_cmd_pub_->publish(cmd);
  }

private:
  int drone_id_;
  double waypoint_threshold_;
  double max_velocity_;

  std::vector<Eigen::Vector3d> waypoints_;
  size_t current_waypoint_idx_ = 0;

  Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_vel_ = Eigen::Vector3d::Zero();

  bool waypoints_received_ = false;
  bool odom_received_ = false;
  bool mission_started_ = false;
  bool mission_completed_ = false;

  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sync_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadwayExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
