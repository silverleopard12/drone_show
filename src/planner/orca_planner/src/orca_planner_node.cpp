#include "orca_planner/orca_planner_node.hpp"
#include <fstream>
#include <chrono>

namespace orca_planner {

ORCAPlannerNode::ORCAPlannerNode()
  : Node("orca_planner_node"),
    num_drones_(10),
    mission_type_("point_to_point"),
    current_vis_step_(0)
{
  RCLCPP_INFO(this->get_logger(), "ORCA Planner Node starting...");

  // Load parameters
  loadParameters();

  // Create visualizer
  visualizer_ = std::make_shared<VisualizationUtils>(
    this->shared_from_this());
  visualizer_->initialize();

  // Create publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/orca_planner/path", 10);

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/orca_planner/status", 10);

  RCLCPP_INFO(this->get_logger(), "ORCA Planner Node initialized.");
}

ORCAPlannerNode::~ORCAPlannerNode() {
}

bool ORCAPlannerNode::initialize() {
  RCLCPP_INFO(this->get_logger(), "Initializing planner components...");

  // Create trajectory generator
  trajectory_generator_ = std::make_shared<ORCATrajectoryGenerator>(config_);

  // Setup mission
  if (!setupMission()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup mission.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Planner initialized successfully.");
  return true;
}

bool ORCAPlannerNode::generateTrajectories() {
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Starting ORCA Trajectory Generation");
  RCLCPP_INFO(this->get_logger(), "========================================");

  publishStatus("PLANNING_STARTED");

  auto start_time = std::chrono::high_resolution_clock::now();

  // Generate trajectories based on mission type
  bool success = false;

  // Use general trajectory generation
  RCLCPP_INFO(this->get_logger(), "Generating trajectories for mission type: %s", mission_type_.c_str());
  success = trajectory_generator_->generateTrajectories(
    start_positions_, goal_positions_, trajectories_);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed for mission type: %s",
                 mission_type_.c_str());
    publishStatus("PLANNING_FAILED");
    return false;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double computation_time = std::chrono::duration<double>(end_time - start_time).count();

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed.");
    publishStatus("PLANNING_FAILED");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory generation completed in %.3f seconds.",
              computation_time);

  // Get and print statistics
  auto stats = trajectory_generator_->getStatistics();
  stats.computation_time = computation_time;
  printStatistics(stats);

  // Visualize trajectories
  visualizer_->visualizeTrajectories(trajectories_);

  // Publish trajectories
  publishTrajectories();

  // Save trajectories if output path specified
  if (!output_path_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Saving trajectories to: %s", output_path_.c_str());
    trajectory_generator_->saveTrajectories(trajectories_, output_path_);
  }

  // Setup visualization timer for animation
  vis_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ORCAPlannerNode::visualizationTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "ORCA Planner completed successfully!");
  RCLCPP_INFO(this->get_logger(), "========================================");

  publishStatus("PLANNING_COMPLETED");
  return true;
}

void ORCAPlannerNode::loadParameters() {
  // Declare and get parameters
  this->declare_parameter("num_drones", 10);
  this->declare_parameter("mission_type", "point_to_point");
  this->declare_parameter("mission_file", "");
  this->declare_parameter("output_path", "");

  // ORCA parameters
  this->declare_parameter("dt", 0.1);
  this->declare_parameter("total_duration", 60.0);
  this->declare_parameter("max_speed", 3.0);
  this->declare_parameter("max_acceleration", 2.0);
  this->declare_parameter("neighbor_dist", 10.0);
  this->declare_parameter("time_horizon", 2.0);
  this->declare_parameter("time_horizon_obst", 2.0);
  this->declare_parameter("agent_radius", 0.5);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("hover_time", 1.0);

  // Get parameters
  num_drones_ = this->get_parameter("num_drones").as_int();
  mission_type_ = this->get_parameter("mission_type").as_string();
  output_path_ = this->get_parameter("output_path").as_string();

  // Setup config
  config_.dt = this->get_parameter("dt").as_double();
  config_.total_duration = this->get_parameter("total_duration").as_double();
  config_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
  config_.hover_time = this->get_parameter("hover_time").as_double();

  config_.orca_config.max_speed = this->get_parameter("max_speed").as_double();
  config_.orca_config.neighbor_dist = this->get_parameter("neighbor_dist").as_double();
  config_.orca_config.time_horizon = this->get_parameter("time_horizon").as_double();
  config_.orca_config.goal_radius = this->get_parameter("agent_radius").as_double();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Num drones: %d", num_drones_);
  RCLCPP_INFO(this->get_logger(), "  Mission type: %s", mission_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max speed: %.2f m/s", config_.orca_config.max_speed);
  RCLCPP_INFO(this->get_logger(), "  Goal radius: %.2f m", config_.orca_config.goal_radius);
}

bool ORCAPlannerNode::setupMission() {
  RCLCPP_INFO(this->get_logger(), "Setting up mission...");

  if (mission_type_ == "point_to_point") {
    // Simple grid start and goal positions
    int rows = 2;
    int cols = (num_drones_ + 1) / 2;
    double spacing = 3.0;

    start_positions_.clear();
    goal_positions_.clear();

    Eigen::Vector3d start_center(0.0, 0.0, 5.0);
    Eigen::Vector3d goal_center(30.0, 0.0, 5.0);

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (start_positions_.size() >= static_cast<size_t>(num_drones_)) break;

        double offset_x = -(cols - 1) * spacing / 2.0 + c * spacing;
        double offset_y = -(rows - 1) * spacing / 2.0 + r * spacing;

        Eigen::Vector3d start = start_center;
        start.x() += offset_x;
        start.y() += offset_y;
        start_positions_.push_back(start);

        Eigen::Vector3d goal = goal_center;
        goal.x() += offset_x;
        goal.y() += offset_y;
        goal_positions_.push_back(goal);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Setup %zu start and goal positions.", start_positions_.size());

  } else {
    RCLCPP_ERROR(this->get_logger(), "Mission type '%s' not yet implemented.",
                 mission_type_.c_str());
    return false;
  }

  return true;
}

void ORCAPlannerNode::publishStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

void ORCAPlannerNode::visualizationTimerCallback() {
  if (trajectories_.empty()) return;

  // Get current positions for all drones at current time step
  std::vector<AgentState> current_agents;

  for (const auto& traj : trajectories_) {
    if (current_vis_step_ < traj.positions.size()) {
      AgentState agent;
      agent.id = traj.drone_id;
      agent.position = traj.positions[current_vis_step_];
      agent.velocity = (current_vis_step_ < traj.velocities.size()) ?
                       traj.velocities[current_vis_step_] :
                       Eigen::Vector3d::Zero();
      agent.radius = config_.orca_config.goal_radius;
      current_agents.push_back(agent);
    }
  }

  // Visualize current positions
  if (!current_agents.empty()) {
    visualizer_->visualizeDronePositions(current_agents);
  }

  // Increment step
  current_vis_step_++;

  // Reset if we've reached the end
  if (trajectories_.empty() || current_vis_step_ >= trajectories_[0].positions.size()) {
    current_vis_step_ = 0;
  }
}

void ORCAPlannerNode::printStatistics(
    const ORCATrajectoryGenerator::Statistics& stats)
{
  RCLCPP_INFO(this->get_logger(), "\n=== ORCA Planner Statistics ===");
  RCLCPP_INFO(this->get_logger(), "Total steps: %d", stats.total_steps);
  RCLCPP_INFO(this->get_logger(), "Computation time: %.3f s", stats.computation_time);
  RCLCPP_INFO(this->get_logger(), "Collision checks: %d", stats.collision_checks);
  RCLCPP_INFO(this->get_logger(), "Average speed: %.2f m/s", stats.average_speed);
  RCLCPP_INFO(this->get_logger(), "Max speed: %.2f m/s", stats.max_speed);
  RCLCPP_INFO(this->get_logger(), "Success: %s", stats.success ? "YES" : "NO");
  if (!stats.error_message.empty()) {
    RCLCPP_WARN(this->get_logger(), "Error: %s", stats.error_message.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "==================================\n");
}

void ORCAPlannerNode::publishTrajectories() {
  // Publish each trajectory as a ROS Path message
  for (const auto& traj : trajectories_) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = this->now();

    for (size_t i = 0; i < traj.positions.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = traj.positions[i].x();
      pose.pose.position.y = traj.positions[i].y();
      pose.pose.position.z = traj.positions[i].z();
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
  }

  RCLCPP_INFO(this->get_logger(), "Published %zu trajectory paths.", trajectories_.size());
}

} // namespace orca_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<orca_planner::ORCAPlannerNode>();

  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed.");
    rclcpp::shutdown();
    return 1;
  }

  if (!node->generateTrajectories()) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory generation failed.");
    rclcpp::shutdown();
    return 1;
  }

  // Spin to keep node alive for visualization
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
