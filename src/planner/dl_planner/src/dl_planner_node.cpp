#include "dl_planner/dl_planner_node.hpp"
#include "dl_planner/neural_network/neural_collision_net.hpp"
#include "dl_planner/trajectory_generator/offline_trajectory_generator.hpp"
#include "dl_planner/collision_checker/swarm_collision_checker.hpp"
#include <chrono>
#include <fstream>

namespace dl_planner {

DLPlannerNode::DLPlannerNode()
  : Node("dl_planner_node"),
    num_drones_(10)
{
  RCLCPP_INFO(this->get_logger(), "DL Planner Node starting...");

  // Load parameters
  loadParameters();

  // Initialize neural network
  NeuralNetConfig net_config;
  net_config.model_path = model_path_;
  net_config.collision_threshold = collision_threshold_;
  neural_net_ = std::make_shared<NeuralCollisionNet>(net_config);

  // Load model if path is provided
  if (!model_path_.empty()) {
    neural_net_->loadModel(model_path_);
  }

  // Create trajectory generator
  TrajectoryConfig traj_config;
  traj_config.dt = dt_;
  traj_config.max_duration = max_duration_;
  traj_config.max_velocity = max_velocity_;
  traj_config.max_acceleration = max_acceleration_;
  traj_config.safety_distance = safety_distance_;

  trajectory_generator_ = std::make_shared<OfflineTrajectoryGenerator>();
  trajectory_generator_->initialize(traj_config);
  trajectory_generator_->setNeuralNetwork(neural_net_);

  // Create collision checker
  collision_checker_ = std::make_shared<SwarmCollisionChecker>();
  collision_checker_->setSafetyDistance(safety_distance_);
  trajectory_generator_->setCollisionChecker(collision_checker_);

  // Create publishers
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/dl_planner/status", 10);

  RCLCPP_INFO(this->get_logger(), "DL Planner Node initialized.");
}

DLPlannerNode::~DLPlannerNode() {
}

bool DLPlannerNode::initialize() {
  RCLCPP_INFO(this->get_logger(), "Initializing planner...");

  // Print neural network info
  std::cout << neural_net_->getModelInfo() << std::endl;

  // Setup mission
  if (!setupMission()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup mission.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Planner initialized successfully.");
  return true;
}

bool DLPlannerNode::generateTrajectories() {
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Starting DL Trajectory Generation");
  RCLCPP_INFO(this->get_logger(), "========================================");

  publishStatus("PLANNING_STARTED");

  auto start_time = std::chrono::high_resolution_clock::now();

  // Generate trajectories
  RCLCPP_INFO(this->get_logger(), "Generating trajectories for %d drones...", num_drones_);

  bool success = trajectory_generator_->generateTrajectories(
    start_positions_, goal_positions_, trajectories_);

  auto end_time = std::chrono::high_resolution_clock::now();
  double computation_time = std::chrono::duration<double>(end_time - start_time).count();

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed.");
    publishStatus("PLANNING_FAILED");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory generation completed in %.3f seconds.",
              computation_time);

  // Validate trajectories
  RCLCPP_INFO(this->get_logger(), "Validating trajectories...");
  bool valid = validateTrajectories();

  if (!valid) {
    RCLCPP_WARN(this->get_logger(), "Some trajectories failed validation.");
  } else {
    RCLCPP_INFO(this->get_logger(), "All trajectories validated successfully.");
  }

  // Print statistics
  printStatistics(computation_time);

  // Save trajectories if output path specified
  if (!output_path_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Saving trajectories to: %s", output_path_.c_str());
    trajectory_generator_->saveTrajectories(trajectories_, output_path_);
  }

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "DL Planner completed successfully!");
  RCLCPP_INFO(this->get_logger(), "========================================");

  publishStatus("PLANNING_COMPLETED");
  return true;
}

void DLPlannerNode::loadParameters() {
  // Declare and get parameters
  this->declare_parameter("num_drones", 10);
  this->declare_parameter("model_path", "");
  this->declare_parameter("output_path", "");

  // Neural network parameters
  this->declare_parameter("collision_threshold", 0.5);

  // Trajectory parameters
  this->declare_parameter("dt", 0.1);
  this->declare_parameter("max_duration", 60.0);
  this->declare_parameter("max_velocity", 3.0);
  this->declare_parameter("max_acceleration", 2.0);
  this->declare_parameter("safety_distance", 2.0);

  // Mission parameters
  this->declare_parameter("mission_type", "point_to_point");

  // Get parameters
  num_drones_ = this->get_parameter("num_drones").as_int();
  model_path_ = this->get_parameter("model_path").as_string();
  output_path_ = this->get_parameter("output_path").as_string();
  collision_threshold_ = this->get_parameter("collision_threshold").as_double();
  dt_ = this->get_parameter("dt").as_double();
  max_duration_ = this->get_parameter("max_duration").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  safety_distance_ = this->get_parameter("safety_distance").as_double();
  mission_type_ = this->get_parameter("mission_type").as_string();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Num drones: %d", num_drones_);
  RCLCPP_INFO(this->get_logger(), "  Mission type: %s", mission_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Model path: %s",
              model_path_.empty() ? "(not specified)" : model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max velocity: %.2f m/s", max_velocity_);
  RCLCPP_INFO(this->get_logger(), "  Safety distance: %.2f m", safety_distance_);
}

bool DLPlannerNode::setupMission() {
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

    RCLCPP_INFO(this->get_logger(), "Setup %zu start and goal positions.",
                start_positions_.size());

  } else {
    RCLCPP_ERROR(this->get_logger(), "Mission type '%s' not yet implemented.",
                 mission_type_.c_str());
    return false;
  }

  return true;
}

bool DLPlannerNode::validateTrajectories() {
  int num_collisions = 0;
  int num_violations = 0;

  for (size_t i = 0; i < trajectories_.size(); ++i) {
    const auto& traj_i = trajectories_[i];

    // Check velocity constraints
    for (const auto& vel : traj_i.velocities) {
      if (vel.norm() > max_velocity_ * 1.1) {  // 10% tolerance
        num_violations++;
        break;
      }
    }

    // Check collisions with other drones
    for (size_t j = i + 1; j < trajectories_.size(); ++j) {
      const auto& traj_j = trajectories_[j];

      std::vector<CollisionInfo> collisions;
      if (collision_checker_->checkTrajectoryCollision(traj_i, traj_j, collisions)) {
        num_collisions += collisions.size();
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Validation results:");
  RCLCPP_INFO(this->get_logger(), "  Velocity violations: %d", num_violations);
  RCLCPP_INFO(this->get_logger(), "  Trajectory collisions: %d", num_collisions);

  return (num_collisions == 0 && num_violations == 0);
}

void DLPlannerNode::printStatistics(double computation_time) {
  RCLCPP_INFO(this->get_logger(), "\n=== DL Planner Statistics ===");
  RCLCPP_INFO(this->get_logger(), "Drones: %d", num_drones_);
  RCLCPP_INFO(this->get_logger(), "Trajectories generated: %zu", trajectories_.size());
  RCLCPP_INFO(this->get_logger(), "Computation time: %.3f s", computation_time);

  if (!trajectories_.empty()) {
    double avg_duration = 0.0;
    for (const auto& traj : trajectories_) {
      avg_duration += traj.total_time;
    }
    avg_duration /= trajectories_.size();

    RCLCPP_INFO(this->get_logger(), "Average trajectory duration: %.2f s", avg_duration);
  }

  RCLCPP_INFO(this->get_logger(), "Neural network:");
  RCLCPP_INFO(this->get_logger(), "  Model loaded: %s",
              neural_net_->isModelLoaded() ? "Yes" : "No (using random weights)");
  RCLCPP_INFO(this->get_logger(), "  Collision threshold: %.2f", collision_threshold_);

  RCLCPP_INFO(this->get_logger(), "==================================\n");
}

void DLPlannerNode::publishStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

} // namespace dl_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<dl_planner::DLPlannerNode>();

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

  rclcpp::shutdown();
  return 0;
}
