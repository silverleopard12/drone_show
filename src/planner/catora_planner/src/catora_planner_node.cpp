#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <memory>
#include <vector>

#include "catora_planner/formation_reshaper.hpp"
#include "catora_planner/msg/trajectory.hpp"
#include "catora_planner/srv/get_assignment.hpp"
#include "catora_planner/srv/get_reshaping_trajectories.hpp"

using namespace catora_planner;

class CatoraPlannerNode : public rclcpp::Node
{
public:
  CatoraPlannerNode() : Node("catora_planner_node")
  {
    // Declare parameters
    this->declare_parameter<double>("max_velocity", 2.0);
    this->declare_parameter<double>("max_acceleration", 2.0);
    this->declare_parameter<double>("trajectory_dt", 0.2);

    // Get parameters
    double max_vel = this->get_parameter("max_velocity").as_double();
    double max_acc = this->get_parameter("max_acceleration").as_double();
    double traj_dt = this->get_parameter("trajectory_dt").as_double();

    // Initialize formation reshaper
    formation_reshaper_ = std::make_unique<FormationReshaper>(max_acc, max_vel, traj_dt);

    // Create services
    assignment_service_ = this->create_service<catora_planner::srv::GetAssignment>(
        "~/get_assignment",
        std::bind(&CatoraPlannerNode::getAssignmentCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    trajectories_service_ = this->create_service<catora_planner::srv::GetReshapingTrajectories>(
        "~/get_reshaping_trajectories",
        std::bind(&CatoraPlannerNode::getReshapingTrajectoriesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "==================================================");
    RCLCPP_INFO(this->get_logger(), "  CAT-ORA Planner Node Initialized");
    RCLCPP_INFO(this->get_logger(), "==================================================");
    RCLCPP_INFO(this->get_logger(), "  Max velocity:     %.2f m/s", max_vel);
    RCLCPP_INFO(this->get_logger(), "  Max acceleration: %.2f m/s^2", max_acc);
    RCLCPP_INFO(this->get_logger(), "  Trajectory dt:    %.2f s", traj_dt);
    RCLCPP_INFO(this->get_logger(), "==================================================");
  }

private:
  void getAssignmentCallback(
      const std::shared_ptr<catora_planner::srv::GetAssignment::Request> request,
      std::shared_ptr<catora_planner::srv::GetAssignment::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[Assignment] Request received: %zu robots",
                request->initial_configurations.size());

    // Validate input
    if (request->initial_configurations.empty() || request->goal_configurations.empty())
    {
      response->success = false;
      response->message = "Empty configurations provided";
      RCLCPP_WARN(this->get_logger(), "[Assignment] %s", response->message.c_str());
      return;
    }

    if (request->initial_configurations.size() != request->goal_configurations.size())
    {
      response->success = false;
      response->message = "Number of initial and goal configurations must match";
      RCLCPP_WARN(this->get_logger(), "[Assignment] %s", response->message.c_str());
      return;
    }

    // Convert to Eigen vectors
    std::vector<Eigen::Vector3d> initial_configs;
    std::vector<Eigen::Vector3d> goal_configs;

    for (const auto& pt : request->initial_configurations)
    {
      initial_configs.emplace_back(pt.x, pt.y, pt.z);
    }

    for (const auto& pt : request->goal_configurations)
    {
      goal_configs.emplace_back(pt.x, pt.y, pt.z);
    }

    // Compute assignment
    try
    {
      auto result = formation_reshaper_->getCatoraAssignment(initial_configs, goal_configs);
      auto assignment = result.first;

      // Convert to response format
      response->mapping.resize(assignment.size());
      for (size_t i = 0; i < assignment.size(); ++i)
      {
        response->mapping[i] = assignment[i].second;
      }

      response->success = true;
      response->message = "Assignment computed successfully";

      RCLCPP_INFO(this->get_logger(), "[Assignment] ✓ Computed for %zu robots (time: %.2f ms)",
                  assignment.size(), result.second[0]);
    }
    catch (const std::exception& e)
    {
      response->success = false;
      response->message = std::string("Error computing assignment: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "[Assignment] ✗ %s", response->message.c_str());
    }
  }

  void getReshapingTrajectoriesCallback(
      const std::shared_ptr<catora_planner::srv::GetReshapingTrajectories::Request> request,
      std::shared_ptr<catora_planner::srv::GetReshapingTrajectories::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[Trajectories] Request received: %zu robots",
                request->initial_configurations.size());

    // Validate input
    if (request->initial_configurations.empty() || request->goal_configurations.empty())
    {
      response->success = false;
      response->message = "Empty configurations provided";
      RCLCPP_WARN(this->get_logger(), "[Trajectories] %s", response->message.c_str());
      return;
    }

    if (request->initial_configurations.size() != request->goal_configurations.size())
    {
      response->success = false;
      response->message = "Number of initial and goal configurations must match";
      RCLCPP_WARN(this->get_logger(), "[Trajectories] %s", response->message.c_str());
      return;
    }

    // Update constraints if provided
    if (request->max_velocity > 0 && request->max_acceleration > 0 && request->trajectory_dt > 0)
    {
      formation_reshaper_->setConstraintsAndDt(request->max_acceleration, request->max_velocity, request->trajectory_dt);
      RCLCPP_INFO(this->get_logger(), "[Trajectories] Updated constraints: vel=%.2f, acc=%.2f, dt=%.2f",
                  request->max_velocity, request->max_acceleration, request->trajectory_dt);
    }

    // Convert to Eigen vectors
    std::vector<Eigen::Vector3d> initial_configs;
    std::vector<Eigen::Vector3d> goal_configs;

    for (const auto& pt : request->initial_configurations)
    {
      initial_configs.emplace_back(pt.x, pt.y, pt.z);
    }

    for (const auto& pt : request->goal_configurations)
    {
      goal_configs.emplace_back(pt.x, pt.y, pt.z);
    }

    // Compute trajectories
    try
    {
      auto trajectories = formation_reshaper_->getReshapingTrajectoriesCatora(initial_configs, goal_configs);

      // Convert to response format
      response->trajectories.resize(trajectories.size());
      for (size_t i = 0; i < trajectories.size(); ++i)
      {
        catora_planner::msg::Trajectory traj_msg;
        traj_msg.waypoints.resize(trajectories[i].size());
        for (size_t j = 0; j < trajectories[i].size(); ++j)
        {
          traj_msg.waypoints[j].x = trajectories[i][j].x();
          traj_msg.waypoints[j].y = trajectories[i][j].y();
          traj_msg.waypoints[j].z = trajectories[i][j].z();
        }
        response->trajectories[i] = traj_msg;
      }

      response->success = true;
      response->message = "Trajectories computed successfully";

      if (!trajectories.empty() && !trajectories[0].empty())
      {
        RCLCPP_INFO(this->get_logger(), "[Trajectories] ✓ Generated %zu trajectories with %zu waypoints each",
                    trajectories.size(), trajectories[0].size());
      }
    }
    catch (const std::exception& e)
    {
      response->success = false;
      response->message = std::string("Error computing trajectories: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "[Trajectories] ✗ %s", response->message.c_str());
    }
  }

  std::unique_ptr<FormationReshaper> formation_reshaper_;
  rclcpp::Service<catora_planner::srv::GetAssignment>::SharedPtr assignment_service_;
  rclcpp::Service<catora_planner::srv::GetReshapingTrajectories>::SharedPtr trajectories_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CatoraPlannerNode>();

  RCLCPP_INFO(node->get_logger(), "Spinning CAT-ORA Planner Node...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
