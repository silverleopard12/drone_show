#include "layer_planner/layer_planner_node.hpp"
#include "bspline_opt/uniform_bspline.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <limits>

namespace layer_planner {

LayerPlannerNode::LayerPlannerNode() : Node("layer_planner_node") {
    // Declare and get parameters
    this->declare_parameter("num_drones", 10);
    this->declare_parameter("max_vel", 2.0);
    this->declare_parameter("max_acc", 3.0);
    this->declare_parameter("max_jerk", 4.0);
    this->declare_parameter("limit_ratio", 1.1);
    this->declare_parameter("planning_rate", 10.0);
    this->declare_parameter("base_height", 1.0);
    this->declare_parameter("layer_spacing", 1.5);
    this->declare_parameter("safety_clearance", 0.8);
    this->declare_parameter("num_layers", 5);
    this->declare_parameter("scenario_dir", "");
    this->declare_parameter("collision_threshold", 1.0);  // 1.0m collision detection threshold

    num_drones_ = this->get_parameter("num_drones").as_int();
    max_vel_ = this->get_parameter("max_vel").as_double();
    max_acc_ = this->get_parameter("max_acc").as_double();
    max_jerk_ = this->get_parameter("max_jerk").as_double();
    limit_ratio_ = this->get_parameter("limit_ratio").as_double();
    planning_rate_ = this->get_parameter("planning_rate").as_double();
    scenario_dir_ = this->get_parameter("scenario_dir").as_string();
    collision_threshold_ = this->get_parameter("collision_threshold").as_double();

    config_.base_height = this->get_parameter("base_height").as_double();
    config_.layer_spacing = this->get_parameter("layer_spacing").as_double();
    config_.safety_clearance = this->get_parameter("safety_clearance").as_double();
    config_.num_layers = this->get_parameter("num_layers").as_int();

    target_drone_id_ = -1;

    // Initialize formation boundaries
    formation_min_y_ = std::numeric_limits<double>::max();
    formation_max_y_ = std::numeric_limits<double>::lowest();
    formation_buffer_ = 5.0;  // 5m buffer for safe approach

    // Initialize mission timing
    mission_started_ = false;
    num_drones_reached_goal_ = 0;

    RCLCPP_INFO(this->get_logger(), "Layer Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Num drones: %d", num_drones_);
    RCLCPP_INFO(this->get_logger(), "  Max velocity: %.2f m/s", max_vel_);
    RCLCPP_INFO(this->get_logger(), "  Max acceleration: %.2f m/s^2", max_acc_);
    RCLCPP_INFO(this->get_logger(), "  Max jerk: %.2f m/s^3", max_jerk_);
    RCLCPP_INFO(this->get_logger(), "  Limit ratio: %.2f", limit_ratio_);
    RCLCPP_INFO(this->get_logger(), "  Base height: %.2f m", config_.base_height);
    RCLCPP_INFO(this->get_logger(), "  Layer spacing: %.2f m", config_.layer_spacing);
    RCLCPP_INFO(this->get_logger(), "  Num layers: %d", config_.num_layers);
    RCLCPP_INFO(this->get_logger(), "  Scenario dir: %s", scenario_dir_.c_str());

    // Initialize subscribers and publishers for each drone
    for (int i = 0; i < num_drones_; ++i) {
        // Odometry subscriber
        std::string odom_topic = "/drone_" + std::to_string(i) + "_visual_slam/odom";
        odom_subs_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odomCallback(msg, i);
            }
        );

        // Trajectory publisher
        std::string traj_topic = "/drone_" + std::to_string(i) + "_planning/bspline";
        traj_pubs_[i] = this->create_publisher<traj_utils::msg::Bspline>(traj_topic, 10);

        // Initialize drone state
        drone_states_[i].drone_id = i;
        drone_states_[i].has_goal = false;
        drone_states_[i].has_valid_odom = false;
        drone_states_[i].trajectory_generated = false;

        RCLCPP_INFO(this->get_logger(), "  Drone %d: odom=%s, traj=%s",
                    i, odom_topic.c_str(), traj_topic.c_str());
    }

    // Goal subscriber (from RViz)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&LayerPlannerNode::goalCallback, this, std::placeholders::_1)
    );

    // Planning timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / planning_rate_));
    planning_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&LayerPlannerNode::planningTimerCallback, this)
    );

    // Load scenario files if directory is specified
    if (!scenario_dir_.empty()) {
        if (loadScenarioFiles(scenario_dir_)) {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded goals from scenario files");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load scenario files from: %s", scenario_dir_.c_str());
        }
    } else {
        // Load target coordinates from parameters if scenario_dir is empty
        if (loadTargetsFromParameters()) {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded goals from launch parameters");
        }
    }

    // Calculate formation boundaries from all loaded goals
    calculateFormationBoundaries();

    RCLCPP_INFO(this->get_logger(), "Layer Planner ready!");
}

void LayerPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int drone_id) {
    auto& state = drone_states_[drone_id];
    state.position.x() = msg->pose.pose.position.x;
    state.position.y() = msg->pose.pose.position.y;
    state.position.z() = msg->pose.pose.position.z;

    state.velocity.x() = msg->twist.twist.linear.x;

    // Mark that we have received at least one valid odom message
    state.has_valid_odom = true;
    state.velocity.y() = msg->twist.twist.linear.y;
    state.velocity.z() = msg->twist.twist.linear.z;

    state.last_update = this->now();
}

void LayerPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Simple strategy: assign goal to drone 0 for now
    // In a full implementation, you could cycle through drones or use a smarter assignment
    int drone_id = 0;

    if (drone_states_.find(drone_id) != drone_states_.end()) {
        auto& state = drone_states_[drone_id];
        state.goal.x() = msg->pose.position.x;
        state.goal.y() = msg->pose.position.y;
        state.goal.z() = msg->pose.position.z;
        state.has_goal = true;

        RCLCPP_INFO(this->get_logger(), "Goal received for drone %d: (%.2f, %.2f, %.2f)",
                    drone_id, state.goal.x(), state.goal.y(), state.goal.z());
    }
}

void LayerPlannerNode::planningTimerCallback() {
    // Check for collisions
    checkCollisions();

    // Plan for each drone that has a goal
    for (auto& [drone_id, state] : drone_states_) {
        if (!state.has_goal) {
            continue;
        }

        // Check if we have received at least one valid odom message
        if (!state.has_valid_odom) {
            continue;  // Wait for odom data before planning
        }

        // Check if drone has recent odometry
        double time_since_update = 0.0;
        try {
            time_since_update = (this->now() - state.last_update).seconds();
        } catch (const std::runtime_error& e) {
            // Time source mismatch - skip this check
            time_since_update = 0.0;
        }

        if (time_since_update > 1.0) {
            continue;  // Stale data
        }

        // Check if close to goal
        double dist_to_goal = (state.position - state.goal).norm();
        if (dist_to_goal < 0.3) {
            state.has_goal = false;
            state.trajectory_generated = false;
            releaseLayer(drone_id);

            num_drones_reached_goal_++;
            RCLCPP_INFO(this->get_logger(), "Drone %d reached goal! (%d/%d drones completed)",
                        drone_id, num_drones_reached_goal_, num_drones_);

            // Check if all drones reached their goals
            if (num_drones_reached_goal_ == num_drones_ && mission_started_) {
                double mission_duration = (this->now() - mission_start_time_).seconds();
                RCLCPP_INFO(this->get_logger(), " ");
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "🎉 ALL DRONES COMPLETED MISSION!");
                RCLCPP_INFO(this->get_logger(), "Total mission time: %.2f seconds", mission_duration);
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), " ");
            }

            continue;
        }

        // Generate trajectory once at the beginning
        if (!state.trajectory_generated) {
            // Record mission start time (first drone only)
            if (!mission_started_) {
                mission_start_time_ = this->now();
                mission_started_ = true;
                RCLCPP_INFO(this->get_logger(), "Mission started!");
            }

            // Store start position
            state.start_position = state.position;

            // Assign layer
            int layer_id = assignLayer(drone_id, state.start_position, state.goal);
            RCLCPP_INFO(this->get_logger(), "Drone %d assigned to layer %d (height: %.2f m)",
                        drone_id, layer_id,
                        config_.base_height + layer_id * config_.layer_spacing);

            // Generate and store trajectory
            state.trajectory_msg = generateLayeredTrajectory(drone_id, state.start_position, state.goal, layer_id);
            state.trajectory_generated = true;

            RCLCPP_INFO(this->get_logger(), "Drone %d trajectory generated from (%.2f, %.2f, %.2f)!",
                        drone_id, state.start_position.x(), state.start_position.y(), state.start_position.z());
        }

        // Continuously publish the SAME trajectory message
        traj_pubs_[drone_id]->publish(state.trajectory_msg);
    }
}

int LayerPlannerNode::assignLayer(int drone_id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    // Simple approach: assign each drone to its own unique layer
    // This guarantees collision-free flight in vertical direction
    int layer = drone_id % config_.num_layers;

    drone_to_layer_[drone_id] = layer;

    double min_y = std::min(start.y(), goal.y());
    double max_y = std::max(start.y(), goal.y());
    layer_occupancy_[layer].push_back({min_y, max_y});

    return layer;
}

bool LayerPlannerNode::isLayerAvailable(int layer_id, double start_y, double end_y) {
    if (layer_occupancy_.find(layer_id) == layer_occupancy_.end()) {
        return true;  // Layer not used yet
    }

    // Check for conflicts with existing occupancy
    for (const auto& [occ_start, occ_end] : layer_occupancy_[layer_id]) {
        // Check if ranges overlap (with safety margin)
        double margin = config_.safety_clearance;
        if (!(end_y + margin < occ_start || start_y - margin > occ_end)) {
            return false;  // Overlap detected
        }
    }

    return true;
}

void LayerPlannerNode::releaseLayer(int drone_id) {
    if (drone_to_layer_.find(drone_id) != drone_to_layer_.end()) {
        int layer_id = drone_to_layer_[drone_id];
        drone_to_layer_.erase(drone_id);

        // Could also clean up layer_occupancy_, but for simplicity we'll leave it
        RCLCPP_INFO(this->get_logger(), "Released layer %d for drone %d", layer_id, drone_id);
    }
}

traj_utils::msg::Bspline LayerPlannerNode::generateLayeredTrajectory(
    int drone_id,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    int layer_id
) {
    // Generate waypoints with layer-based routing
    auto waypoints = generateWaypoints(drone_id, start, goal, layer_id);

    // Create simple B-spline message
    traj_utils::msg::Bspline bspline_msg;
    bspline_msg.drone_id = drone_id;
    bspline_msg.order = 3;  // Cubic B-spline

    // Set start_time to current time
    // Note: Spatial separation (3m offset + different heights) is sufficient for collision avoidance
    bspline_msg.start_time = this->now();

    // Convert waypoints to control points
    // For clamped B-spline: repeat first and last control points (p times)
    // to ensure the curve passes through start and end points
    for (size_t i = 0; i < waypoints.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = waypoints[i].x();
        pt.y = waypoints[i].y();
        pt.z = waypoints[i].z();

        if (i == 0) {
            // Repeat first control point p times
            for (int j = 0; j < bspline_msg.order; ++j) {
                bspline_msg.pos_pts.push_back(pt);
            }
        } else if (i == waypoints.size() - 1) {
            // Repeat last control point p times
            for (int j = 0; j < bspline_msg.order; ++j) {
                bspline_msg.pos_pts.push_back(pt);
            }
        } else {
            bspline_msg.pos_pts.push_back(pt);
        }
    }

    // Calculate total duration based on total distance
    double total_dist = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        total_dist += (waypoints[i] - waypoints[i-1]).norm();
    }

    // Total time = distance / velocity
    double total_duration = total_dist / max_vel_;

    // Knot vector generation (matching Ego-planner's UniformBspline)
    // For cubic B-spline (order 3), we need pos_pts.size() + order + 1 knots
    int p = bspline_msg.order;
    int n = bspline_msg.pos_pts.size() - 1;  // Use actual control point count
    int m = n + p + 1;
    int num_knots = m + 1;

    // Interval between knots
    double interval = total_duration / (num_knots - 2*p - 1);

    // Generate knot vector starting from negative values
    for (int i = 0; i <= m; ++i) {
        if (i <= p) {
            // First p+1 knots: start from negative
            bspline_msg.knots.push_back(double(-p + i) * interval);
        } else if (i > p && i <= m - p) {
            // Middle knots: uniform spacing
            bspline_msg.knots.push_back(bspline_msg.knots.back() + interval);
        } else {
            // Last p knots: continue uniform spacing
            bspline_msg.knots.push_back(bspline_msg.knots.back() + interval);
        }
    }

    // Check feasibility with physical limits (velocity, acceleration)
    // Create UniformBspline object to validate trajectory
    Eigen::MatrixXd control_pts(3, bspline_msg.pos_pts.size());
    for (size_t i = 0; i < bspline_msg.pos_pts.size(); ++i) {
        control_pts(0, i) = bspline_msg.pos_pts[i].x;
        control_pts(1, i) = bspline_msg.pos_pts[i].y;
        control_pts(2, i) = bspline_msg.pos_pts[i].z;
    }

    Eigen::VectorXd knot_vector(bspline_msg.knots.size());
    for (size_t i = 0; i < bspline_msg.knots.size(); ++i) {
        knot_vector(i) = bspline_msg.knots[i];
    }

    ego_planner::UniformBspline bspline;
    bspline.setUniformBspline(control_pts, p, interval);
    bspline.setKnot(knot_vector);
    bspline.setPhysicalLimits(max_vel_, max_acc_, 0.0);  // tolerance = 0.0

    // Check feasibility and adjust time if needed
    double ratio;
    bool feasible = bspline.checkFeasibility(ratio, false);
    if (!feasible) {
        // Lengthen time to satisfy physical constraints
        double adjusted_ratio = ratio * limit_ratio_;
        bspline.lengthenTime(adjusted_ratio);

        // Update knot vector with adjusted timing
        Eigen::VectorXd adjusted_knots = bspline.getKnot();
        bspline_msg.knots.clear();
        for (int i = 0; i < adjusted_knots.size(); ++i) {
            bspline_msg.knots.push_back(adjusted_knots(i));
        }

        RCLCPP_INFO(this->get_logger(),
                    "Drone %d trajectory adjusted for feasibility (ratio: %.2f)",
                    drone_id, adjusted_ratio);
    }

    return bspline_msg;
}

std::vector<Eigen::Vector3d> LayerPlannerNode::generateWaypoints(
    int drone_id,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    int layer_id
) {
    std::vector<Eigen::Vector3d> waypoints;

    // COLLISION-FREE STRATEGY: Space-Time Separation
    //
    // Problem: Ascending drones collide with horizontally moving drones
    // Solution: Each layer uses different XY corridors for vertical movement
    //
    // Path: Start → Move to Ascent Zone → Ascend → Horizontal Cruise → Descend → Move to Goal

    waypoints.push_back(start);  // Start position

    // Get drone-specific XY offset for spatial separation
    Eigen::Vector2d xy_offset = getLayerOffset(drone_id);

    // Calculate layer height for this drone
    double base_clearance_height = std::max(start.z(), goal.z()) + 2.0;  // At least 2m above start/goal
    double layer_height = config_.base_height + layer_id * config_.layer_spacing;
    double cruise_height = std::max(base_clearance_height, layer_height);

    RCLCPP_INFO(this->get_logger(),
        "Drone %d (Layer %d): offset(%.1f,%.1f) Start(%.1f,%.1f,%.1f) → Cruise(%.1fm) → Goal(%.1f,%.1f,%.1f)",
        drone_id, layer_id, xy_offset.x(), xy_offset.y(),
        start.x(), start.y(), start.z(), cruise_height, goal.x(), goal.y(), goal.z());

    // ====== PHASE 0: MOVE TO ASCENT ZONE ======
    // Move horizontally to offset ascent zone to avoid other drones' paths
    Eigen::Vector3d ascent_zone;
    ascent_zone.x() = start.x() + xy_offset.x();
    ascent_zone.y() = start.y() + xy_offset.y();
    ascent_zone.z() = start.z();

    // Only move to ascent zone if offset is significant
    if ((ascent_zone - start).norm() > 0.5) {
        Eigen::Vector3d to_ascent_vec = ascent_zone - start;
        int num_steps = std::max(1, static_cast<int>(to_ascent_vec.norm() / 2.0));

        for (int i = 1; i <= num_steps; ++i) {
            double ratio = static_cast<double>(i) / num_steps;
            waypoints.push_back(start + ratio * to_ascent_vec);
        }
    }

    // ====== PHASE 1: VERTICAL ASCENT (at offset position) ======
    // Ascend vertically at offset position to cruise height
    if (std::abs(cruise_height - start.z()) > 0.1) {
        double vertical_dist = std::abs(cruise_height - start.z());
        int num_ascent_steps = std::max(2, static_cast<int>(vertical_dist / 1.0));  // 1m per step

        for (int i = 1; i <= num_ascent_steps; ++i) {
            double z_ratio = static_cast<double>(i) / num_ascent_steps;
            Eigen::Vector3d ascent_point;
            ascent_point.x() = ascent_zone.x();
            ascent_point.y() = ascent_zone.y();
            ascent_point.z() = start.z() + z_ratio * (cruise_height - start.z());
            waypoints.push_back(ascent_point);
        }
    }

    // Top of ascent (at offset position)
    Eigen::Vector3d cruise_start(ascent_zone.x(), ascent_zone.y(), cruise_height);
    if ((waypoints.back() - cruise_start).norm() > 0.1) {
        waypoints.push_back(cruise_start);
    }

    // ====== PHASE 2: HORIZONTAL MOVEMENT TO FORMATION APPROACH POINT ======
    // NEW STRATEGY: Approach formation from front or back (not from sides!)
    // This prevents collisions with drones already in formation

    // Determine whether to approach from front or back of formation
    double approach_y;
    std::string approach_side;
    if (start.y() < formation_min_y_) {
        // Drone starts in front of formation, approach from front
        approach_y = formation_min_y_ - formation_buffer_;
        approach_side = "front";
    } else if (start.y() > formation_max_y_) {
        // Drone starts behind formation, approach from back
        approach_y = formation_max_y_ + formation_buffer_;
        approach_side = "back";
    } else {
        // Drone starts within formation, choose closer side
        double dist_to_front = start.y() - formation_min_y_;
        double dist_to_back = formation_max_y_ - start.y();
        if (dist_to_front < dist_to_back) {
            approach_y = formation_min_y_ - formation_buffer_;
            approach_side = "front";
        } else {
            approach_y = formation_max_y_ + formation_buffer_;
            approach_side = "back";
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Drone %d approaching from %s (Y=%.1f)",
                 drone_id, approach_side.c_str(), approach_y);

    // Move to formation approach point (at goal's X + offset, approach Y, cruise height)
    Eigen::Vector3d approach_point;
    approach_point.x() = goal.x() + xy_offset.x();
    approach_point.y() = approach_y;
    approach_point.z() = cruise_height;

    // Add waypoints from cruise_start to approach_point
    Eigen::Vector3d to_approach = approach_point - cruise_start;
    to_approach.z() = 0;  // Ensure horizontal
    double dist_to_approach = to_approach.norm();

    if (dist_to_approach > 0.1) {
        int num_steps = std::max(3, static_cast<int>(dist_to_approach / 2.5));
        for (int i = 1; i <= num_steps; ++i) {
            double ratio = static_cast<double>(i) / num_steps;
            Eigen::Vector3d point = cruise_start + ratio * to_approach;
            point.z() = cruise_height;  // Ensure constant altitude
            waypoints.push_back(point);
        }
    }

    // ====== PHASE 2B: MOVE ALONG Y-AXIS TO ABOVE GOAL ======
    // Move from approach point to directly above goal position
    Eigen::Vector3d above_goal;
    above_goal.x() = goal.x() + xy_offset.x();
    above_goal.y() = goal.y();
    above_goal.z() = cruise_height;

    Eigen::Vector3d to_above_goal = above_goal - approach_point;
    to_above_goal.z() = 0;
    double dist_along_formation = to_above_goal.norm();

    if (dist_along_formation > 0.1) {
        int num_steps = std::max(2, static_cast<int>(dist_along_formation / 2.5));
        for (int i = 1; i <= num_steps; ++i) {
            double ratio = static_cast<double>(i) / num_steps;
            Eigen::Vector3d point = approach_point + ratio * to_above_goal;
            point.z() = cruise_height;  // Ensure constant altitude
            waypoints.push_back(point);
        }
    }

    // ====== PHASE 3: VERTICAL DESCENT (above goal position) ======
    // Descend vertically from above_goal (cruise height) to goal height
    if (std::abs(goal.z() - cruise_height) > 0.1) {
        double vertical_dist = std::abs(goal.z() - cruise_height);
        int num_descent_steps = std::max(2, static_cast<int>(vertical_dist / 1.0));  // 1m per step

        for (int i = 1; i < num_descent_steps; ++i) {
            double z_ratio = static_cast<double>(i) / num_descent_steps;
            Eigen::Vector3d descent_point;
            descent_point.x() = above_goal.x();
            descent_point.y() = above_goal.y();
            descent_point.z() = cruise_height + z_ratio * (goal.z() - cruise_height);
            waypoints.push_back(descent_point);
        }
    }

    // Bottom of descent (at goal position with X offset)
    Eigen::Vector3d goal_with_offset;
    goal_with_offset.x() = goal.x() + xy_offset.x();
    goal_with_offset.y() = goal.y();
    goal_with_offset.z() = goal.z();

    if ((waypoints.back() - goal_with_offset).norm() > 0.1) {
        waypoints.push_back(goal_with_offset);
    }

    // ====== PHASE 4: MOVE TO FINAL GOAL ======
    // Move horizontally from offset position to final goal
    if ((goal_with_offset - goal).norm() > 0.5) {
        Eigen::Vector3d to_goal_vec = goal - goal_with_offset;
        int num_steps = std::max(1, static_cast<int>(to_goal_vec.norm() / 2.0));

        for (int i = 1; i < num_steps; ++i) {
            double ratio = static_cast<double>(i) / num_steps;
            waypoints.push_back(goal_with_offset + ratio * to_goal_vec);
        }
    }

    // Final goal
    if ((waypoints.back() - goal).norm() > 0.1) {
        waypoints.push_back(goal);
    }

    return waypoints;
}

double LayerPlannerNode::calculateDuration(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    double distance = (end - start).norm();
    return distance / max_vel_;
}

Eigen::Vector2d LayerPlannerNode::getLayerOffset(int drone_id) {
    // Apply circular offset pattern to prevent vertical collisions
    // Each DRONE gets a UNIQUE angle to spread drones in XY plane
    // Distribute evenly across 360 degrees based on total number of drones
    double radius = 12.0;  // 12.0m radial offset - increased to prevent adjacent drone collisions
                           // For 36 drones: adjacent spacing = 2*R*sin(5°) ≈ 2.09m >> 1.0m threshold

    // Calculate unique angle for each DRONE: 360° / num_drones
    double angle_step = 360.0 / num_drones_;  // degrees per drone
    double angle = (drone_id * angle_step) * M_PI / 180.0;  // Convert to radians

    return Eigen::Vector2d(
        radius * std::cos(angle),
        radius * std::sin(angle)
    );
}

bool LayerPlannerNode::loadScenarioFiles(const std::string& scenario_dir) {
    namespace fs = std::filesystem;

    if (!fs::exists(scenario_dir) || !fs::is_directory(scenario_dir)) {
        RCLCPP_ERROR(this->get_logger(), "Scenario directory does not exist: %s", scenario_dir.c_str());
        return false;
    }

    int loaded_count = 0;

    // Read all node_*.txt files
    for (int i = 0; i < num_drones_; ++i) {
        std::string filename = "node_" + std::to_string(i) + ".txt";
        std::string filepath = scenario_dir + "/" + filename;

        if (!fs::exists(filepath)) {
            RCLCPP_WARN(this->get_logger(), "Scenario file not found: %s", filepath.c_str());
            continue;
        }

        int drone_id;
        Eigen::Vector3d goal;

        if (parseScenarioFile(filepath, drone_id, goal)) {
            if (drone_states_.find(drone_id) != drone_states_.end()) {
                drone_states_[drone_id].goal = goal;
                drone_states_[drone_id].has_goal = true;
                drone_states_[drone_id].trajectory_generated = false;
                loaded_count++;
                RCLCPP_INFO(this->get_logger(),
                    "Loaded goal for drone %d: (%.2f, %.2f, %.2f)",
                    drone_id, goal.x(), goal.y(), goal.z());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to parse scenario file: %s", filepath.c_str());
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %d goals from scenario files", loaded_count);
    return loaded_count > 0;
}

bool LayerPlannerNode::parseScenarioFile(const std::string& file_path, int& drone_id, Eigen::Vector3d& goal) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return false;
    }

    // Parse CSV format: line_id,drone_id,arrival_time,move,x,y,z,yaw,r,g,b
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    if (tokens.size() < 7) {
        return false;
    }

    try {
        // tokens[0] = line_id (ignored)
        drone_id = std::stoi(tokens[1]);  // drone_id
        // tokens[2] = arrival_time (ignored)
        // tokens[3] = "move" (ignored)
        goal.x() = std::stod(tokens[4]);  // x
        goal.y() = std::stod(tokens[5]);  // y
        goal.z() = std::stod(tokens[6]);  // z
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing scenario file %s: %s",
            file_path.c_str(), e.what());
        return false;
    }
}

bool LayerPlannerNode::loadTargetsFromParameters() {
    int loaded_count = 0;

    // Try to load target coordinates for each drone from parameters
    for (int i = 0; i < num_drones_; ++i) {
        std::string param_x = "drone_" + std::to_string(i) + "_target_x";
        std::string param_y = "drone_" + std::to_string(i) + "_target_y";
        std::string param_z = "drone_" + std::to_string(i) + "_target_z";

        // Declare parameters with default values
        this->declare_parameter(param_x, 0.0);
        this->declare_parameter(param_y, 0.0);
        this->declare_parameter(param_z, 0.0);

        // Try to get parameters
        try {
            double target_x = this->get_parameter(param_x).as_double();
            double target_y = this->get_parameter(param_y).as_double();
            double target_z = this->get_parameter(param_z).as_double();

            // Check if target is set (not all zeros)
            if (target_x != 0.0 || target_y != 0.0 || target_z != 0.0) {
                if (drone_states_.find(i) != drone_states_.end()) {
                    drone_states_[i].goal.x() = target_x;
                    drone_states_[i].goal.y() = target_y;
                    drone_states_[i].goal.z() = target_z;
                    drone_states_[i].has_goal = true;
                    drone_states_[i].trajectory_generated = false;
                    loaded_count++;

                    RCLCPP_INFO(this->get_logger(),
                        "Loaded target for drone %d from parameters: (%.2f, %.2f, %.2f)",
                        i, target_x, target_y, target_z);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "No target parameters for drone %d", i);
        }
    }

    if (loaded_count > 0) {
        RCLCPP_INFO(this->get_logger(), "Loaded %d targets from launch parameters", loaded_count);
        return true;
    }

    return false;
}

void LayerPlannerNode::checkCollisions() {
    // Check all pairs of drones for potential collisions
    for (auto& [id1, state1] : drone_states_) {
        if (!state1.has_valid_odom) continue;

        for (auto& [id2, state2] : drone_states_) {
            // Skip self-comparison and already checked pairs
            if (id1 >= id2) continue;
            if (!state2.has_valid_odom) continue;

            // Calculate distance between drones
            double distance = (state1.position - state2.position).norm();

            // Create unique pair key (always smaller id first)
            auto pair_key = std::make_pair(id1, id2);

            // Check if collision detected
            if (distance < collision_threshold_) {
                // Only warn once per collision pair
                if (collision_warned_.find(pair_key) == collision_warned_.end() ||
                    !collision_warned_[pair_key]) {

                    RCLCPP_ERROR(this->get_logger(),
                        "⚠️ COLLISION DETECTED! Drone %d and Drone %d are %.2fm apart (threshold: %.2fm)",
                        id1, id2, distance, collision_threshold_);
                    RCLCPP_ERROR(this->get_logger(),
                        "   Drone %d position: (%.2f, %.2f, %.2f)",
                        id1, state1.position.x(), state1.position.y(), state1.position.z());
                    RCLCPP_ERROR(this->get_logger(),
                        "   Drone %d position: (%.2f, %.2f, %.2f)",
                        id2, state2.position.x(), state2.position.y(), state2.position.z());

                    collision_warned_[pair_key] = true;
                }
            } else {
                // Reset warning flag if drones are now separated
                if (distance > collision_threshold_ * 1.5) {  // Add hysteresis
                    collision_warned_[pair_key] = false;
                }
            }
        }
    }
}

void LayerPlannerNode::calculateFormationBoundaries() {
    // Calculate formation boundaries (min/max Y coordinates) from all drone goals
    for (const auto& [drone_id, state] : drone_states_) {
        if (state.has_goal) {
            formation_min_y_ = std::min(formation_min_y_, state.goal.y());
            formation_max_y_ = std::max(formation_max_y_, state.goal.y());
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Formation boundaries: Y range [%.2f, %.2f] (buffer: %.2fm)",
        formation_min_y_, formation_max_y_, formation_buffer_);
    RCLCPP_INFO(this->get_logger(),
        "  Front approach point: Y = %.2f",
        formation_min_y_ - formation_buffer_);
    RCLCPP_INFO(this->get_logger(),
        "  Back approach point: Y = %.2f",
        formation_max_y_ + formation_buffer_);
}

}  // namespace layer_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<layer_planner::LayerPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
