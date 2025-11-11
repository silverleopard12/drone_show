#include "headway_planner/headway_planner_node.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace fs = std::filesystem;

namespace headway_planner {

HeadwayPlannerNode::HeadwayPlannerNode()
  : Node("headway_planner_node"),
    num_drones_(10)
{
  RCLCPP_INFO(this->get_logger(), "Headway Planner Node starting...");

  // Load parameters
  loadParameters();

  // Create publishers
  vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.vis_topic, 10);

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    config_.status_topic, 10);

  // Create waypoint publishers for each drone
  // Use transient_local QoS so late-joining executors can receive waypoints
  rclcpp::QoS qos(10);
  qos.transient_local();  // Late joiners will receive last published message

  for (int i = 0; i < num_drones_; ++i) {
    auto pub = this->create_publisher<nav_msgs::msg::Path>(
      "/drone_" + std::to_string(i) + "/headway/waypoints", qos);
    waypoint_pubs_.push_back(pub);
  }

  // Create output directory
  fs::create_directories(output_dir_);

  RCLCPP_INFO(this->get_logger(), "Headway Planner Node initialized.");
}

HeadwayPlannerNode::~HeadwayPlannerNode() {
}

bool HeadwayPlannerNode::initialize() {
  RCLCPP_INFO(this->get_logger(), "Initializing planner components...");

  // Create components
  event_extractor_ = std::make_shared<EventExtractor>(config_);

  if (config_.scheduler_type == "greedy") {
    scheduler_ = std::make_shared<GreedyScheduler>(config_);
  } else if (config_.scheduler_type == "milp") {
    scheduler_ = std::make_shared<MILPScheduler>(config_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown scheduler type: %s", config_.scheduler_type.c_str());
    return false;
  }

  slot_allocator_ = std::make_shared<SlotAllocator>(config_);
  plan_generator_ = std::make_shared<PlanGenerator>(config_);

  // Configure plan generator
  plan_generator_->setTakeoffAltitude(config_.takeoff_altitude);
  plan_generator_->enableLanding(config_.enable_landing);
  plan_generator_->setGoalHoverTime(config_.goal_hover_time);
  plan_generator_->setWaypointSpacing(config_.waypoint_spacing);

  RCLCPP_INFO(this->get_logger(), "Components initialized.");
  return true;
}

bool HeadwayPlannerNode::plan() {
  RCLCPP_INFO(this->get_logger(), "\n========================================");
  RCLCPP_INFO(this->get_logger(), "Starting Headway Planner");
  RCLCPP_INFO(this->get_logger(), "========================================");

  publishStatus("PLANNING_STARTED");

  // Step 1: Load mission
  if (!loadMission()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mission.");
    publishStatus("LOAD_MISSION_FAILED");
    return false;
  }

  // Step 2: Extract collision events
  if (!extractCollisionEvents()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract collision events.");
    publishStatus("EVENT_EXTRACTION_FAILED");
    return false;
  }

  // Step 3: Solve headways
  if (!solveHeadways()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to solve headways.");
    publishStatus("HEADWAY_SOLVING_FAILED");
    return false;
  }

  // Step 4: Allocate altitude slots
  if (!allocateAltitudeSlots()) {
    RCLCPP_WARN(this->get_logger(), "Altitude slot allocation had issues.");
  }

  // Step 5: Generate plan files
  if (!generatePlanFiles()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate plan files.");
    publishStatus("PLAN_GENERATION_FAILED");
    return false;
  }

  // Step 6: Publish waypoints to executor nodes
  publishWaypoints();

  // Step 7: Visualize and summarize
  visualize();
  printStatistics();

  if (config_.save_summary) {
    saveSummary(output_dir_ + "/" + config_.summary_file);
  }

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Headway Planner completed successfully!");
  RCLCPP_INFO(this->get_logger(), "========================================\n");

  publishStatus("PLANNING_COMPLETED");

  return true;
}

void HeadwayPlannerNode::loadParameters() {
  // Declare and get parameters
  this->declare_parameter("v_max", 10.0);
  this->declare_parameter("a_max", 5.0);
  this->declare_parameter("j_max", 10.0);
  this->declare_parameter("v_z_max", 3.0);
  this->declare_parameter("a_z_max", 2.0);
  this->declare_parameter("d_min", 2.0);
  this->declare_parameter("epsilon_t", 0.2);
  this->declare_parameter("e_pos", 0.1);
  this->declare_parameter("v_des", 5.0);
  this->declare_parameter("alpha", 1.2);
  this->declare_parameter("T_max", 300.0);
  this->declare_parameter("d_gate", 10.0);
  this->declare_parameter("parallel_threshold", 5.0);
  this->declare_parameter("h_slot", 2.0);
  this->declare_parameter("enable_slotting", true);
  this->declare_parameter("slot_threshold", 1.0);
  this->declare_parameter("scheduler_type", "greedy");
  this->declare_parameter("max_iterations", 100);
  this->declare_parameter("tolerance", 0.01);
  this->declare_parameter("takeoff_altitude", 5.0);
  this->declare_parameter("goal_hover_time", 2.0);
  this->declare_parameter("enable_landing", false);
  this->declare_parameter("waypoint_spacing", 5.0);
  this->declare_parameter("mission_source", "formation");
  this->declare_parameter("formation.num_drones", 10);
  this->declare_parameter("formation.grid_rows", 2);
  this->declare_parameter("formation.grid_cols", 5);
  this->declare_parameter("formation.spacing", 5.0);
  this->declare_parameter("output_dir", "/tmp/headway_planner_output");
  this->declare_parameter("plan_file_prefix", "drone_");
  this->declare_parameter("save_summary", true);
  this->declare_parameter("summary_file", "summary.txt");
  this->declare_parameter("verbose", true);
  this->declare_parameter("print_events", true);
  this->declare_parameter("print_statistics", true);
  this->declare_parameter("vis_topic", "/headway_planner/visualization");
  this->declare_parameter("status_topic", "/headway_planner/status");

  // Get parameters
  config_.v_max = this->get_parameter("v_max").as_double();
  config_.a_max = this->get_parameter("a_max").as_double();
  config_.j_max = this->get_parameter("j_max").as_double();
  config_.v_z_max = this->get_parameter("v_z_max").as_double();
  config_.a_z_max = this->get_parameter("a_z_max").as_double();
  config_.d_min = this->get_parameter("d_min").as_double();
  config_.epsilon_t = this->get_parameter("epsilon_t").as_double();
  config_.e_pos = this->get_parameter("e_pos").as_double();
  config_.v_des = this->get_parameter("v_des").as_double();
  config_.alpha = this->get_parameter("alpha").as_double();
  config_.T_max = this->get_parameter("T_max").as_double();
  config_.d_gate = this->get_parameter("d_gate").as_double();
  config_.parallel_threshold = this->get_parameter("parallel_threshold").as_double();
  config_.h_slot = this->get_parameter("h_slot").as_double();
  config_.enable_slotting = this->get_parameter("enable_slotting").as_bool();
  config_.slot_threshold = this->get_parameter("slot_threshold").as_double();
  config_.scheduler_type = this->get_parameter("scheduler_type").as_string();
  config_.max_iterations = this->get_parameter("max_iterations").as_int();
  config_.tolerance = this->get_parameter("tolerance").as_double();
  config_.takeoff_altitude = this->get_parameter("takeoff_altitude").as_double();
  config_.goal_hover_time = this->get_parameter("goal_hover_time").as_double();
  config_.enable_landing = this->get_parameter("enable_landing").as_bool();
  config_.waypoint_spacing = this->get_parameter("waypoint_spacing").as_double();
  config_.mission_source = this->get_parameter("mission_source").as_string();
  num_drones_ = this->get_parameter("formation.num_drones").as_int();
  output_dir_ = this->get_parameter("output_dir").as_string();
  config_.plan_file_prefix = this->get_parameter("plan_file_prefix").as_string();
  config_.save_summary = this->get_parameter("save_summary").as_bool();
  config_.summary_file = this->get_parameter("summary_file").as_string();
  config_.verbose = this->get_parameter("verbose").as_bool();
  config_.print_events = this->get_parameter("print_events").as_bool();
  config_.print_statistics = this->get_parameter("print_statistics").as_bool();
  config_.vis_topic = this->get_parameter("vis_topic").as_string();
  config_.status_topic = this->get_parameter("status_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Scheduler: %s", config_.scheduler_type.c_str());
  RCLCPP_INFO(this->get_logger(), "  Num drones: %d", num_drones_);
  RCLCPP_INFO(this->get_logger(), "  v_des: %.2f m/s", config_.v_des);
  RCLCPP_INFO(this->get_logger(), "  d_min: %.2f m", config_.d_min);
  RCLCPP_INFO(this->get_logger(), "  Output dir: %s", output_dir_.c_str());
}

bool HeadwayPlannerNode::loadMission() {
  RCLCPP_INFO(this->get_logger(), "Loading mission (source: %s)...", config_.mission_source.c_str());

  drones_.clear();

  if (config_.mission_source == "formation") {
    // Load from formation generator
    int rows = this->get_parameter("formation.grid_rows").as_int();
    int cols = this->get_parameter("formation.grid_cols").as_int();
    double spacing = this->get_parameter("formation.spacing").as_double();

    std::vector<Eigen::Vector3d> start_positions, goal_positions;

    Eigen::Vector3d start_center(0.0, 0.0, 5.0);
    Eigen::Vector3d goal_center(50.0, 0.0, 5.0);

    loadGridFormation(rows, cols, spacing, start_center, start_positions);
    loadGridFormation(rows, cols, spacing, goal_center, goal_positions);

    for (int i = 0; i < num_drones_; ++i) {
      DroneInfo drone;
      drone.id = i;
      drone.P = start_positions[i];
      drone.Q = goal_positions[i];
      drone.L = (drone.Q - drone.P).norm();
      drone.u_hat = (drone.Q - drone.P) / drone.L;
      drone.phi = 0.0;
      drone.slack = 0.0;

      drones_.push_back(drone);
    }

  } else {
    RCLCPP_ERROR(this->get_logger(), "Mission source '%s' not implemented yet.", config_.mission_source.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu drones.", drones_.size());
  return true;
}

void HeadwayPlannerNode::loadGridFormation(
    int rows, int cols, double spacing,
    const Eigen::Vector3d& center,
    std::vector<Eigen::Vector3d>& positions)
{
  positions.clear();

  double offset_x = -(cols - 1) * spacing / 2.0;
  double offset_y = -(rows - 1) * spacing / 2.0;

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      Eigen::Vector3d pos = center;
      pos.x() += offset_x + c * spacing;
      pos.y() += offset_y + r * spacing;
      positions.push_back(pos);
    }
  }
}

bool HeadwayPlannerNode::extractCollisionEvents() {
  RCLCPP_INFO(this->get_logger(), "Extracting collision events...");

  events_ = event_extractor_->extractEvents(drones_);

  RCLCPP_INFO(this->get_logger(), "Detected %zu collision events.", events_.size());

  if (config_.print_events && config_.verbose) {
    event_extractor_->printStatistics(events_);
  }

  return true;
}

bool HeadwayPlannerNode::solveHeadways() {
  RCLCPP_INFO(this->get_logger(), "Solving headways with %s scheduler...", config_.scheduler_type.c_str());

  bool success = scheduler_->solve(drones_, events_);

  if (success) {
    double makespan = scheduler_->getMakespan(drones_);
    RCLCPP_INFO(this->get_logger(), "Headway solving succeeded. Makespan: %.2f s", makespan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Headway solving failed.");
  }

  return success;
}

bool HeadwayPlannerNode::allocateAltitudeSlots() {
  RCLCPP_INFO(this->get_logger(), "Allocating altitude slots...");

  int num_slots = slot_allocator_->allocateSlots(drones_, events_);

  RCLCPP_INFO(this->get_logger(), "Allocated %d altitude slots.", num_slots);

  return true;
}

bool HeadwayPlannerNode::generatePlanFiles() {
  RCLCPP_INFO(this->get_logger(), "Generating PX4 .plan files...");

  int num_generated = plan_generator_->generateAllPlans(
    drones_, output_dir_, config_.plan_file_prefix);

  if (num_generated != static_cast<int>(drones_.size())) {
    RCLCPP_ERROR(this->get_logger(), "Only generated %d/%zu plans.", num_generated, drones_.size());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Generated %d plan files.", num_generated);
  return true;
}

void HeadwayPlannerNode::visualize() {
  if (!config_.enable_visualization) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Publishing visualization...");

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;

  // Visualize drone trajectories
  for (size_t i = 0; i < drones_.size(); ++i) {
    const auto& drone = drones_[i];

    // Path as LINE_STRIP
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = this->now();
    path_marker.ns = "drone_paths";
    path_marker.id = marker_id++;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.1;  // Line width

    // Color based on drone ID
    path_marker.color.r = (i % 3 == 0) ? 1.0 : 0.0;
    path_marker.color.g = (i % 3 == 1) ? 1.0 : 0.0;
    path_marker.color.b = (i % 3 == 2) ? 1.0 : 0.0;
    path_marker.color.a = 0.8;

    // Sample points along path
    double s = 0.0;
    while (s <= drone.L) {
      Eigen::Vector3d pos = drone.getPositionAtS(s);

      // Apply altitude offset (find slot for current s position)
      for (const auto& slot : drone.slots) {
        if (s >= slot.s_start && s <= slot.s_end) {
          pos.z() += slot.h_offset;
          break;
        }
      }

      geometry_msgs::msg::Point p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      path_marker.points.push_back(p);

      s += config_.waypoint_spacing;
    }

    markers.markers.push_back(path_marker);

    // Start point
    visualization_msgs::msg::Marker start_marker;
    start_marker.header = path_marker.header;
    start_marker.ns = "start_points";
    start_marker.id = marker_id++;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position.x = drone.P.x();
    start_marker.pose.position.y = drone.P.y();
    start_marker.pose.position.z = drone.P.z();
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.5;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    markers.markers.push_back(start_marker);

    // Goal point
    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.ns = "goal_points";
    goal_marker.id = marker_id++;
    goal_marker.pose.position.x = drone.Q.x();
    goal_marker.pose.position.y = drone.Q.y();
    goal_marker.pose.position.z = drone.Q.z();
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    markers.markers.push_back(goal_marker);
  }

  // Visualize collision events
  for (const auto& event : events_) {
    visualization_msgs::msg::Marker event_marker;
    event_marker.header.frame_id = "world";
    event_marker.header.stamp = this->now();
    event_marker.ns = "collision_events";
    event_marker.id = marker_id++;
    event_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    event_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position at event location
    const auto& drone_i = drones_[event.drone_i];
    Eigen::Vector3d pos_i = drone_i.getPositionAtS(event.s_i);

    event_marker.pose.position.x = pos_i.x();
    event_marker.pose.position.y = pos_i.y();
    event_marker.pose.position.z = pos_i.z();

    event_marker.scale.x = event_marker.scale.y = config_.d_min * 2;
    event_marker.scale.z = 0.2;

    event_marker.color.r = 1.0;
    event_marker.color.g = 1.0;
    event_marker.color.b = 0.0;
    event_marker.color.a = 0.3;

    markers.markers.push_back(event_marker);
  }

  vis_pub_->publish(markers);
  RCLCPP_INFO(this->get_logger(), "Published %zu visualization markers", markers.markers.size());
}

void HeadwayPlannerNode::publishWaypoints() {
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints to executor nodes...");

  for (size_t i = 0; i < drones_.size(); ++i) {
    const auto& drone = drones_[i];

    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = this->now();

    // Extract waypoints along the drone's trajectory
    double s = 0.0;
    while (s <= drone.L) {
      Eigen::Vector3d pos = drone.getPositionAtS(s);

      // Apply altitude slotting
      for (const auto& slot : drone.slots) {
        if (s >= slot.s_start && s <= slot.s_end) {
          pos.z() += slot.h_offset;
          break;
        }
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.header.stamp = this->now();
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = pos.z();

      // Orientation (facing forward along trajectory)
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;

      path.poses.push_back(pose);

      s += config_.waypoint_spacing;
    }

    RCLCPP_INFO(this->get_logger(), "Drone %zu: Publishing %zu waypoints", i, path.poses.size());
    waypoint_pubs_[i]->publish(path);
  }

  RCLCPP_INFO(this->get_logger(), "All waypoints published to %zu drones", drones_.size());
}

void HeadwayPlannerNode::printStatistics() {
  if (!config_.print_statistics) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n=== Headway Planner Statistics ===");
  RCLCPP_INFO(this->get_logger(), "Drones: %zu", drones_.size());
  RCLCPP_INFO(this->get_logger(), "Events detected: %zu", events_.size());
  RCLCPP_INFO(this->get_logger(), "Makespan: %.2f s", scheduler_->getMakespan(drones_));
  RCLCPP_INFO(this->get_logger(), "Scheduler: %s", config_.scheduler_type.c_str());

  auto slot_stats = slot_allocator_->getStatistics();
  RCLCPP_INFO(this->get_logger(), "Altitude slots: %d", slot_stats.slots_allocated);
  RCLCPP_INFO(this->get_logger(), "==================================\n");
}

void HeadwayPlannerNode::publishStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

bool HeadwayPlannerNode::saveSummary(const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open summary file: %s", filename.c_str());
    return false;
  }

  file << "Headway Planner Results\n";
  file << "=======================\n\n";
  file << "Drones: " << drones_.size() << "\n";
  file << "Events detected: " << events_.size() << "\n";
  file << "Makespan: " << std::fixed << std::setprecision(2)
       << scheduler_->getMakespan(drones_) << " s\n";
  file << "Scheduler: " << config_.scheduler_type << "\n";

  auto slot_stats = slot_allocator_->getStatistics();
  file << "Altitude slots used: " << slot_stats.slots_allocated << "\n";
  file << "Layers used: " << slot_stats.num_layers_used << "\n";

  file.close();

  RCLCPP_INFO(this->get_logger(), "Summary saved to: %s", filename.c_str());
  return true;
}

} // namespace headway_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<headway_planner::HeadwayPlannerNode>();

  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed.");
    rclcpp::shutdown();
    return 1;
  }

  if (!node->plan()) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    rclcpp::shutdown();
    return 1;
  }

  // Keep node alive for visualization
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
