#include "headway_planner/trajectory/plan_generator.hpp"
#include "headway_planner/utils/geometry_utils.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

using json = nlohmann::json;

namespace headway_planner {

PlanGenerator::PlanGenerator(const PlannerConfig& config)
  : config_(config),
    takeoff_altitude_(5.0),
    enable_landing_(false),
    goal_hover_time_(2.0),
    waypoint_spacing_(5.0)
{
}

PlanGenerator::~PlanGenerator() {
}

bool PlanGenerator::generatePlan(
    const DroneInfo& drone,
    const std::string& output_file)
{
  std::cout << "[PlanGenerator] Generating plan for drone " << drone.id
            << " -> " << output_file << std::endl;

  // Build PX4Plan structure
  PX4Plan plan = buildPlan(drone);

  // Write JSON file
  return writeJSON(plan, output_file);
}

int PlanGenerator::generateAllPlans(
    const std::vector<DroneInfo>& drones,
    const std::string& output_dir,
    const std::string& prefix)
{
  int num_generated = 0;

  for (const auto& drone : drones) {
    std::string filename = output_dir + "/" + prefix + std::to_string(drone.id) + ".plan";
    if (generatePlan(drone, filename)) {
      num_generated++;
    }
  }

  std::cout << "[PlanGenerator] Generated " << num_generated << " plan files in "
            << output_dir << std::endl;

  return num_generated;
}

PX4Plan PlanGenerator::buildPlan(const DroneInfo& drone) {
  PX4Plan plan;

  plan.fileType = "Plan";
  plan.geoFence = json::object();
  plan.groundStation = "QGroundControl";
  plan.rallyPoints = json::object();
  plan.version = 1;

  int seq = 0;

  // Add initial sequence (takeoff, loiter for headway)
  addInitialSequence(plan, drone, seq);

  // Add main waypoint sequence
  addMainWaypoints(plan, drone, seq);

  // Add final sequence (goal hover, land)
  addFinalSequence(plan, drone, seq);

  return plan;
}

void PlanGenerator::addInitialSequence(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq)
{
  // 1. TAKEOFF
  auto takeoff = createTakeoff(seq++, drone.P, takeoff_altitude_);
  plan.mission.items.push_back(takeoff);

  // 2. LOITER_TIME for headway offset
  if (drone.phi > 0.01) {
    auto loiter = createLoiterTime(seq++, drone.P, drone.phi);
    plan.mission.items.push_back(loiter);
  }

  // 3. DO_CHANGE_SPEED
  auto change_speed = createChangeSpeed(seq++, config_.v_des);
  plan.mission.items.push_back(change_speed);
}

void PlanGenerator::addMainWaypoints(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq)
{
  // Sample waypoints along path
  auto waypoints = sampleWaypoints(drone);

  for (const auto& wp : waypoints) {
    auto waypoint = createWaypoint(seq++, wp);
    plan.mission.items.push_back(waypoint);
  }
}

void PlanGenerator::addFinalSequence(
    PX4Plan& plan,
    const DroneInfo& drone,
    int& seq)
{
  // Goal hover
  if (goal_hover_time_ > 0.01) {
    auto hover = createLoiterTime(seq++, drone.Q, goal_hover_time_);
    plan.mission.items.push_back(hover);
  }

  // Landing
  if (enable_landing_) {
    auto land = createLand(seq++, drone.Q);
    plan.mission.items.push_back(land);
  }

  // Set planned home position
  plan.mission.plannedHomePosition = {drone.P.x(), drone.P.y(), drone.P.z()};
  plan.mission.firmwareType = 12;  // PX4 Pro
  plan.mission.vehicleType = 2;    // Quadcopter
}

PX4MissionItem PlanGenerator::createWaypoint(
    int seq,
    const Eigen::Vector3d& position,
    double param1)
{
  PX4MissionItem item;
  item.autoContinue = true;
  item.command = 16;  // MAV_CMD_NAV_WAYPOINT
  item.doJumpId = seq;
  item.frame = 3;     // MAV_FRAME_GLOBAL_RELATIVE_ALT
  item.params = {param1, 0, 0, std::nan(""), position.x(), position.y(), position.z()};
  item.type = "SimpleItem";

  return item;
}

PX4MissionItem PlanGenerator::createLoiterTime(
    int seq,
    const Eigen::Vector3d& position,
    double duration)
{
  PX4MissionItem item;
  item.autoContinue = true;
  item.command = 19;  // MAV_CMD_NAV_LOITER_TIME
  item.doJumpId = seq;
  item.frame = 3;
  item.params = {duration, 0, 0, std::nan(""), position.x(), position.y(), position.z()};
  item.type = "SimpleItem";

  return item;
}

PX4MissionItem PlanGenerator::createChangeSpeed(
    int seq,
    double speed)
{
  PX4MissionItem item;
  item.autoContinue = true;
  item.command = 178;  // MAV_CMD_DO_CHANGE_SPEED
  item.doJumpId = seq;
  item.frame = 2;      // MAV_FRAME_MISSION
  item.params = {1, speed, -1, 0, 0, 0, 0};  // Speed type 1 = groundspeed
  item.type = "SimpleItem";

  return item;
}

PX4MissionItem PlanGenerator::createTakeoff(
    int seq,
    const Eigen::Vector3d& position,
    double altitude)
{
  PX4MissionItem item;
  item.autoContinue = true;
  item.command = 22;  // MAV_CMD_NAV_TAKEOFF
  item.doJumpId = seq;
  item.frame = 3;
  item.params = {0, 0, 0, std::nan(""), position.x(), position.y(), altitude};
  item.type = "SimpleItem";

  return item;
}

PX4MissionItem PlanGenerator::createLand(
    int seq,
    const Eigen::Vector3d& position)
{
  PX4MissionItem item;
  item.autoContinue = true;
  item.command = 21;  // MAV_CMD_NAV_LAND
  item.doJumpId = seq;
  item.frame = 3;
  item.params = {0, 0, 0, std::nan(""), position.x(), position.y(), 0};
  item.type = "SimpleItem";

  return item;
}

std::vector<Eigen::Vector3d> PlanGenerator::sampleWaypoints(const DroneInfo& drone) {
  std::vector<Eigen::Vector3d> waypoints;

  // Start waypoint
  waypoints.push_back(drone.P);

  // Sample intermediate waypoints with altitude slots
  double s = 0.0;
  while (s < drone.L) {
    s += waypoint_spacing_;
    if (s >= drone.L) break;

    // Get altitude offset at this position
    double h_offset = getAltitudeOffsetAtS(drone, s);

    // Interpolate position
    Eigen::Vector3d pos = drone.P + (s / drone.L) * (drone.Q - drone.P);
    pos.z() += h_offset;

    waypoints.push_back(pos);
  }

  // Goal waypoint
  waypoints.push_back(drone.Q);

  return waypoints;
}

double PlanGenerator::getAltitudeOffsetAtS(const DroneInfo& drone, double s) const {
  // Check if s is within any slot
  for (const auto& slot : drone.slots) {
    if (s >= slot.s_start && s <= slot.s_end) {
      // Smooth transition using cosine interpolation
      double progress = (s - slot.s_start) / (slot.s_end - slot.s_start);

      // Cosine ramp up/down for smooth vertical profile
      if (progress < 0.5) {
        // Climbing phase
        return slot.h_offset * (1.0 - std::cos(progress * 2.0 * M_PI)) / 2.0;
      } else {
        // Descending phase
        return slot.h_offset * (1.0 + std::cos((progress - 0.5) * 2.0 * M_PI)) / 2.0;
      }
    }
  }

  return 0.0;  // No slot at this position
}

bool PlanGenerator::writeJSON(const PX4Plan& plan, const std::string& filename) {
  try {
    json j;

    j["fileType"] = plan.fileType;
    j["geoFence"] = plan.geoFence;
    j["groundStation"] = plan.groundStation;
    j["rallyPoints"] = plan.rallyPoints;
    j["version"] = plan.version;

    // Mission items
    json mission;
    mission["cruiseSpeed"] = config_.v_des;
    mission["firmwareType"] = plan.mission.firmwareType;
    mission["hoverSpeed"] = config_.v_des * 0.5;
    mission["vehicleType"] = plan.mission.vehicleType;

    // Planned home position
    mission["plannedHomePosition"] = plan.mission.plannedHomePosition;

    // Items
    json items = json::array();
    for (const auto& item : plan.mission.items) {
      json j_item;
      j_item["autoContinue"] = item.autoContinue;
      j_item["command"] = item.command;
      j_item["doJumpId"] = item.doJumpId;
      j_item["frame"] = item.frame;
      j_item["params"] = item.params;
      j_item["type"] = item.type;

      items.push_back(j_item);
    }

    mission["items"] = items;
    j["mission"] = mission;

    // Write to file
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "[PlanGenerator] ERROR: Could not open file " << filename << std::endl;
      return false;
    }

    file << j.dump(2);  // Pretty print with 2-space indentation
    file.close();

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[PlanGenerator] ERROR: " << e.what() << std::endl;
    return false;
  }
}

} // namespace headway_planner
