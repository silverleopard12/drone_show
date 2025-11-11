#include "orca_planner/utils/visualization_utils.hpp"
#include <cmath>

namespace orca_planner {

VisualizationUtils::VisualizationUtils(rclcpp::Node::SharedPtr node)
  : node_(node),
    frame_id_("world"),
    marker_id_(0)
{
}

VisualizationUtils::~VisualizationUtils() {
}

void VisualizationUtils::initialize() {
  // Create publishers
  trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/orca_planner/trajectories", 10);

  agent_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/orca_planner/agents", 10);

  formation_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/orca_planner/formations", 10);

  debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/orca_planner/debug", 10);

  RCLCPP_INFO(node_->get_logger(), "Visualization publishers initialized.");
}

void VisualizationUtils::visualizeTrajectories(
    const std::vector<DroneTrajectory>& trajectories)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_id_ = 0;

  for (const auto& traj : trajectories) {
    auto color = getColorForDrone(traj.drone_id);
    auto marker = createTrajectoryMarker(traj, color, marker_id_++);
    marker_array.markers.push_back(marker);
  }

  trajectory_pub_->publish(marker_array);
}

void VisualizationUtils::visualizeTrajectory(
    const DroneTrajectory& trajectory,
    const std::vector<double>& color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = createTrajectoryMarker(trajectory, color, 0);
  marker_array.markers.push_back(marker);

  trajectory_pub_->publish(marker_array);
}

void VisualizationUtils::visualizeDronePositions(
    const std::vector<AgentState>& agents)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_id_ = 0;

  for (const auto& agent : agents) {
    auto marker = createAgentMarker(agent, marker_id_);
    marker_array.markers.push_back(marker);

    // Add text label
    auto text_marker = createTextMarker(
      "D" + std::to_string(agent.id),
      agent.position + Eigen::Vector3d(0, 0, 0.5),
      marker_id_ + 1000
    );
    marker_array.markers.push_back(text_marker);

    marker_id_++;
  }

  agent_pub_->publish(marker_array);
}

void VisualizationUtils::visualizeORCAPlanes(
    const std::vector<ORCAPlane>& planes,
    const Eigen::Vector3d& agent_position)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_id_ = 0;

  for (const auto& plane : planes) {
    // Create arrow marker for plane normal
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "orca_planes";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Arrow from agent position in direction of plane normal
    geometry_msgs::msg::Point start, end;
    start.x = agent_position.x();
    start.y = agent_position.y();
    start.z = agent_position.z();

    Eigen::Vector3d end_pos = agent_position + plane.normal * 2.0;
    end.x = end_pos.x();
    end.y = end_pos.y();
    end.z = end_pos.z();

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.1;  // Arrow shaft diameter
    marker.scale.y = 0.2;  // Arrow head diameter
    marker.scale.z = 0.3;  // Arrow head length

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    marker_array.markers.push_back(marker);
  }

  debug_pub_->publish(marker_array);
}

void VisualizationUtils::visualizeFormation(
    const Formation& formation,
    const std::string& ns)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_id_ = 0;

  for (const auto& pos : formation.positions) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.6;

    marker_array.markers.push_back(marker);
  }

  formation_pub_->publish(marker_array);
}

void VisualizationUtils::visualizeWaypoints(
    const std::vector<Waypoint>& waypoints,
    const std::string& ns)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_id_ = 0;

  // Waypoint spheres
  for (const auto& wp : waypoints) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = wp.position.x();
    marker.pose.position.y = wp.position.y();
    marker.pose.position.z = wp.position.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    marker_array.markers.push_back(marker);
  }

  // Connect waypoints with line strip
  if (waypoints.size() > 1) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = node_->now();
    line_marker.ns = ns + "_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;

    for (const auto& wp : waypoints) {
      geometry_msgs::msg::Point p;
      p.x = wp.position.x();
      p.y = wp.position.y();
      p.z = wp.position.z();
      line_marker.points.push_back(p);
    }

    line_marker.scale.x = 0.1;  // Line width

    line_marker.color.r = 1.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.5;

    marker_array.markers.push_back(line_marker);
  }

  formation_pub_->publish(marker_array);
}

void VisualizationUtils::clearAll() {
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  marker_array.markers.push_back(delete_marker);

  trajectory_pub_->publish(marker_array);
  agent_pub_->publish(marker_array);
  formation_pub_->publish(marker_array);
  debug_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker VisualizationUtils::createTrajectoryMarker(
    const DroneTrajectory& trajectory,
    const std::vector<double>& color,
    int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "trajectories";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Add all positions as line strip
  for (const auto& pos : trajectory.positions) {
    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    marker.points.push_back(p);
  }

  marker.scale.x = 0.1;  // Line width

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = (color.size() > 3) ? color[3] : 0.8;

  return marker;
}

visualization_msgs::msg::Marker VisualizationUtils::createAgentMarker(
    const AgentState& agent,
    int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "agents";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = agent.position.x();
  marker.pose.position.y = agent.position.y();
  marker.pose.position.z = agent.position.z();
  marker.pose.orientation.w = 1.0;

  double radius = agent.radius;
  marker.scale.x = radius * 2.0;
  marker.scale.y = radius * 2.0;
  marker.scale.z = radius * 2.0;

  auto color = getColorForDrone(agent.id);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.9;

  return marker;
}

visualization_msgs::msg::Marker VisualizationUtils::createTextMarker(
    const std::string& text,
    const Eigen::Vector3d& position,
    int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "text";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.pose.orientation.w = 1.0;

  marker.scale.z = 0.5;  // Text height
  marker.text = text;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

std::vector<double> VisualizationUtils::getColorForDrone(int drone_id) const {
  // Generate unique color for each drone
  // Use HSV to RGB conversion with varying hue

  double hue = std::fmod(drone_id * 137.5, 360.0);  // Golden angle
  double saturation = 0.8;
  double value = 0.9;

  // HSV to RGB conversion
  double c = value * saturation;
  double x = c * (1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0));
  double m = value - c;

  double r, g, b;
  if (hue < 60) {
    r = c; g = x; b = 0;
  } else if (hue < 120) {
    r = x; g = c; b = 0;
  } else if (hue < 180) {
    r = 0; g = c; b = x;
  } else if (hue < 240) {
    r = 0; g = x; b = c;
  } else if (hue < 300) {
    r = x; g = 0; b = c;
  } else {
    r = c; g = 0; b = x;
  }

  return {r + m, g + m, b + m};
}

} // namespace orca_planner
