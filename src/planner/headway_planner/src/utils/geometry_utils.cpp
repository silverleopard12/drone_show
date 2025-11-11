#include "headway_planner/utils/geometry_utils.hpp"
#include <cmath>
#include <algorithm>

namespace headway_planner {

double GeometryUtils::lineSegmentMinDistance(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& u1, double L1,
    const Eigen::Vector3d& P2, const Eigen::Vector3d& u2, double L2,
    double& t_closest, double& s_closest)
{
  // Line segment distance using linear algebra
  // Line 1: r1(t) = P1 + t*u1, 0 <= t <= L1
  // Line 2: r2(s) = P2 + s*u2, 0 <= s <= L2
  // Minimize: ||r1(t) - r2(s)||^2

  Eigen::Vector3d w = P1 - P2;

  // Coefficients for quadratic minimization
  double a = u1.dot(u1);  // Should be 1.0 for unit vectors
  double b = u1.dot(u2);
  double c = u2.dot(u2);  // Should be 1.0 for unit vectors
  double d = u1.dot(w);
  double e = u2.dot(w);

  double denom = a * c - b * b;

  // Check if lines are parallel
  if (std::abs(denom) < 1e-9) {
    // Parallel lines - find closest endpoint
    t_closest = 0.0;

    // Project P1 onto line 2
    s_closest = -e / c;
    s_closest = std::max(0.0, std::min(L2, s_closest));

    Eigen::Vector3d closest_point = P2 + s_closest * u2;
    return (P1 - closest_point).norm();
  }

  // Solve for unconstrained minimum
  double t_unconstrained = (b * e - c * d) / denom;
  double s_unconstrained = (a * e - b * d) / denom;

  // Clamp to segment bounds
  t_closest = std::max(0.0, std::min(L1, t_unconstrained));
  s_closest = std::max(0.0, std::min(L2, s_unconstrained));

  // If clamping occurred, re-optimize the other parameter
  if (t_unconstrained < 0.0 || t_unconstrained > L1) {
    // t was clamped, re-optimize s
    s_closest = std::max(0.0, std::min(L2, (b * t_closest - e) / c));
  }

  if (s_unconstrained < 0.0 || s_unconstrained > L2) {
    // s was clamped, re-optimize t
    t_closest = std::max(0.0, std::min(L1, (b * s_closest + d) / a));
  }

  // Compute final distance
  Eigen::Vector3d point1 = P1 + t_closest * u1;
  Eigen::Vector3d point2 = P2 + s_closest * u2;

  return (point1 - point2).norm();
}

double GeometryUtils::crossingAngle(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2)
{
  // Compute angle between unit vectors (in degrees)
  double dot_product = u1.dot(u2);

  // Clamp to avoid numerical issues with acos
  dot_product = std::max(-1.0, std::min(1.0, dot_product));

  double angle_rad = std::acos(std::abs(dot_product));
  double angle_deg = angle_rad * 180.0 / M_PI;

  return angle_deg;
}

double GeometryUtils::relativeVelocity(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2,
    double v_des)
{
  // Relative velocity magnitude at crossing point
  // v_rel = ||v1 - v2|| where v1 = v_des * u1, v2 = v_des * u2

  Eigen::Vector3d v1 = v_des * u1;
  Eigen::Vector3d v2 = v_des * u2;

  return (v1 - v2).norm();
}

bool GeometryUtils::areParallel(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2,
    double threshold_deg)
{
  double angle = crossingAngle(u1, u2);
  return angle < threshold_deg;
}

Eigen::Vector3d GeometryUtils::interpolate(
    const Eigen::Vector3d& P,
    const Eigen::Vector3d& Q,
    double s,
    double L)
{
  // Linear interpolation along path
  // s: arc length parameter [0, L]

  if (L < 1e-9) {
    return P;
  }

  double t = s / L;  // Normalized parameter [0, 1]
  t = std::max(0.0, std::min(1.0, t));

  return P + t * (Q - P);
}

double GeometryUtils::clamp(double value, double min_val, double max_val) {
  return std::max(min_val, std::min(max_val, value));
}

Eigen::Vector3d GeometryUtils::clamp(
    const Eigen::Vector3d& vec,
    double min_val,
    double max_val)
{
  return Eigen::Vector3d(
    clamp(vec.x(), min_val, max_val),
    clamp(vec.y(), min_val, max_val),
    clamp(vec.z(), min_val, max_val)
  );
}

} // namespace headway_planner
