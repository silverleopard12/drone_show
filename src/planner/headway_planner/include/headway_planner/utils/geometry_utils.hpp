#ifndef HEADWAY_PLANNER_GEOMETRY_UTILS_HPP
#define HEADWAY_PLANNER_GEOMETRY_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace headway_planner {

/**
 * @brief Geometry utility functions
 */
class GeometryUtils {
public:
  /**
   * @brief Compute minimum distance between two line segments in 3D
   *
   * Line 1: P1 + t * u1, t in [0, L1]
   * Line 2: P2 + s * u2, s in [0, L2]
   *
   * @param P1 Start of line 1
   * @param u1 Direction of line 1 (unit vector)
   * @param L1 Length of line 1
   * @param P2 Start of line 2
   * @param u2 Direction of line 2 (unit vector)
   * @param L2 Length of line 2
   * @param t_closest Output: parameter t at closest point on line 1
   * @param s_closest Output: parameter s at closest point on line 2
   * @return Minimum distance
   */
  static double lineSegmentMinDistance(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& u1, double L1,
    const Eigen::Vector3d& P2, const Eigen::Vector3d& u2, double L2,
    double& t_closest, double& s_closest
  );

  /**
   * @brief Compute crossing angle between two directions
   * @return Angle in radians [0, π]
   */
  static double crossingAngle(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2
  );

  /**
   * @brief Compute relative velocity magnitude
   */
  static double relativeVelocity(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2,
    double v_des
  );

  /**
   * @brief Check if two line segments are nearly parallel
   * @param threshold Angle threshold in degrees
   */
  static bool areParallel(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2,
    double threshold_deg
  );

  /**
   * @brief Linear interpolation along path
   */
  static Eigen::Vector3d interpolate(
    const Eigen::Vector3d& P,
    const Eigen::Vector3d& Q,
    double s,
    double L
  );

  /**
   * @brief Clamp value to range [min, max]
   */
  static double clamp(double value, double min_val, double max_val);

  /**
   * @brief Clamp vector components
   */
  static Eigen::Vector3d clamp(
    const Eigen::Vector3d& vec,
    double min_val,
    double max_val
  );
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_GEOMETRY_UTILS_HPP
