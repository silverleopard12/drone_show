#ifndef __TYPE_CONVERTOR_H__
#define __TYPE_CONVERTOR_H__

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace catora_planner
{

struct Vector3d
{
  double x;
  double y;
  double z;
};

struct Vector4d
{
  double x;
  double y;
  double z;
  double heading;

  double dist(Vector4d va) {
    return sqrt(pow(va.x - x, 2) + pow(va.y - y, 2) + pow(va.z - z, 2));
  }
};

class TypeConvertor {
public:
  TypeConvertor();
  ~TypeConvertor(void);

  static Eigen::Vector3d vector4dToEigen3d(const Vector4d& vector4d);
  static Vector4d eigen3dToVector4d(const Eigen::Vector3d& eigen_vec);
  static double vector4dDist(const Vector4d& a, const Vector4d& b);
  static std::vector<Vector4d> trajectoryE3dToVector4d(const std::vector<Eigen::Vector3d>& trajectory_3d);
};

}  // namespace catora_planner

#endif
