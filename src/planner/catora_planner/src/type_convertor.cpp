#include "catora_planner/type_convertor.hpp"

using namespace catora_planner;

TypeConvertor::TypeConvertor() {
}

TypeConvertor::~TypeConvertor(void) {
}

Eigen::Vector3d TypeConvertor::vector4dToEigen3d(const Vector4d& vector4d) {
  return Eigen::Vector3d(vector4d.x, vector4d.y, vector4d.z);
}

Vector4d TypeConvertor::eigen3dToVector4d(const Eigen::Vector3d& eigen_vec) {
  Vector4d v;
  v.x       = eigen_vec(0);
  v.y       = eigen_vec(1);
  v.z       = eigen_vec(2);
  v.heading = 0.0;
  return v;
}

double TypeConvertor::vector4dDist(const Vector4d& a, const Vector4d& b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

std::vector<Vector4d> TypeConvertor::trajectoryE3dToVector4d(const std::vector<Eigen::Vector3d>& trajectory_3d) {
  std::vector<Vector4d> trajectory_4d;
  for (auto& pos : trajectory_3d) {
    trajectory_4d.push_back(eigen3dToVector4d(pos));
  }
  return trajectory_4d;
}
