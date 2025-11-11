#include "orca_planner/spatial_hash/spatial_hash_3d.hpp"
#include <algorithm>
#include <cmath>

namespace orca_planner {

SpatialHash3D::SpatialHash3D(double cell_size)
  : cell_size_(cell_size) {
}

SpatialHash3D::~SpatialHash3D() {
}

void SpatialHash3D::clear() {
  grid_.clear();
  agent_positions_.clear();
}

void SpatialHash3D::insert(int agent_id, const Eigen::Vector3d& position) {
  // Store position
  agent_positions_[agent_id] = position;

  // Get cell key
  CellKey key = getCellKey(position);

  // Insert into grid
  grid_[key].push_back(agent_id);
}

std::vector<int> SpatialHash3D::findNearby(
    const Eigen::Vector3d& position,
    double radius) const {

  std::vector<int> nearby;

  // Get cells to check
  CellKey center_key = getCellKey(position);
  std::vector<CellKey> cells_to_check = getCellsInRadius(center_key, radius);

  // Collect all agents in those cells
  for (const auto& cell_key : cells_to_check) {
    auto it = grid_.find(cell_key);
    if (it != grid_.end()) {
      for (int agent_id : it->second) {
        // Check actual distance
        auto pos_it = agent_positions_.find(agent_id);
        if (pos_it != agent_positions_.end()) {
          double dist = (pos_it->second - position).norm();
          if (dist <= radius) {
            nearby.push_back(agent_id);
          }
        }
      }
    }
  }

  return nearby;
}

std::vector<int> SpatialHash3D::findKNearest(
    const Eigen::Vector3d& position,
    int k) const {

  // Start with a reasonable radius
  double radius = cell_size_ * 2.0;

  std::vector<int> candidates;

  // Expand search until we have enough candidates
  for (int expansion = 0; expansion < 5; ++expansion) {
    candidates = findNearby(position, radius);

    if (static_cast<int>(candidates.size()) >= k) {
      break;
    }

    radius *= 2.0;  // Double search radius
  }

  // Sort by distance
  std::sort(candidates.begin(), candidates.end(),
    [this, &position](int a, int b) {
      double dist_a = (agent_positions_.at(a) - position).norm();
      double dist_b = (agent_positions_.at(b) - position).norm();
      return dist_a < dist_b;
    });

  // Return k nearest
  if (static_cast<int>(candidates.size()) > k) {
    candidates.resize(k);
  }

  return candidates;
}

SpatialHash3D::CellKey SpatialHash3D::getCellKey(
    const Eigen::Vector3d& position) const {

  int x = static_cast<int>(std::floor(position.x() / cell_size_));
  int y = static_cast<int>(std::floor(position.y() / cell_size_));
  int z = static_cast<int>(std::floor(position.z() / cell_size_));

  return CellKey(x, y, z);
}

std::vector<SpatialHash3D::CellKey> SpatialHash3D::getCellsInRadius(
    const CellKey& center,
    double radius) const {

  std::vector<CellKey> cells;

  // Number of cells to check in each direction
  int cell_range = static_cast<int>(std::ceil(radius / cell_size_));

  for (int dx = -cell_range; dx <= cell_range; ++dx) {
    for (int dy = -cell_range; dy <= cell_range; ++dy) {
      for (int dz = -cell_range; dz <= cell_range; ++dz) {
        cells.push_back(CellKey(
          center.x + dx,
          center.y + dy,
          center.z + dz
        ));
      }
    }
  }

  return cells;
}

} // namespace orca_planner
