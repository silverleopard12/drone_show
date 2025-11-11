#ifndef ORCA_PLANNER_SPATIAL_HASH_3D_HPP
#define ORCA_PLANNER_SPATIAL_HASH_3D_HPP

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <memory>

namespace orca_planner {

/**
 * @brief 3D spatial hash for efficient neighbor queries
 *
 * Divides 3D space into a grid and stores agents in corresponding cells.
 * Enables O(1) neighbor queries instead of O(n).
 */
class SpatialHash3D {
public:
  using Ptr = std::shared_ptr<SpatialHash3D>;

  /**
   * @brief Constructor
   * @param cell_size Size of each grid cell (should be >= neighbor_dist)
   */
  explicit SpatialHash3D(double cell_size = 5.0);
  ~SpatialHash3D();

  /**
   * @brief Clear all agents from hash
   */
  void clear();

  /**
   * @brief Insert agent into spatial hash
   * @param agent_id Agent identifier
   * @param position 3D position
   */
  void insert(int agent_id, const Eigen::Vector3d& position);

  /**
   * @brief Find all agents within radius of position
   * @param position Query position
   * @param radius Search radius
   * @return Vector of agent IDs
   */
  std::vector<int> findNearby(
    const Eigen::Vector3d& position,
    double radius
  ) const;

  /**
   * @brief Find k nearest neighbors
   * @param position Query position
   * @param k Number of neighbors
   * @return Vector of agent IDs sorted by distance
   */
  std::vector<int> findKNearest(
    const Eigen::Vector3d& position,
    int k
  ) const;

  /**
   * @brief Get statistics
   */
  size_t getAgentCount() const { return agent_positions_.size(); }
  size_t getCellCount() const { return grid_.size(); }
  double getCellSize() const { return cell_size_; }

private:
  double cell_size_;

  // Hash key for a 3D cell
  struct CellKey {
    int x, y, z;

    CellKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const CellKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  // Hash function for CellKey
  struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
      // Simple hash combining x, y, z
      return std::hash<int>()(k.x) ^
             (std::hash<int>()(k.y) << 1) ^
             (std::hash<int>()(k.z) << 2);
    }
  };

  // Grid: maps cell key to list of agent IDs in that cell
  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> grid_;

  // Agent positions for distance calculations
  std::unordered_map<int, Eigen::Vector3d> agent_positions_;

  /**
   * @brief Compute cell key for a position
   */
  CellKey getCellKey(const Eigen::Vector3d& position) const;

  /**
   * @brief Get all cells within radius of a cell
   */
  std::vector<CellKey> getCellsInRadius(
    const CellKey& center,
    double radius
  ) const;
};

} // namespace orca_planner

#endif // ORCA_PLANNER_SPATIAL_HASH_3D_HPP
