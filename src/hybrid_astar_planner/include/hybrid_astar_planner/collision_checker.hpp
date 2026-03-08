#ifndef HYBRID_ASTAR_PLANNER__COLLISION_CHECKER_HPP_
#define HYBRID_ASTAR_PLANNER__COLLISION_CHECKER_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/motion_model.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <unordered_map>
#include <cstdint>
#include <vector>

namespace hybrid_astar_planner
{

/// Footprint-based collision checker using Nav2 Costmap2D
class CollisionChecker
{
public:
  CollisionChecker() = default;

  void initialize(
    nav2_costmap_2d::Costmap2D * costmap,
    const MotionModel * motion_model,
    double xy_resolution,
    double theta_resolution_rad);

  /// Update costmap reference (cheaper than full re-initialization)
  void updateCostmap(nav2_costmap_2d::Costmap2D * costmap);

  /// Get costmap resolution in meters
  double getResolution() const { return resolution_; }

  /// Check if a state is collision-free (uses cache)
  bool isValid(const State & state);

  /// Check if a path (sequence of states) is collision-free
  bool isPathValid(const std::vector<State> & path);

  /// Get the normalized traversal cost at world coordinates [0, 1]
  /// 0 = far from obstacles, 1 = at obstacle
  double getTraversalCost(double x, double y) const;

  /// Get distance to nearest obstacle in meters
  double getObstacleDistance(double x, double y) const;

  /// Clear the collision cache
  void clearCache();

  /// Get costmap pointer
  nav2_costmap_2d::Costmap2D * getCostmap() const { return costmap_; }

private:
  /// Pack discrete state into uint64 key for cache
  uint64_t packKey(int ix, int iy, int itheta) const;

  /// Check if world coordinates are within costmap bounds
  bool isInBounds(double x, double y) const;

  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  const MotionModel * motion_model_ = nullptr;

  double xy_resolution_ = 0.05;
  double theta_resolution_rad_ = 0.087;  // ~5 degrees

  // Collision cache: packed (ix, iy, itheta) -> valid
  std::unordered_map<uint64_t, bool> cache_;

  // Costmap parameters
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__COLLISION_CHECKER_HPP_
