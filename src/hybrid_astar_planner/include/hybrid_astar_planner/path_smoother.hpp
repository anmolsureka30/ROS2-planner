#ifndef HYBRID_ASTAR_PLANNER__PATH_SMOOTHER_HPP_
#define HYBRID_ASTAR_PLANNER__PATH_SMOOTHER_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <vector>

namespace hybrid_astar_planner
{

// Forward declaration
class CollisionChecker;

/// Gradient descent path smoother
/// Minimizes multi-objective cost: smoothness + obstacle repulsion + original path fidelity
class PathSmoother
{
public:
  PathSmoother() = default;

  void initialize(const SmootherConfig & config);

  /// Smooth a path, keeping start and end states fixed
  /// Returns the smoothed path
  std::vector<State> smooth(
    const std::vector<State> & path,
    CollisionChecker & collision_checker) const;

private:
  SmootherConfig config_;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__PATH_SMOOTHER_HPP_
