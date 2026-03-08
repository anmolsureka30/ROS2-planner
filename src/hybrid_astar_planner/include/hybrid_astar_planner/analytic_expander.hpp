#ifndef HYBRID_ASTAR_PLANNER__ANALYTIC_EXPANDER_HPP_
#define HYBRID_ASTAR_PLANNER__ANALYTIC_EXPANDER_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <vector>
#include <optional>
#include <memory>

namespace hybrid_astar_planner
{

// Forward declaration
class CollisionChecker;

/// Analytical path expansion using OMPL Reeds-Shepp curves
class AnalyticExpander
{
public:
  AnalyticExpander() = default;

  void initialize(double turning_radius, double step_size);

  /// Attempt analytical expansion from current state to goal
  /// Returns path (including start and goal) if collision-free, std::nullopt otherwise
  std::optional<std::vector<State>> tryExpansion(
    const State & current,
    const State & goal,
    CollisionChecker & collision_checker) const;

  /// Compute the Reeds-Shepp distance between two states (no collision check)
  double getDistance(const State & from, const State & to) const;

  /// Get the turning radius
  double getTurningRadius() const { return turning_radius_; }

private:
  /// Discretize a Reeds-Shepp path into a sequence of states
  std::vector<State> discretizePath(
    const State & start,
    const State & goal,
    double path_length) const;

  double turning_radius_ = 0.0;
  double step_size_ = 0.01;

  std::shared_ptr<ompl::base::ReedsSheppStateSpace> rs_space_;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__ANALYTIC_EXPANDER_HPP_
