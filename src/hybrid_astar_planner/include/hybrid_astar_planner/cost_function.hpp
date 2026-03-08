#ifndef HYBRID_ASTAR_PLANNER__COST_FUNCTION_HPP_
#define HYBRID_ASTAR_PLANNER__COST_FUNCTION_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/types.hpp"

namespace hybrid_astar_planner
{

/// Multi-component cost function based on Smac Planner formulation
/// Computes the traversal cost between two states considering:
/// - Steering magnitude and changes
/// - Reversing penalty
/// - Direction switching penalty
/// - Non-straight motion penalty
/// - Cost-aware traversal (obstacle proximity)
class CostFunction
{
public:
  CostFunction() = default;

  void initialize(
    const CostConfig & cost_config,
    const VehicleConfig & vehicle_config,
    double step_size);

  /// Compute motion cost from current to neighbor state
  /// costmap_cost: normalized obstacle proximity cost [0, 1] at neighbor position
  double computeCost(
    const State & current,
    const State & neighbor,
    double costmap_cost = 0.0) const;

private:
  CostConfig config_;
  double max_steering_angle_ = 0.0;
  double step_size_ = 0.0;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__COST_FUNCTION_HPP_
