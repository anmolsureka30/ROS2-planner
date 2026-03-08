#include "hybrid_astar_planner/cost_function.hpp"
#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

void CostFunction::initialize(
  const CostConfig & cost_config,
  const VehicleConfig & vehicle_config,
  double step_size)
{
  config_ = cost_config;
  max_steering_angle_ = vehicle_config.max_steering_angle;
  step_size_ = step_size;
}

double CostFunction::computeCost(
  const State & current,
  const State & neighbor,
  double costmap_cost) const
{
  double cost = step_size_;

  // Steering magnitude penalty
  double steer_ratio = std::abs(neighbor.steering) / max_steering_angle_;
  cost *= (1.0 + config_.steering_penalty * steer_ratio);

  // Reversing penalty
  if (neighbor.direction == Direction::REVERSE) {
    cost *= config_.reversing_penalty;
  }

  // Smac Planner traversal penalties (Eq. 2): 3-case logic
  // Case 1: Curved + turn direction changed → (1 + β + γ)
  // Case 2: Curved + same turn direction → (1 + β)
  // Case 3: Starting turn from straight → (1 + β)
  if (std::abs(neighbor.steering) > 1e-3) {
    if (std::abs(current.steering) > 1e-3) {
      bool sign_changed = (current.steering > 0) != (neighbor.steering > 0);
      if (sign_changed) {
        cost *= (1.0 + config_.non_straight_penalty + config_.change_direction_penalty);
      } else {
        cost *= (1.0 + config_.non_straight_penalty);
      }
    } else {
      cost *= (1.0 + config_.non_straight_penalty);
    }
  }

  // Steering change penalty (smoothness) — only when not first expansion from start
  if (current.parent != nullptr) {
    double steer_change = std::abs(neighbor.steering - current.steering) / max_steering_angle_;
    cost *= (1.0 + config_.steering_change_penalty * steer_change);
  }

  // Cost-aware traversal multiplier (Smac Planner Eq. 1)
  if (config_.cost_penalty_alpha > 0.0 && costmap_cost > 0.0) {
    cost *= (1.0 + config_.cost_penalty_alpha * costmap_cost);
  }

  // Direction switch penalty (forward <-> reverse transition)
  if (current.direction != neighbor.direction) {
    cost += config_.direction_switch_penalty;
  }

  return cost;
}

}  // namespace hybrid_astar_planner
