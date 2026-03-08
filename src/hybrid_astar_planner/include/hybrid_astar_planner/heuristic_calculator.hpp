#ifndef HYBRID_ASTAR_PLANNER__HEURISTIC_CALCULATOR_HPP_
#define HYBRID_ASTAR_PLANNER__HEURISTIC_CALCULATOR_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/logger.hpp>
#include <vector>
#include <memory>

namespace hybrid_astar_planner
{

// Forward declaration
class AnalyticExpander;

/// Heuristic calculator supporting multiple admissible heuristics
/// - euclidean: Simple Euclidean distance
/// - reeds_shepp: Non-holonomic RS distance (ignores obstacles)
/// - dijkstra: Cost-aware 2D Dijkstra (considers obstacles)
/// - max: max(reeds_shepp, dijkstra) — tightest admissible bound (recommended)
class HeuristicCalculator
{
public:
  HeuristicCalculator() = default;

  void initialize(
    const HeuristicConfig & config,
    nav2_costmap_2d::Costmap2D * costmap,
    const AnalyticExpander * analytic_expander,
    double turning_radius,
    const rclcpp::Logger & logger = rclcpp::get_logger("heuristic_calculator"));

  /// Update costmap reference (cheaper than full re-initialization)
  void updateCostmap(nav2_costmap_2d::Costmap2D * costmap);

  /// Compute heuristic value from state to goal
  double compute(const State & state, const State & goal) const;

  /// Precompute 2D Dijkstra distance map from goal (call once per planning query)
  void precomputeFromGoal(const State & goal);

private:
  double euclideanDistance(const State & state, const State & goal) const;
  double reedSheppDistance(const State & state, const State & goal) const;
  double dijkstraDistance(const State & state, const State & goal) const;

  /// Compute cost-aware 2D Dijkstra from goal pixel to all cells
  void compute2DDijkstra(unsigned int goal_mx, unsigned int goal_my);

  HeuristicConfig config_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  const AnalyticExpander * analytic_expander_ = nullptr;
  double turning_radius_ = 0.0;
  rclcpp::Logger logger_{rclcpp::get_logger("heuristic_calculator")};

  // 2D Dijkstra distance map (flattened row-major)
  std::vector<float> distance_map_;
  unsigned int map_size_x_ = 0;
  unsigned int map_size_y_ = 0;
  bool dijkstra_computed_ = false;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__HEURISTIC_CALCULATOR_HPP_
