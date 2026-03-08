#ifndef HYBRID_ASTAR_PLANNER__SEARCH_ENGINE_HPP_
#define HYBRID_ASTAR_PLANNER__SEARCH_ENGINE_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/motion_model.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/heuristic_calculator.hpp"
#include "hybrid_astar_planner/analytic_expander.hpp"
#include "hybrid_astar_planner/cost_function.hpp"
#include "hybrid_astar_planner/path_smoother.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <rclcpp/logger.hpp>
#include <vector>
#include <optional>
#include <memory>
#include <queue>
#include <unordered_map>

namespace hybrid_astar_planner
{

/// Core Hybrid A* search engine
/// Manages the A* search loop, node expansion, analytical expansion,
/// and path reconstruction
class SearchEngine
{
public:
  SearchEngine() = default;

  void initialize(
    const SearchConfig & search_config,
    const GoalConfig & goal_config,
    MotionModel * motion_model,
    CollisionChecker * collision_checker,
    HeuristicCalculator * heuristic,
    AnalyticExpander * analytic_expander,
    CostFunction * cost_function,
    PathSmoother * path_smoother,
    const rclcpp::Logger & logger);

  /// Plan a path from start to goal
  /// Returns (path, stats) if successful, std::nullopt if no path found
  std::optional<std::pair<std::vector<State>, PlanningStats>> plan(
    const State & start,
    const State & goal);

private:
  /// Generate neighbor states using motion primitives
  std::vector<State *> getNeighbors(State * current, const State & goal);

  /// Reconstruct path by following parent pointers
  std::vector<State> reconstructPath(const State * state) const;

  /// Compute path length
  double computePathLength(const std::vector<State> & path) const;

  /// Compute minimum clearance to obstacles
  double computeMinClearance(const std::vector<State> & path) const;

  /// Compute curvature metric (sum of absolute heading changes)
  double computeCurvatureMetric(const std::vector<State> & path) const;

  /// Smooth path with post-smoothing collision safety check
  void smoothPathSafely(std::vector<State> & path);

  /// Interpolate path with sub-steps for dense output
  std::vector<State> interpolatePath(
    const std::vector<State> & path, int substeps = 5) const;

  /// Allocate a new node from the pool
  State * allocateNode();

  /// Reset the node pool for a new search
  void resetPool();

  // Configuration
  SearchConfig search_config_;
  GoalConfig goal_config_;
  double theta_resolution_rad_ = 0.087;

  // Components (non-owning)
  MotionModel * motion_model_ = nullptr;
  CollisionChecker * collision_checker_ = nullptr;
  HeuristicCalculator * heuristic_ = nullptr;
  AnalyticExpander * analytic_expander_ = nullptr;
  CostFunction * cost_function_ = nullptr;
  PathSmoother * path_smoother_ = nullptr;

  // Node pool to avoid heap allocations during search
  std::vector<std::unique_ptr<State>> node_pool_;
  size_t pool_index_ = 0;
  static constexpr size_t INITIAL_POOL_SIZE = 50000;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("search_engine")};
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__SEARCH_ENGINE_HPP_
