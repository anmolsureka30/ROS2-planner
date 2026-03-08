#include "hybrid_astar_planner/search_engine.hpp"

#include <rclcpp/logging.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

void SearchEngine::initialize(
  const SearchConfig & search_config,
  const GoalConfig & goal_config,
  MotionModel * motion_model,
  CollisionChecker * collision_checker,
  HeuristicCalculator * heuristic,
  AnalyticExpander * analytic_expander,
  CostFunction * cost_function,
  PathSmoother * path_smoother,
  const rclcpp::Logger & logger)
{
  search_config_ = search_config;
  goal_config_ = goal_config;
  theta_resolution_rad_ = search_config_.theta_resolution * M_PI / 180.0;

  motion_model_ = motion_model;
  collision_checker_ = collision_checker;
  heuristic_ = heuristic;
  analytic_expander_ = analytic_expander;
  cost_function_ = cost_function;
  path_smoother_ = path_smoother;
  logger_ = logger;

  // Pre-allocate node pool
  node_pool_.reserve(INITIAL_POOL_SIZE);
  for (size_t i = 0; i < INITIAL_POOL_SIZE; ++i) {
    node_pool_.push_back(std::make_unique<State>());
  }
}

std::optional<std::pair<std::vector<State>, PlanningStats>> SearchEngine::plan(
  const State & start,
  const State & goal)
{
  auto plan_start = std::chrono::steady_clock::now();

  PlanningStats stats;
  resetPool();
  collision_checker_->clearCache();

  // Validate start and goal
  State start_state = start;
  if (!collision_checker_->isValid(start_state)) {
    RCLCPP_ERROR(logger_, "Start state is in collision!");
    return std::nullopt;
  }

  State goal_state = goal;
  if (!collision_checker_->isValid(goal_state)) {
    RCLCPP_ERROR(logger_, "Goal state is in collision!");
    return std::nullopt;
  }

  // Precompute 2D heuristic from goal
  heuristic_->precomputeFromGoal(goal);

  // Initialize start node
  State * start_node = allocateNode();
  *start_node = start;
  start_node->g = 0.0;
  start_node->h = heuristic_->compute(*start_node, goal);
  start_node->parent = nullptr;

  // Open set (min-heap on f-value)
  std::priority_queue<State *, std::vector<State *>, StateCompare> open_set;
  open_set.push(start_node);

  // Closed set — pre-size for expected capacity
  std::unordered_map<DiscreteState, State *, DiscreteStateHash> closed_set;
  closed_set.reserve(INITIAL_POOL_SIZE);

  // Track best state near goal
  State * best_near_goal = nullptr;
  double min_dist = std::numeric_limits<double>::infinity();

  // Analytical expansion control
  int shot_check_counter = 0;
  static constexpr int SHOT_CHECK_EVERY = 20;

  const double xy_tol = goal_config_.xy_tolerance;
  const double theta_tol = goal_config_.theta_tolerance * M_PI / 180.0;
  const double xy_res = search_config_.xy_resolution;
  const double theta_res = theta_resolution_rad_;
  const double shot_distance = search_config_.shot_distance;

  // Cache motion primitives reference to avoid repeated virtual calls
  const auto & primitives = motion_model_->getPrimitives();
  const int num_primitives = static_cast<int>(primitives.size());

  int iterations = 0;

  // Pre-allocate timeout check interval (avoid chrono call every iteration)
  static constexpr int TIMEOUT_CHECK_EVERY = 500;

  while (!open_set.empty()) {
    State * current = open_set.top();
    open_set.pop();
    iterations++;

    // Timeout check — only every N iterations to reduce chrono overhead
    if (iterations % TIMEOUT_CHECK_EVERY == 0) {
      double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - plan_start).count();
      if (elapsed > search_config_.timeout) {
        RCLCPP_WARN(logger_, "Planning timeout (%.1fs) reached after %d iterations",
          search_config_.timeout, iterations);
        break;
      }
    }

    // Discretize current for closed-set lookup
    DiscreteState current_idx = discretize(*current, xy_res, theta_res);

    // Closed-set duplicate check — skip if a better node already exists
    auto closed_it = closed_set.find(current_idx);
    if (closed_it != closed_set.end()) {
      if (closed_it->second->g <= current->g) {
        continue;
      }
    }
    closed_set[current_idx] = current;

    double dist_to_goal = current->distanceTo(goal);

    // Progress reporting
    if (iterations % 5000 == 0) {
      double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - plan_start).count();
      RCLCPP_INFO(logger_, "  Nodes: %d, best_dist: %.2fm, time: %.2fs",
        iterations, min_dist, elapsed);
    }

    // Track best state near goal
    if (dist_to_goal < min_dist) {
      min_dist = dist_to_goal;
      best_near_goal = current;
    }

    // === EXACT GOAL CHECK ===
    double heading_diff = std::abs(angleDiff(current->theta, goal.theta));
    if (dist_to_goal < xy_tol && heading_diff < theta_tol) {
      double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - plan_start).count();
      RCLCPP_INFO(logger_, "Exact goal reached! dist=%.3fm, heading_err=%.1fdeg",
        dist_to_goal, heading_diff * 180.0 / M_PI);

      std::vector<State> path = reconstructPath(current);
      path.push_back(goal);
      smoothPathSafely(path);

      stats.nodes_expanded = iterations;
      stats.nodes_visited = static_cast<int>(closed_set.size());
      stats.search_time = elapsed;
      stats.path_length = computePathLength(path);
      stats.min_clearance = computeMinClearance(path);
      stats.curvature_sum = computeCurvatureMetric(path);
      stats.success = true;
      stats.analytical_expansion = false;

      return std::make_pair(path, stats);
    }

    // === ANALYTICAL EXPANSION (Reeds-Shepp shot) ===
    static constexpr double ALWAYS_TRY_SHOT_DISTANCE = 3.0;  // meters
    bool should_try_shot = false;
    if (dist_to_goal < ALWAYS_TRY_SHOT_DISTANCE) {
      should_try_shot = true;
    } else if (dist_to_goal < shot_distance) {
      shot_check_counter++;
      if (shot_check_counter >= SHOT_CHECK_EVERY) {
        shot_check_counter = 0;
        should_try_shot = true;
      }
    }

    if (should_try_shot && analytic_expander_) {
      auto rs_result = analytic_expander_->tryExpansion(*current, goal, *collision_checker_);
      if (rs_result.has_value()) {
        double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - plan_start).count();
        RCLCPP_INFO(logger_, "Analytical shot! dist=%.2fm", dist_to_goal);

        std::vector<State> path = reconstructPath(current);
        auto & rs_path = rs_result.value();
        if (rs_path.size() > 1) {
          path.insert(path.end(), rs_path.begin() + 1, rs_path.end());
        }
        smoothPathSafely(path);

        stats.nodes_expanded = iterations;
        stats.nodes_visited = static_cast<int>(closed_set.size());
        stats.search_time = elapsed;
        stats.path_length = computePathLength(path);
        stats.min_clearance = computeMinClearance(path);
        stats.curvature_sum = computeCurvatureMetric(path);
        stats.success = true;
        stats.analytical_expansion = true;

        return std::make_pair(path, stats);
      }
    }

    // === EXPANSION ===
    // Inline neighbor generation to avoid vector allocation overhead
    for (int p = 0; p < num_primitives; ++p) {
      const auto & prim = primitives[p];
      State result = motion_model_->applyMotion(*current, prim.direction, prim.steering_angle);

      // Early collision check before allocating a node
      if (!collision_checker_->isValid(result)) {
        continue;
      }

      DiscreteState neighbor_idx = discretize(result, xy_res, theta_res);

      // Compute cost
      double costmap_cost = collision_checker_->getTraversalCost(result.x, result.y);
      double motion_cost = cost_function_->computeCost(*current, result, costmap_cost);
      double new_g = current->g + motion_cost;

      // Check closed set — skip if existing node is better
      auto closed_check = closed_set.find(neighbor_idx);
      if (closed_check != closed_set.end()) {
        if (closed_check->second->g <= new_g) {
          continue;
        }
      }

      // Allocate node only after all checks pass
      State * neighbor = allocateNode();
      *neighbor = result;
      neighbor->g = new_g;
      neighbor->h = heuristic_->compute(*neighbor, goal);
      neighbor->parent = current;

      open_set.push(neighbor);
    }

    // Iteration limit
    if (iterations > search_config_.max_iterations) {
      RCLCPP_WARN(logger_, "Max iterations (%d) reached!", search_config_.max_iterations);
      break;
    }
  }

  // Fallback: try RS from best near-goal state
  double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - plan_start).count();

  RCLCPP_INFO(logger_, "Search finished. Iterations: %d, best dist: %.2fm, time: %.2fs",
    iterations, min_dist, elapsed);

  if (best_near_goal && min_dist < search_config_.shot_distance) {
    if (analytic_expander_) {
      auto rs_result = analytic_expander_->tryExpansion(
        *best_near_goal, goal, *collision_checker_);
      if (rs_result.has_value()) {
        std::vector<State> path = reconstructPath(best_near_goal);
        auto & rs_path = rs_result.value();
        if (rs_path.size() > 1) {
          path.insert(path.end(), rs_path.begin() + 1, rs_path.end());
        }

        smoothPathSafely(path);

        stats.nodes_expanded = iterations;
        stats.nodes_visited = static_cast<int>(closed_set.size());
        stats.search_time = elapsed;
        stats.path_length = computePathLength(path);
        stats.min_clearance = computeMinClearance(path);
        stats.curvature_sum = computeCurvatureMetric(path);
        stats.success = true;
        stats.analytical_expansion = true;

        return std::make_pair(path, stats);
      }
    }

    // Return partial path to best near-goal state
    std::vector<State> path = reconstructPath(best_near_goal);
    smoothPathSafely(path);

    stats.nodes_expanded = iterations;
    stats.nodes_visited = static_cast<int>(closed_set.size());
    stats.search_time = elapsed;
    stats.path_length = computePathLength(path);
    stats.min_clearance = computeMinClearance(path);
    stats.curvature_sum = computeCurvatureMetric(path);
    stats.success = false;

    return std::make_pair(path, stats);
  }

  return std::nullopt;
}

std::vector<State> SearchEngine::reconstructPath(const State * state) const
{
  std::vector<State> path;
  const State * current = state;

  while (current != nullptr) {
    path.push_back(*current);
    current = current->parent;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

double SearchEngine::computePathLength(const std::vector<State> & path) const
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

double SearchEngine::computeMinClearance(const std::vector<State> & path) const
{
  double min_clearance = std::numeric_limits<double>::infinity();
  int step = std::max(1, static_cast<int>(path.size()) / 50);

  for (size_t i = 0; i < path.size(); i += step) {
    double clearance = collision_checker_->getObstacleDistance(path[i].x, path[i].y);
    min_clearance = std::min(min_clearance, clearance);
  }

  return min_clearance;
}

double SearchEngine::computeCurvatureMetric(const std::vector<State> & path) const
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += std::abs(angleDiff(path[i].theta, path[i - 1].theta));
  }
  return total;
}

void SearchEngine::smoothPathSafely(std::vector<State> & path)
{
  if (!path_smoother_) {
    return;
  }
  auto smoothed = path_smoother_->smooth(path, *collision_checker_);
  if (collision_checker_->isPathValid(smoothed)) {
    path = smoothed;
  } else {
    RCLCPP_WARN(logger_, "Smoothed path has collisions, using unsmoothed path");
  }
}

std::vector<State> SearchEngine::interpolatePath(
  const std::vector<State> & path, int substeps) const
{
  if (path.size() < 2 || substeps < 1) {
    return path;
  }

  std::vector<State> dense;
  dense.reserve(path.size() * substeps);
  dense.push_back(path[0]);

  for (size_t i = 1; i < path.size(); ++i) {
    for (int j = 1; j <= substeps; ++j) {
      double t = static_cast<double>(j) / substeps;
      State interp;
      interp.x = path[i - 1].x + t * (path[i].x - path[i - 1].x);
      interp.y = path[i - 1].y + t * (path[i].y - path[i - 1].y);
      interp.theta = path[i - 1].theta + t * angleDiff(path[i].theta, path[i - 1].theta);
      interp.theta = normalizeAngle(interp.theta);
      interp.direction = path[i].direction;
      interp.steering = path[i].steering;
      dense.push_back(interp);
    }
  }

  return dense;
}

State * SearchEngine::allocateNode()
{
  if (pool_index_ >= node_pool_.size()) {
    // Grow the pool
    size_t new_size = node_pool_.size() * 2;
    node_pool_.reserve(new_size);
    for (size_t i = node_pool_.size(); i < new_size; ++i) {
      node_pool_.push_back(std::make_unique<State>());
    }
  }

  State * node = node_pool_[pool_index_].get();
  node->index = static_cast<int>(pool_index_);
  pool_index_++;
  return node;
}

void SearchEngine::resetPool()
{
  // Only reset up to the number of nodes actually used last time
  size_t reset_count = std::min(pool_index_, node_pool_.size());
  for (size_t i = 0; i < reset_count; ++i) {
    node_pool_[i]->parent = nullptr;
    node_pool_[i]->g = 0.0;
    node_pool_[i]->h = 0.0;
  }
  pool_index_ = 0;
}

}  // namespace hybrid_astar_planner
