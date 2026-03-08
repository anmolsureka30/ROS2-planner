#include "hybrid_astar_planner/path_smoother.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"

#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

void PathSmoother::initialize(const SmootherConfig & config)
{
  config_ = config;
}

std::vector<State> PathSmoother::smooth(
  const std::vector<State> & path,
  CollisionChecker & collision_checker) const
{
  if (!config_.enabled || path.size() < 3) {
    return path;
  }

  // Work on a copy
  std::vector<State> smoothed = path;
  std::vector<State> original = path;  // keep original for reference

  double prev_change = std::numeric_limits<double>::max();

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    double total_change = 0.0;

    // Update interior points only (keep start and end fixed)
    for (size_t i = 1; i < smoothed.size() - 1; ++i) {
      double old_x = smoothed[i].x;
      double old_y = smoothed[i].y;

      // Smoothness gradient: move toward midpoint of neighbors
      // L_smooth = |p_{i-1} - 2p_i + p_{i+1}|^2
      double smooth_dx = smoothed[i - 1].x - 2.0 * smoothed[i].x + smoothed[i + 1].x;
      double smooth_dy = smoothed[i - 1].y - 2.0 * smoothed[i].y + smoothed[i + 1].y;

      // Original path fidelity gradient
      double orig_dx = original[i].x - smoothed[i].x;
      double orig_dy = original[i].y - smoothed[i].y;

      // Obstacle repulsion gradient
      double obs_dx = 0.0;
      double obs_dy = 0.0;
      double obs_dist = collision_checker.getObstacleDistance(smoothed[i].x, smoothed[i].y);

      if (obs_dist < config_.obstacle_margin && obs_dist > 1e-6) {
        // Compute repulsion direction using finite differences
        // Scale eps with costmap resolution for stability
        double eps = std::max(0.005, collision_checker.getResolution() * 0.5);
        double dist_px = collision_checker.getObstacleDistance(
          smoothed[i].x + eps, smoothed[i].y);
        double dist_py = collision_checker.getObstacleDistance(
          smoothed[i].x, smoothed[i].y + eps);

        // Gradient points toward increasing distance (away from obstacle)
        double grad_x = (dist_px - obs_dist) / eps;
        double grad_y = (dist_py - obs_dist) / eps;

        double repulsion_strength = (config_.obstacle_margin - obs_dist) / config_.obstacle_margin;
        obs_dx = grad_x * repulsion_strength;
        obs_dy = grad_y * repulsion_strength;
      }

      // Combined update
      double new_x = smoothed[i].x + config_.learning_rate * (
        config_.w_smooth * smooth_dx +
        config_.w_original * orig_dx +
        config_.w_obstacle * obs_dx);

      double new_y = smoothed[i].y + config_.learning_rate * (
        config_.w_smooth * smooth_dy +
        config_.w_original * orig_dy +
        config_.w_obstacle * obs_dy);

      // Safety check: reject update if it causes collision
      State test_state(new_x, new_y, smoothed[i].theta);
      if (collision_checker.isValid(test_state)) {
        smoothed[i].x = new_x;
        smoothed[i].y = new_y;
      }

      total_change += std::hypot(smoothed[i].x - old_x, smoothed[i].y - old_y);
    }

    // Convergence check
    if (total_change < config_.tolerance) {
      break;
    }

    if (std::abs(total_change - prev_change) < config_.tolerance * 0.1) {
      break;
    }
    prev_change = total_change;
  }

  // Update headings from path tangent (skip direction change boundaries)
  for (size_t i = 1; i < smoothed.size() - 1; ++i) {
    // Don't update heading at direction change boundaries — it causes discontinuities
    if (smoothed[i - 1].direction != smoothed[i + 1].direction) {
      continue;
    }

    double dx = smoothed[i + 1].x - smoothed[i - 1].x;
    double dy = smoothed[i + 1].y - smoothed[i - 1].y;
    if (std::hypot(dx, dy) > 1e-6) {
      smoothed[i].theta = std::atan2(dy, dx);
      // Preserve direction for reverse segments
      if (smoothed[i].direction == Direction::REVERSE) {
        smoothed[i].theta = normalizeAngle(smoothed[i].theta + M_PI);
      }
    }
  }

  return smoothed;
}

}  // namespace hybrid_astar_planner
