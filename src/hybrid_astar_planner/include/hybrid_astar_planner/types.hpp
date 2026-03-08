#ifndef HYBRID_ASTAR_PLANNER__TYPES_HPP_
#define HYBRID_ASTAR_PLANNER__TYPES_HPP_

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstdint>

namespace hybrid_astar_planner
{

/// Motion direction
enum class Direction : int8_t {
  FORWARD = 1,
  REVERSE = -1
};

/// Motion primitive: (direction, steering_angle)
struct MotionPrimitive {
  Direction direction;
  double steering_angle;  // radians
};

/// Vehicle dimensions (all in meters)
struct VehicleConfig {
  double wheelbase = 0.18;
  double length = 0.30;
  double width = 0.20;
  double rear_axle_to_back = 0.06;
  double max_steering_angle = M_PI / 6.0;  // 30 degrees
};

/// Search parameters
struct SearchConfig {
  double xy_resolution = 0.05;       // meters
  double theta_resolution = 5.0;     // degrees (stored as degrees, converted internally)
  double shot_distance = 1.5;        // meters
  int max_iterations = 100000;
  double timeout = 5.0;              // seconds
};

/// Cost function weights
struct CostConfig {
  double steering_penalty = 1.5;
  double reversing_penalty = 2.0;
  double steering_change_penalty = 1.5;
  double direction_switch_penalty = 10.0;
  double non_straight_penalty = 0.03;
  double change_direction_penalty = 0.05;
  double cost_penalty_alpha = 1.5;
};

/// Heuristic configuration
struct HeuristicConfig {
  std::string type = "max";  // euclidean | reeds_shepp | dijkstra | max
  double cost_alpha = 1.0;
};

/// Goal tolerance
struct GoalConfig {
  double xy_tolerance = 0.02;     // meters
  double theta_tolerance = 5.0;   // degrees
};

/// Smoother parameters
struct SmootherConfig {
  bool enabled = true;
  double w_smooth = 0.4;
  double w_obstacle = 0.3;
  double w_original = 0.3;
  double obstacle_margin = 0.15;   // meters
  int max_iterations = 200;
  double tolerance = 1e-4;
  double learning_rate = 0.5;
};

/// Motion model config
struct MotionConfig {
  double step_size = 0.05;        // meters
  int num_steering_angles = 5;
  bool allow_reverse = true;
};

/// Planning statistics
struct PlanningStats {
  int nodes_expanded = 0;
  int nodes_visited = 0;
  double search_time = 0.0;       // seconds
  double path_length = 0.0;       // meters
  double min_clearance = 0.0;     // meters
  double curvature_sum = 0.0;     // radians
  bool success = false;
  bool analytical_expansion = false;
};

/// Normalize angle to [-pi, pi] using O(1) fmod instead of while loops
inline double normalizeAngle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) { angle += 2.0 * M_PI; }
  return angle - M_PI;
}

/// Signed angle difference in [-pi, pi]
inline double angleDiff(double a, double b)
{
  return normalizeAngle(a - b);
}

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__TYPES_HPP_
