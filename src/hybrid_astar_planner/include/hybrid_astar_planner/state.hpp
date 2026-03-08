#ifndef HYBRID_ASTAR_PLANNER__STATE_HPP_
#define HYBRID_ASTAR_PLANNER__STATE_HPP_

#include "hybrid_astar_planner/types.hpp"
#include <functional>
#include <cmath>

namespace hybrid_astar_planner
{

/// Continuous SE(2) state for Hybrid A* search
struct State
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;      // heading in radians [-pi, pi]

  double g = 0.0;           // cost-so-far
  double h = 0.0;           // heuristic estimate

  Direction direction = Direction::FORWARD;
  double steering = 0.0;    // steering angle used to reach this state

  State * parent = nullptr; // for path reconstruction (non-owning)
  int index = -1;           // index in node pool

  double f() const { return g + h; }

  double distanceTo(const State & other) const
  {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::hypot(dx, dy);
  }

  State() = default;

  State(double x_, double y_, double theta_)
  : x(x_), y(y_), theta(normalizeAngle(theta_)) {}

  State(double x_, double y_, double theta_, double g_, double h_)
  : x(x_), y(y_), theta(normalizeAngle(theta_)), g(g_), h(h_) {}
};

/// Discretized state for closed set indexing
struct DiscreteState
{
  int ix = 0;
  int iy = 0;
  int itheta = 0;

  bool operator==(const DiscreteState & other) const
  {
    return ix == other.ix && iy == other.iy && itheta == other.itheta;
  }
};

/// Hash function for DiscreteState
struct DiscreteStateHash
{
  std::size_t operator()(const DiscreteState & s) const
  {
    // Pack into a single uint64 for fast hashing
    std::size_t h = 0;
    h ^= std::hash<int>()(s.ix) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(s.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(s.itheta) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

/// Discretize a continuous state to grid indices
inline DiscreteState discretize(
  const State & state,
  double xy_resolution,
  double theta_resolution_rad)
{
  int num_theta_bins = static_cast<int>(std::round(2.0 * M_PI / theta_resolution_rad));
  if (num_theta_bins <= 0) { num_theta_bins = 72; }

  double normalized_theta = state.theta;
  while (normalized_theta < 0) { normalized_theta += 2.0 * M_PI; }
  while (normalized_theta >= 2.0 * M_PI) { normalized_theta -= 2.0 * M_PI; }

  return DiscreteState{
    static_cast<int>(std::floor(state.x / xy_resolution)),
    static_cast<int>(std::floor(state.y / xy_resolution)),
    static_cast<int>(std::floor(normalized_theta / theta_resolution_rad)) % num_theta_bins
  };
}

/// Comparator for priority queue (min-heap on f-value)
struct StateCompare
{
  bool operator()(const State * a, const State * b) const
  {
    return a->f() > b->f();  // min-heap: smaller f on top
  }
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__STATE_HPP_
