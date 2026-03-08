#include "hybrid_astar_planner/analytic_expander.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"

#include <ompl/base/ScopedState.h>
#include <cmath>

namespace hybrid_astar_planner
{

void AnalyticExpander::initialize(double turning_radius, double step_size)
{
  turning_radius_ = turning_radius;
  step_size_ = step_size;

  // Create OMPL Reeds-Shepp state space with given turning radius
  rs_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(turning_radius);
}

std::optional<std::vector<State>> AnalyticExpander::tryExpansion(
  const State & current,
  const State & goal,
  CollisionChecker & collision_checker) const
{
  double path_length = getDistance(current, goal);

  if (path_length < 1e-6 || std::isinf(path_length)) {
    return std::nullopt;
  }

  // Discretize the Reeds-Shepp path
  std::vector<State> path = discretizePath(current, goal, path_length);

  if (path.size() < 2) {
    return std::nullopt;
  }

  // Check collision along the path
  if (!collision_checker.isPathValid(path)) {
    return std::nullopt;
  }

  return path;
}

double AnalyticExpander::getDistance(const State & from, const State & to) const
{
  ompl::base::ScopedState<ompl::base::SE2StateSpace> s1(rs_space_);
  ompl::base::ScopedState<ompl::base::SE2StateSpace> s2(rs_space_);

  s1->setX(from.x);
  s1->setY(from.y);
  s1->setYaw(from.theta);

  s2->setX(to.x);
  s2->setY(to.y);
  s2->setYaw(to.theta);

  return rs_space_->distance(s1.get(), s2.get());
}

std::vector<State> AnalyticExpander::discretizePath(
  const State & start,
  const State & goal,
  double path_length) const
{
  std::vector<State> path;

  ompl::base::ScopedState<ompl::base::SE2StateSpace> s1(rs_space_);
  ompl::base::ScopedState<ompl::base::SE2StateSpace> s2(rs_space_);
  ompl::base::ScopedState<ompl::base::SE2StateSpace> interp(rs_space_);

  s1->setX(start.x);
  s1->setY(start.y);
  s1->setYaw(start.theta);

  s2->setX(goal.x);
  s2->setY(goal.y);
  s2->setYaw(goal.theta);

  // Number of interpolation steps
  int num_steps = std::max(static_cast<int>(std::ceil(path_length / step_size_)), 2);

  path.reserve(num_steps + 1);

  for (int i = 0; i <= num_steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_steps);
    rs_space_->interpolate(s1.get(), s2.get(), t, interp.get());

    State state(interp->getX(), interp->getY(), interp->getYaw());

    // Determine direction using heading-vs-movement comparison
    // More robust than dot product: handles cusps and tiny displacements
    if (i > 0) {
      double dx = state.x - path.back().x;
      double dy = state.y - path.back().y;
      double dist = std::hypot(dx, dy);
      if (dist > 1e-6) {
        double move_heading = std::atan2(dy, dx);
        double heading_diff = std::abs(angleDiff(move_heading, path.back().theta));
        state.direction = (heading_diff < M_PI / 2.0) ? Direction::FORWARD : Direction::REVERSE;
      } else {
        // Near-zero displacement: inherit previous direction
        state.direction = path.back().direction;
      }
    } else {
      state.direction = start.direction;
    }

    path.push_back(state);
  }

  return path;
}

}  // namespace hybrid_astar_planner
