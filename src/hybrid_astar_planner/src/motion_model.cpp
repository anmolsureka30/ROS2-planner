#include "hybrid_astar_planner/motion_model.hpp"
#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

void MotionModel::initialize(const VehicleConfig & vehicle, const MotionConfig & motion)
{
  vehicle_ = vehicle;
  motion_ = motion;

  // Minimum turning radius: R = L / tan(delta_max)
  if (std::abs(vehicle_.max_steering_angle) > 1e-6) {
    min_turning_radius_ = vehicle_.wheelbase / std::tan(vehicle_.max_steering_angle);
  } else {
    min_turning_radius_ = 1e6;  // effectively infinite
  }

  generatePrimitives();
  generateFootprintTemplate();
}

void MotionModel::generatePrimitives()
{
  primitives_.clear();

  int n = motion_.num_steering_angles;
  double max_steer = vehicle_.max_steering_angle;

  // Generate evenly spaced steering angles
  std::vector<double> angles;
  if (n == 1) {
    angles.push_back(0.0);
  } else {
    for (int i = 0; i < n; ++i) {
      double angle = -max_steer + 2.0 * max_steer * i / (n - 1);
      angles.push_back(angle);
    }
  }

  // Forward primitives
  for (double angle : angles) {
    primitives_.push_back({Direction::FORWARD, angle});
  }

  // Reverse primitives
  if (motion_.allow_reverse) {
    for (double angle : angles) {
      primitives_.push_back({Direction::REVERSE, angle});
    }
  }
}

void MotionModel::generateFootprintTemplate()
{
  footprint_template_.clear();

  double half_w = vehicle_.width / 2.0;
  double rear = -vehicle_.rear_axle_to_back;
  double front = vehicle_.length - vehicle_.rear_axle_to_back;

  // Corner vertices
  vertex_template_[0] = {front, half_w};    // front-left
  vertex_template_[1] = {front, -half_w};   // front-right
  vertex_template_[2] = {rear, -half_w};    // rear-right
  vertex_template_[3] = {rear, half_w};     // rear-left

  // Sparse collision check points (16-20 points)
  // 4 corners
  for (const auto & v : vertex_template_) {
    footprint_template_.push_back(v);
  }

  // Edge midpoints
  footprint_template_.push_back({front, 0.0});         // front center
  footprint_template_.push_back({rear, 0.0});           // rear center
  footprint_template_.push_back({(front + rear) / 2.0, half_w});   // left center
  footprint_template_.push_back({(front + rear) / 2.0, -half_w});  // right center

  // Quarter-width points on front and rear edges
  footprint_template_.push_back({front, half_w / 2.0});
  footprint_template_.push_back({front, -half_w / 2.0});
  footprint_template_.push_back({rear, half_w / 2.0});
  footprint_template_.push_back({rear, -half_w / 2.0});

  // Side midpoints at quarter and three-quarter length
  double q1 = rear + (front - rear) * 0.25;
  double q3 = rear + (front - rear) * 0.75;
  footprint_template_.push_back({q1, half_w});
  footprint_template_.push_back({q1, -half_w});
  footprint_template_.push_back({q3, half_w});
  footprint_template_.push_back({q3, -half_w});
}

State MotionModel::applyMotion(
  const State & current,
  Direction direction,
  double steering_angle) const
{
  double step = motion_.step_size;
  double v = (direction == Direction::FORWARD) ? step : -step;

  double new_x, new_y, new_theta;

  if (std::abs(steering_angle) < 1e-6) {
    // Straight motion
    new_x = current.x + v * std::cos(current.theta);
    new_y = current.y + v * std::sin(current.theta);
    new_theta = current.theta;
  } else {
    // Arc motion using exact geometry
    double R = vehicle_.wheelbase / std::tan(steering_angle);
    double dtheta = v / R;

    new_x = current.x + R * (std::sin(current.theta + dtheta) - std::sin(current.theta));
    new_y = current.y + R * (-std::cos(current.theta + dtheta) + std::cos(current.theta));
    new_theta = normalizeAngle(current.theta + dtheta);
  }

  State result(new_x, new_y, new_theta);
  result.direction = direction;
  result.steering = steering_angle;
  return result;
}

Footprint MotionModel::getFootprint(const State & state) const
{
  double cos_t = std::cos(state.theta);
  double sin_t = std::sin(state.theta);

  Footprint fp;
  fp.points.reserve(footprint_template_.size());

  for (const auto & [lx, ly] : footprint_template_) {
    auto [wx, wy] = rotatePoint(lx, ly, cos_t, sin_t);
    fp.points.push_back({wx + state.x, wy + state.y});
  }

  return fp;
}

std::array<std::pair<double, double>, 4> MotionModel::getVertices(const State & state) const
{
  double cos_t = std::cos(state.theta);
  double sin_t = std::sin(state.theta);

  std::array<std::pair<double, double>, 4> result;
  for (int i = 0; i < 4; ++i) {
    auto [lx, ly] = vertex_template_[i];
    auto [wx, wy] = rotatePoint(lx, ly, cos_t, sin_t);
    result[i] = {wx + state.x, wy + state.y};
  }
  return result;
}

std::pair<double, double> MotionModel::rotatePoint(
  double x, double y, double cos_t, double sin_t)
{
  return {
    x * cos_t - y * sin_t,
    x * sin_t + y * cos_t
  };
}

}  // namespace hybrid_astar_planner
