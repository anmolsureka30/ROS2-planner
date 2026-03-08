#ifndef HYBRID_ASTAR_PLANNER__MOTION_MODEL_HPP_
#define HYBRID_ASTAR_PLANNER__MOTION_MODEL_HPP_

#include "hybrid_astar_planner/state.hpp"
#include "hybrid_astar_planner/types.hpp"
#include <vector>
#include <array>

namespace hybrid_astar_planner
{

/// Sparse footprint points for collision checking
struct Footprint
{
  std::vector<std::pair<double, double>> points;  // (x, y) in world frame
};

/// Bicycle kinematic model with motion primitive generation
class MotionModel
{
public:
  MotionModel() = default;

  /// Initialize with vehicle and motion configuration
  void initialize(const VehicleConfig & vehicle, const MotionConfig & motion);

  /// Apply a single motion step using exact arc geometry
  /// Returns the resulting state (g, h, parent not set)
  State applyMotion(
    const State & current,
    Direction direction,
    double steering_angle) const;

  /// Get all motion primitives
  const std::vector<MotionPrimitive> & getPrimitives() const { return primitives_; }

  /// Get vehicle footprint points (sparse) rotated to given state
  Footprint getFootprint(const State & state) const;

  /// Get vehicle footprint corner vertices in world frame
  std::array<std::pair<double, double>, 4> getVertices(const State & state) const;

  /// Get the minimum turning radius
  double getMinTurningRadius() const { return min_turning_radius_; }

  // Accessors
  const VehicleConfig & vehicleConfig() const { return vehicle_; }
  const MotionConfig & motionConfig() const { return motion_; }

private:
  void generatePrimitives();
  void generateFootprintTemplate();

  /// Rotate a point (x, y) by angle theta
  static std::pair<double, double> rotatePoint(
    double x, double y, double cos_t, double sin_t);

  VehicleConfig vehicle_;
  MotionConfig motion_;
  double min_turning_radius_ = 0.0;

  std::vector<MotionPrimitive> primitives_;

  // Template footprint points in vehicle frame (computed once)
  std::vector<std::pair<double, double>> footprint_template_;
  // Template corner vertices in vehicle frame
  std::array<std::pair<double, double>, 4> vertex_template_;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__MOTION_MODEL_HPP_
