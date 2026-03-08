#include "hybrid_astar_planner/collision_checker.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

void CollisionChecker::initialize(
  nav2_costmap_2d::Costmap2D * costmap,
  const MotionModel * motion_model,
  double xy_resolution,
  double theta_resolution_rad)
{
  costmap_ = costmap;
  motion_model_ = motion_model;
  xy_resolution_ = xy_resolution;
  theta_resolution_rad_ = theta_resolution_rad;

  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();

  clearCache();
}

void CollisionChecker::updateCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  clearCache();
}

bool CollisionChecker::isValid(const State & state)
{
  // Pack discrete state into cache key
  DiscreteState ds = discretize(state, xy_resolution_, theta_resolution_rad_);
  uint64_t key = packKey(ds.ix, ds.iy, ds.itheta);

  auto it = cache_.find(key);
  if (it != cache_.end()) {
    return it->second;
  }

  // Bounds check
  if (!isInBounds(state.x, state.y)) {
    cache_[key] = false;
    return false;
  }

  // Get vehicle footprint points in world frame
  Footprint fp = motion_model_->getFootprint(state);

  // Check each footprint point against costmap
  bool valid = true;
  for (const auto & [wx, wy] : fp.points) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      valid = false;
      break;
    }

    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      valid = false;
      break;
    }
  }

  cache_[key] = valid;
  return valid;
}

bool CollisionChecker::isPathValid(const std::vector<State> & path)
{
  if (path.empty()) {
    return false;
  }

  // Check every point — path is already discretized at step_size intervals
  // (typically 0.05m, smaller than vehicle width), and collision cache makes
  // repeated checks fast
  for (const auto & state : path) {
    if (!isValid(state)) {
      return false;
    }
  }

  return true;
}

double CollisionChecker::getTraversalCost(double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return 1.0;  // max cost for out-of-bounds
  }

  unsigned char cost = costmap_->getCost(mx, my);

  // Normalize costmap cost to [0, 1]
  // Nav2 costmap: 0 = free, 252 = inscribed, 253 = lethal, 254 = no info
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return 1.0;
  }

  return static_cast<double>(cost) / static_cast<double>(
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
}

double CollisionChecker::getObstacleDistance(double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return 0.0;
  }

  unsigned char cost = costmap_->getCost(mx, my);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return 0.0;
  }
  if (cost == nav2_costmap_2d::FREE_SPACE) {
    // Far from obstacles — return a large value scaled by map size
    return resolution_ * std::min(size_x_, size_y_) * 0.5;
  }

  // Inverse of Nav2 inflation formula: cost = 253 * exp(-scaling * (d - inscribed_r))
  // Approximate with linear mapping scaled by inflation radius
  double normalized = static_cast<double>(cost) / 252.0;
  // Use costmap resolution as a conservative scale factor
  double inflation_radius = resolution_ * 10.0;  // ~10 cells typical inflation
  return (1.0 - normalized) * inflation_radius;
}

void CollisionChecker::clearCache()
{
  cache_.clear();
}

uint64_t CollisionChecker::packKey(int ix, int iy, int itheta) const
{
  // Pack 3 ints into a single uint64
  // Assumes ix, iy fit in 24 bits each and itheta fits in 16 bits
  uint64_t key = 0;
  key |= (static_cast<uint64_t>(ix + 0x800000) & 0xFFFFFF) << 40;
  key |= (static_cast<uint64_t>(iy + 0x800000) & 0xFFFFFF) << 16;
  key |= (static_cast<uint64_t>(itheta) & 0xFFFF);
  return key;
}

bool CollisionChecker::isInBounds(double x, double y) const
{
  unsigned int mx, my;
  return costmap_->worldToMap(x, y, mx, my);
}

}  // namespace hybrid_astar_planner
