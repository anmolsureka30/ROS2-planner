#include "hybrid_astar_planner/heuristic_calculator.hpp"
#include "hybrid_astar_planner/analytic_expander.hpp"

#include <rclcpp/logging.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <queue>
#include <cmath>
#include <limits>

namespace hybrid_astar_planner
{

void HeuristicCalculator::initialize(
  const HeuristicConfig & config,
  nav2_costmap_2d::Costmap2D * costmap,
  const AnalyticExpander * analytic_expander,
  double turning_radius,
  const rclcpp::Logger & logger)
{
  config_ = config;
  costmap_ = costmap;
  analytic_expander_ = analytic_expander;
  turning_radius_ = turning_radius;
  logger_ = logger;

  map_size_x_ = costmap_->getSizeInCellsX();
  map_size_y_ = costmap_->getSizeInCellsY();

  distance_map_.resize(map_size_x_ * map_size_y_, std::numeric_limits<float>::infinity());
  dijkstra_computed_ = false;
}

void HeuristicCalculator::updateCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
  map_size_x_ = costmap_->getSizeInCellsX();
  map_size_y_ = costmap_->getSizeInCellsY();
  distance_map_.resize(map_size_x_ * map_size_y_, std::numeric_limits<float>::infinity());
  dijkstra_computed_ = false;
}

double HeuristicCalculator::compute(const State & state, const State & goal) const
{
  if (config_.type == "euclidean") {
    return euclideanDistance(state, goal);
  } else if (config_.type == "reeds_shepp") {
    return reedSheppDistance(state, goal);
  } else if (config_.type == "dijkstra") {
    return dijkstraDistance(state, goal);
  } else {  // "max" (default and recommended)
    double h_rs = reedSheppDistance(state, goal);
    double h_2d = dijkstraDistance(state, goal);
    return std::max(h_rs, h_2d);
  }
}

void HeuristicCalculator::precomputeFromGoal(const State & goal)
{
  dijkstra_computed_ = false;  // Reset before attempting

  unsigned int goal_mx, goal_my;
  if (!costmap_->worldToMap(goal.x, goal.y, goal_mx, goal_my)) {
    RCLCPP_WARN(logger_, "Goal outside costmap for Dijkstra precomputation");
    return;
  }

  unsigned char goal_cost = costmap_->getCost(goal_mx, goal_my);
  if (goal_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_WARN(logger_, "Goal in obstacle for Dijkstra precomputation");
    return;
  }

  compute2DDijkstra(goal_mx, goal_my);
  dijkstra_computed_ = true;
}

double HeuristicCalculator::euclideanDistance(
  const State & state, const State & goal) const
{
  return std::hypot(state.x - goal.x, state.y - goal.y);
}

double HeuristicCalculator::reedSheppDistance(
  const State & state, const State & goal) const
{
  if (!analytic_expander_) {
    return euclideanDistance(state, goal);
  }
  return analytic_expander_->getDistance(state, goal);
}

double HeuristicCalculator::dijkstraDistance(
  const State & state, const State & goal) const
{
  if (!dijkstra_computed_) {
    return euclideanDistance(state, goal);
  }

  unsigned int mx, my;
  if (!costmap_->worldToMap(state.x, state.y, mx, my)) {
    return std::numeric_limits<double>::infinity();
  }

  float dist_cells = distance_map_[my * map_size_x_ + mx];
  if (std::isinf(dist_cells)) {
    return std::numeric_limits<double>::infinity();
  }

  return static_cast<double>(dist_cells) * costmap_->getResolution();
}

void HeuristicCalculator::compute2DDijkstra(unsigned int goal_mx, unsigned int goal_my)
{
  // Reset distance map
  std::fill(distance_map_.begin(), distance_map_.end(),
    std::numeric_limits<float>::infinity());

  // Min-heap: (distance, flat_index)
  using QueueEntry = std::pair<float, unsigned int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;

  unsigned int goal_idx = goal_my * map_size_x_ + goal_mx;
  distance_map_[goal_idx] = 0.0f;
  pq.push({0.0f, goal_idx});

  // 8-connected neighbors with movement costs
  static const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
  static const int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};
  static const float dist[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

  double alpha = config_.cost_alpha;

  while (!pq.empty()) {
    auto [current_dist, current_idx] = pq.top();
    pq.pop();

    // Skip if we already found a better path
    if (current_dist > distance_map_[current_idx]) {
      continue;
    }

    unsigned int cx = current_idx % map_size_x_;
    unsigned int cy = current_idx / map_size_x_;

    for (int i = 0; i < 8; ++i) {
      int nx = static_cast<int>(cx) + dx[i];
      int ny = static_cast<int>(cy) + dy[i];

      // Bounds check
      if (nx < 0 || nx >= static_cast<int>(map_size_x_) ||
        ny < 0 || ny >= static_cast<int>(map_size_y_))
      {
        continue;
      }

      unsigned int nmx = static_cast<unsigned int>(nx);
      unsigned int nmy = static_cast<unsigned int>(ny);

      // Obstacle check
      unsigned char cell_cost = costmap_->getCost(nmx, nmy);
      if (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      // Cost-aware edge cost (Smac Planner Eq. 1)
      double c_ij = static_cast<double>(cell_cost) /
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
      float weighted_cost = dist[i] * static_cast<float>(1.0 + alpha * c_ij);

      unsigned int n_idx = nmy * map_size_x_ + nmx;
      float new_dist = current_dist + weighted_cost;

      if (new_dist < distance_map_[n_idx]) {
        distance_map_[n_idx] = new_dist;
        pq.push({new_dist, n_idx});
      }
    }
  }
}

}  // namespace hybrid_astar_planner
