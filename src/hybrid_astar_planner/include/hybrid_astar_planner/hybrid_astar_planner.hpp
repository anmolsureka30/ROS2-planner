#ifndef HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER_HPP_

#include "hybrid_astar_planner/search_engine.hpp"
#include "hybrid_astar_planner/motion_model.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/heuristic_calculator.hpp"
#include "hybrid_astar_planner/analytic_expander.hpp"
#include "hybrid_astar_planner/cost_function.hpp"
#include "hybrid_astar_planner/path_smoother.hpp"
#include "hybrid_astar_planner/types.hpp"

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <mutex>

namespace hybrid_astar_planner
{

/// Nav2 Global Planner plugin implementing Hybrid A* with:
/// - Cost-aware traversal (Smac Planner formulation)
/// - OMPL Reeds-Shepp analytical expansion
/// - Max(RS, 2D Dijkstra) admissible heuristic
/// - Gradient descent path smoothing
/// - Full vehicle footprint collision checking
class HybridAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  HybridAStarPlanner() = default;
  ~HybridAStarPlanner() override = default;

  /// Configure the planner (called by Nav2 lifecycle)
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// Cleanup resources
  void cleanup() override;

  /// Activate the planner
  void activate() override;

  /// Deactivate the planner
  void deactivate() override;

  /// Create a plan from start to goal
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /// Declare and read ROS2 parameters
  void declareParameters(nav2_util::LifecycleNode::SharedPtr node);

  /// Validate parameter ranges
  void validateParameters();

  /// Initialize all sub-components
  void initializeComponents();

  /// Convert a vector of States to nav_msgs::Path
  nav_msgs::msg::Path statesToPath(
    const std::vector<State> & states,
    const std_msgs::msg::Header & header) const;

  /// Convert PoseStamped to State
  State poseToState(const geometry_msgs::msg::PoseStamped & pose) const;

  /// Publish vehicle footprint markers for visualization
  void publishFootprintMarkers(
    const std::vector<State> & path,
    const std_msgs::msg::Header & header);

  /// Publish planning statistics
  void publishStats(const PlanningStats & stats);

  // Nav2 lifecycle node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("hybrid_astar_planner")};

  // Planning mutex
  std::mutex planning_mutex_;

  // Sub-components
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<CollisionChecker> collision_checker_;
  std::unique_ptr<HeuristicCalculator> heuristic_;
  std::unique_ptr<AnalyticExpander> analytic_expander_;
  std::unique_ptr<CostFunction> cost_function_;
  std::unique_ptr<PathSmoother> path_smoother_;
  std::unique_ptr<SearchEngine> search_engine_;

  // Configuration
  VehicleConfig vehicle_config_;
  MotionConfig motion_config_;
  SearchConfig search_config_;
  CostConfig cost_config_;
  HeuristicConfig heuristic_config_;
  GoalConfig goal_config_;
  SmootherConfig smoother_config_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    footprint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    stats_pub_;

  // State
  bool initialized_ = false;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
