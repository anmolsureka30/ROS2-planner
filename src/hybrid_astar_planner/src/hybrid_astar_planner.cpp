#include "hybrid_astar_planner/hybrid_astar_planner.hpp"

#include <nav2_util/node_utils.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <sstream>

// Register the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav2_core::GlobalPlanner)

namespace hybrid_astar_planner
{

void HybridAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring Hybrid A* Planner plugin: %s", name_.c_str());

  // Declare parameters
  declareParameters(node);

  // Validate parameters
  validateParameters();

  // Create publishers
  footprint_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    name_ + "/vehicle_footprints", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::String>(
    name_ + "/planning_stats", 1);

  // Initialize components
  initializeComponents();

  initialized_ = true;

  RCLCPP_INFO(logger_, "Hybrid A* Planner configured successfully");
}

void HybridAStarPlanner::declareParameters(nav2_util::LifecycleNode::SharedPtr node)
{
  // Vehicle parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vehicle.wheelbase", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vehicle.length", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vehicle.width", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vehicle.rear_axle_to_back", rclcpp::ParameterValue(0.06));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vehicle.max_steering_angle", rclcpp::ParameterValue(30.0));

  // Motion parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".motion.step_size", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".motion.num_steering_angles", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".motion.allow_reverse", rclcpp::ParameterValue(true));

  // Search parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search.xy_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search.theta_resolution", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search.shot_distance", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search.max_iterations", rclcpp::ParameterValue(100000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search.timeout", rclcpp::ParameterValue(5.0));

  // Cost parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.steering_penalty", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.reversing_penalty", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.steering_change_penalty", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.direction_switch_penalty", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.non_straight_penalty", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.change_direction_penalty", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost.cost_penalty_alpha", rclcpp::ParameterValue(1.5));

  // Heuristic parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heuristic.type", rclcpp::ParameterValue(std::string("max")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heuristic.cost_alpha", rclcpp::ParameterValue(1.0));

  // Goal tolerance
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal.xy_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal.theta_tolerance", rclcpp::ParameterValue(5.0));

  // Smoother parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.w_smooth", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.w_obstacle", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.w_original", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.obstacle_margin", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoother.max_iterations", rclcpp::ParameterValue(200));

  // Read all parameters
  node->get_parameter(name_ + ".vehicle.wheelbase", vehicle_config_.wheelbase);
  node->get_parameter(name_ + ".vehicle.length", vehicle_config_.length);
  node->get_parameter(name_ + ".vehicle.width", vehicle_config_.width);
  node->get_parameter(name_ + ".vehicle.rear_axle_to_back", vehicle_config_.rear_axle_to_back);

  double max_steer_deg;
  node->get_parameter(name_ + ".vehicle.max_steering_angle", max_steer_deg);
  vehicle_config_.max_steering_angle = max_steer_deg * M_PI / 180.0;

  node->get_parameter(name_ + ".motion.step_size", motion_config_.step_size);
  node->get_parameter(name_ + ".motion.num_steering_angles", motion_config_.num_steering_angles);
  node->get_parameter(name_ + ".motion.allow_reverse", motion_config_.allow_reverse);

  node->get_parameter(name_ + ".search.xy_resolution", search_config_.xy_resolution);
  node->get_parameter(name_ + ".search.theta_resolution", search_config_.theta_resolution);
  node->get_parameter(name_ + ".search.shot_distance", search_config_.shot_distance);
  node->get_parameter(name_ + ".search.max_iterations", search_config_.max_iterations);
  node->get_parameter(name_ + ".search.timeout", search_config_.timeout);

  node->get_parameter(name_ + ".cost.steering_penalty", cost_config_.steering_penalty);
  node->get_parameter(name_ + ".cost.reversing_penalty", cost_config_.reversing_penalty);
  node->get_parameter(name_ + ".cost.steering_change_penalty",
    cost_config_.steering_change_penalty);
  node->get_parameter(name_ + ".cost.direction_switch_penalty",
    cost_config_.direction_switch_penalty);
  node->get_parameter(name_ + ".cost.non_straight_penalty", cost_config_.non_straight_penalty);
  node->get_parameter(name_ + ".cost.change_direction_penalty",
    cost_config_.change_direction_penalty);
  node->get_parameter(name_ + ".cost.cost_penalty_alpha", cost_config_.cost_penalty_alpha);

  node->get_parameter(name_ + ".heuristic.type", heuristic_config_.type);
  node->get_parameter(name_ + ".heuristic.cost_alpha", heuristic_config_.cost_alpha);

  node->get_parameter(name_ + ".goal.xy_tolerance", goal_config_.xy_tolerance);
  node->get_parameter(name_ + ".goal.theta_tolerance", goal_config_.theta_tolerance);

  node->get_parameter(name_ + ".smoother.enabled", smoother_config_.enabled);
  node->get_parameter(name_ + ".smoother.w_smooth", smoother_config_.w_smooth);
  node->get_parameter(name_ + ".smoother.w_obstacle", smoother_config_.w_obstacle);
  node->get_parameter(name_ + ".smoother.w_original", smoother_config_.w_original);
  node->get_parameter(name_ + ".smoother.obstacle_margin", smoother_config_.obstacle_margin);
  node->get_parameter(name_ + ".smoother.max_iterations", smoother_config_.max_iterations);

  RCLCPP_INFO(logger_, "Vehicle: %.2fm x %.2fm, wheelbase=%.2fm, max_steer=%.0fdeg",
    vehicle_config_.length, vehicle_config_.width,
    vehicle_config_.wheelbase, max_steer_deg);
  RCLCPP_INFO(logger_, "Search: xy_res=%.3fm, theta_res=%.1fdeg, shot_dist=%.1fm",
    search_config_.xy_resolution, search_config_.theta_resolution,
    search_config_.shot_distance);
  RCLCPP_INFO(logger_, "Heuristic: type=%s", heuristic_config_.type.c_str());
}

void HybridAStarPlanner::validateParameters()
{
  if (vehicle_config_.wheelbase <= 0.0) {
    throw std::invalid_argument("vehicle.wheelbase must be positive");
  }
  if (vehicle_config_.length <= 0.0) {
    throw std::invalid_argument("vehicle.length must be positive");
  }
  if (vehicle_config_.width <= 0.0) {
    throw std::invalid_argument("vehicle.width must be positive");
  }
  if (vehicle_config_.max_steering_angle <= 0.0) {
    throw std::invalid_argument("vehicle.max_steering_angle must be positive");
  }
  if (motion_config_.step_size <= 0.0) {
    throw std::invalid_argument("motion.step_size must be positive");
  }
  if (motion_config_.num_steering_angles < 1) {
    throw std::invalid_argument("motion.num_steering_angles must be >= 1");
  }
  if (search_config_.xy_resolution <= 0.0) {
    throw std::invalid_argument("search.xy_resolution must be positive");
  }
  if (search_config_.theta_resolution <= 0.0 || search_config_.theta_resolution >= 360.0) {
    throw std::invalid_argument("search.theta_resolution must be in (0, 360)");
  }
  if (search_config_.timeout <= 0.0) {
    throw std::invalid_argument("search.timeout must be positive");
  }
  if (cost_config_.reversing_penalty < 1.0) {
    RCLCPP_WARN(logger_, "reversing_penalty < 1.0 (%.2f) makes reverse cheaper than forward",
      cost_config_.reversing_penalty);
  }
  if (goal_config_.xy_tolerance <= 0.0) {
    throw std::invalid_argument("goal.xy_tolerance must be positive");
  }
}

void HybridAStarPlanner::initializeComponents()
{
  // 1. Motion Model
  motion_model_ = std::make_unique<MotionModel>();
  motion_model_->initialize(vehicle_config_, motion_config_);

  // 2. Collision Checker
  collision_checker_ = std::make_unique<CollisionChecker>();
  double theta_res_rad = search_config_.theta_resolution * M_PI / 180.0;
  collision_checker_->initialize(
    costmap_, motion_model_.get(),
    search_config_.xy_resolution, theta_res_rad);

  // 3. Analytic Expander (OMPL Reeds-Shepp)
  analytic_expander_ = std::make_unique<AnalyticExpander>();
  double turning_radius = motion_model_->getMinTurningRadius();
  analytic_expander_->initialize(turning_radius, motion_config_.step_size);

  RCLCPP_INFO(logger_, "Turning radius: %.3fm", turning_radius);

  // 4. Heuristic Calculator
  heuristic_ = std::make_unique<HeuristicCalculator>();
  heuristic_->initialize(
    heuristic_config_, costmap_,
    analytic_expander_.get(), turning_radius, logger_);

  // 5. Cost Function
  cost_function_ = std::make_unique<CostFunction>();
  cost_function_->initialize(cost_config_, vehicle_config_, motion_config_.step_size);

  // 6. Path Smoother
  path_smoother_ = std::make_unique<PathSmoother>();
  path_smoother_->initialize(smoother_config_);

  // 7. Search Engine
  search_engine_ = std::make_unique<SearchEngine>();
  search_engine_->initialize(
    search_config_, goal_config_,
    motion_model_.get(), collision_checker_.get(),
    heuristic_.get(), analytic_expander_.get(),
    cost_function_.get(), path_smoother_.get(),
    logger_);
}

void HybridAStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up Hybrid A* Planner plugin");
  search_engine_.reset();
  path_smoother_.reset();
  cost_function_.reset();
  heuristic_.reset();
  analytic_expander_.reset();
  collision_checker_.reset();
  motion_model_.reset();
  initialized_ = false;
}

void HybridAStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating Hybrid A* Planner plugin");
  footprint_pub_->on_activate();
  stats_pub_->on_activate();
}

void HybridAStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating Hybrid A* Planner plugin");
  footprint_pub_->on_deactivate();
  stats_pub_->on_deactivate();
}

nav_msgs::msg::Path HybridAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(planning_mutex_);

  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Planner not initialized!");
    return path;
  }

  // Frame validation
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  if (!start.header.frame_id.empty() && start.header.frame_id != global_frame) {
    RCLCPP_ERROR(logger_,
      "Start frame (%s) doesn't match costmap frame (%s)",
      start.header.frame_id.c_str(), global_frame.c_str());
    return path;
  }
  if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame) {
    RCLCPP_ERROR(logger_,
      "Goal frame (%s) doesn't match costmap frame (%s)",
      goal.header.frame_id.c_str(), global_frame.c_str());
    return path;
  }

  // Update costmap reference (lightweight — only updates pointer if changed)
  costmap_ = costmap_ros_->getCostmap();
  collision_checker_->updateCostmap(costmap_);
  heuristic_->updateCostmap(costmap_);

  // Convert to internal states
  State start_state = poseToState(start);
  State goal_state = poseToState(goal);

  RCLCPP_INFO(logger_, "Planning from (%.2f, %.2f, %.0fdeg) to (%.2f, %.2f, %.0fdeg)",
    start_state.x, start_state.y, start_state.theta * 180.0 / M_PI,
    goal_state.x, goal_state.y, goal_state.theta * 180.0 / M_PI);

  // Run planner
  auto result = search_engine_->plan(start_state, goal_state);

  if (result.has_value()) {
    auto & [states, stats] = result.value();

    path = statesToPath(states, path.header);

    // Publish visualization
    publishFootprintMarkers(states, path.header);
    publishStats(stats);

    RCLCPP_INFO(logger_,
      "Path found: length=%.2fm, nodes=%d, time=%.3fs, analytical=%s",
      stats.path_length, stats.nodes_expanded, stats.search_time,
      stats.analytical_expansion ? "yes" : "no");
  } else {
    RCLCPP_WARN(logger_, "No path found!");
  }

  return path;
}

nav_msgs::msg::Path HybridAStarPlanner::statesToPath(
  const std::vector<State> & states,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(states.size());

  for (const auto & state : states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.theta);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }

  return path;
}

State HybridAStarPlanner::poseToState(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  double yaw = tf2::getYaw(pose.pose.orientation);
  return State(pose.pose.position.x, pose.pose.position.y, yaw);
}

void HybridAStarPlanner::publishFootprintMarkers(
  const std::vector<State> & path,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Show footprint every N states
  int interval = std::max(1, static_cast<int>(path.size()) / 20);

  int id = 0;
  for (size_t i = 0; i < path.size(); i += interval) {
    auto vertices = motion_model_->getVertices(path[i]);

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "vehicle_footprint";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.005;  // line width
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    marker.color.a = 0.6;
    marker.lifetime = rclcpp::Duration(0, 0);  // persistent

    // Add vertices as a closed polygon
    for (int v = 0; v < 4; ++v) {
      geometry_msgs::msg::Point p;
      p.x = vertices[v].first;
      p.y = vertices[v].second;
      p.z = 0.01;
      marker.points.push_back(p);
    }
    // Close the polygon
    geometry_msgs::msg::Point p;
    p.x = vertices[0].first;
    p.y = vertices[0].second;
    p.z = 0.01;
    marker.points.push_back(p);

    markers.markers.push_back(marker);
  }

  // Delete old markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header = header;
  delete_marker.ns = "vehicle_footprint";
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  // Publish delete first, then new markers
  visualization_msgs::msg::MarkerArray delete_array;
  delete_array.markers.push_back(delete_marker);
  footprint_pub_->publish(delete_array);

  if (!markers.markers.empty()) {
    footprint_pub_->publish(markers);
  }
}

void HybridAStarPlanner::publishStats(const PlanningStats & stats)
{
  std_msgs::msg::String msg;
  std::ostringstream ss;
  ss << "{"
     << "\"success\":" << (stats.success ? "true" : "false") << ","
     << "\"nodes_expanded\":" << stats.nodes_expanded << ","
     << "\"nodes_visited\":" << stats.nodes_visited << ","
     << "\"search_time\":" << stats.search_time << ","
     << "\"path_length\":" << stats.path_length << ","
     << "\"min_clearance\":" << stats.min_clearance << ","
     << "\"curvature_sum\":" << stats.curvature_sum << ","
     << "\"analytical_expansion\":" << (stats.analytical_expansion ? "true" : "false")
     << "}";
  msg.data = ss.str();
  stats_pub_->publish(msg);
}

}  // namespace hybrid_astar_planner
