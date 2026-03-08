# Hybrid A* Path Planner - ROS2 Nav2 Plugin

A high-performance **Hybrid A\* global path planner** implemented as a Nav2 plugin for ROS2 Humble. Designed for **Ackermann-steered vehicles** with kinematic constraints, full footprint collision checking, and analytical Reeds-Shepp curve expansion.

## Architecture

```
                        +-----------------+
  ComputePathToPose     |  planner_server | (Nav2 lifecycle node)
  Action Request ------>|                 |
                        |  HybridAStar    | (pluginlib plugin)
                        |  Plugin         |
                        +--------+--------+
                                 |
              +------------------+------------------+
              |                  |                  |
     +--------v-------+ +-------v--------+ +-------v--------+
     | SearchEngine    | | CollisionCheck | | HeuristicCalc  |
     | (A* open/closed)| | (footprint +   | | (max(RS, 2D    |
     |                 | |  costmap cache)| |  Dijkstra))    |
     +--------+--------+ +-------+--------+ +----------------+
              |                  |
     +--------v--------+ +------v---------+
     | MotionModel     | | AnalyticExpand |
     | (bicycle arcs)  | | (OMPL Reeds-  |
     |                 | |  Shepp)        |
     +-----------------+ +------+---------+
                                |
                         +------v---------+
                         | PathSmoother   |
                         | (gradient      |
                         |  descent)      |
                         +----------------+
```

## Features

- **Nav2 Plugin**: Drop-in `nav2_core::GlobalPlanner` plugin, works with any Nav2 stack
- **Ackermann Kinematics**: Exact bicycle-model arc geometry (not straight-line approximation)
- **Reeds-Shepp Analytical Expansion**: OMPL-based optimal curves with forward + reverse segments
- **Admissible Heuristic**: `max(Reeds-Shepp distance, 2D cost-aware Dijkstra)` guarantees optimality
- **Full Footprint Collision Checking**: Rotated rectangular vehicle footprint at every state
- **Cost-Aware Traversal**: 7-term Smac Planner cost function (steering, reversing, proximity penalties)
- **Gradient Descent Smoothing**: 3-objective path smoother (smoothness + obstacle avoidance + fidelity)
- **Performance Optimized**: Node pool pre-allocation, collision cache, inline expansion (~0.5s for 10m paths)

## Packages

| Package | Description |
|---------|-------------|
| `hybrid_astar_planner` | Core planner plugin (C++ library) |
| `hybrid_astar_bringup` | Launch files, configs, maps, scripts |
| `hybrid_astar_description` | Vehicle URDF (Ackermann model) |
| `hybrid_astar_gazebo` | Gazebo simulation world |

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04 or Docker `osrf/ros:humble-desktop-full`)
- **Nav2** packages:
  ```bash
  sudo apt install ros-humble-nav2-core ros-humble-nav2-costmap-2d \
    ros-humble-nav2-util ros-humble-nav2-planner ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs
  ```
- **OMPL** (Open Motion Planning Library):
  ```bash
  sudo apt install ros-humble-ompl libompl-dev
  ```
- **TF2**:
  ```bash
  sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
  ```

## Build

```bash
# Clone
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/anmolsureka30/ROS2-planner.git .

# Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source
source install/setup.bash
```

## Quick Start

### Option 1: Launch file
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start planner (nodes will auto-configure via lifecycle script)
ros2 launch hybrid_astar_bringup planner_demo.launch.py
```

### Option 2: All-in-one script (recommended for Docker)
```bash
bash run_and_visualize.sh
```
This starts all nodes, transitions them through lifecycle states, runs 3 test planning scenarios, and saves visualization PNGs.

### Send a planning goal
```bash
# In another terminal:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 7.0, y: 7.0}, \
  orientation: {z: 0.707, w: 0.707}}}, \
  start: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}, \
  orientation: {w: 1.0}}}, use_start: true}"
```

### Visualize (headless / Docker)
```bash
# Generate path visualization images
python3 scripts/visualize_planner.py 2.0 2.0 0 7.0 7.0 90

# Copy to host (if in Docker)
docker cp <container>:/root/planner_output.png ~/Desktop/
```

## ROS2 Interface

### Nodes

| Node | Package | Role |
|------|---------|------|
| `/planner_server` | `nav2_planner` | Hosts Hybrid A* plugin, handles planning requests |
| `/map_server` | `nav2_map_server` | Publishes static occupancy grid map |
| `/global_costmap/global_costmap` | `nav2_costmap_2d` | Inflated costmap for collision checking |
| `/static_tf_map_to_base` | `tf2_ros` | Static TF (testing without robot) |

### Topics Published

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/plan` | `nav_msgs/msg/Path` | Reliable, Volatile | **Planned path** (array of PoseStamped) |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Reliable, Transient Local | Static map |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Reliable, Transient Local | Inflated costmap |
| `/HybridAStar/planning_stats` | `std_msgs/msg/String` | Reliable, Volatile | Planning metrics (JSON) |
| `/HybridAStar/vehicle_footprints` | `visualization_msgs/msg/MarkerArray` | Reliable | Vehicle footprint along path |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | - | Transform frames |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | **Primary planning interface** |
| `/compute_path_through_poses` | `nav2_msgs/action/ComputePathThroughPoses` | Multi-waypoint planning |

### Action Goal Fields (`ComputePathToPose`)

| Field | Type | Description |
|-------|------|-------------|
| `goal` | `PoseStamped` | Target pose (position + orientation in `map` frame) |
| `start` | `PoseStamped` | Start pose (used if `use_start=true`) |
| `use_start` | `bool` | `true` = use provided start; `false` = use robot TF pose |
| `planner_id` | `string` | Leave empty (defaults to `HybridAStar`) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/is_path_valid` | `nav2_msgs/srv/IsPathValid` | Check if path is collision-free |
| `/global_costmap/get_costmap` | `nav2_msgs/srv/GetCostmap` | Get current costmap |
| `/map_server/map` | `nav_msgs/srv/GetMap` | Get static map |

### Planning Stats JSON (`/HybridAStar/planning_stats`)

```json
{
  "success": true,
  "nodes_expanded": 43848,
  "nodes_visited": 28653,
  "search_time": 0.532,
  "path_length": 8.11,
  "min_clearance": 0.25,
  "curvature_sum": 3.14,
  "analytical_expansion": true
}
```

## Controller Team Integration

### Path Format

The `/plan` topic publishes `nav_msgs/msg/Path` with:
- **Frame**: `map`
- **Waypoint spacing**: ~0.05m (configurable via `step_size`)
- **Heading**: Encoded as quaternion. Extract yaw: `yaw = atan2(2*w*z, 1 - 2*z*z)`
- **Reverse segments**: Check `dot(heading_vector, displacement_vector)` — negative = reversing

### Minimal Python Subscriber

```python
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Path

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.create_subscription(Path, '/plan', self.path_cb, 10)

    def path_cb(self, msg):
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            qz, qw = ps.pose.orientation.z, ps.pose.orientation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            # Use (x, y, yaw) for control

rclpy.init()
rclpy.spin(PathFollower())
```

### Minimal C++ Subscriber

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>

class PathFollower : public rclcpp::Node {
public:
  PathFollower() : Node("path_follower") {
    sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, [this](nav_msgs::msg::Path::SharedPtr msg) {
        for (auto& ps : msg->poses) {
          double x = ps.pose.position.x;
          double y = ps.pose.position.y;
          double yaw = tf2::getYaw(ps.pose.orientation);
          // Use (x, y, yaw) for control
        }
      });
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
};
```

## Parameters

All parameters are declared under `planner_server.ros__parameters.HybridAStar`:

### Vehicle

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicle.wheelbase` | 0.18 | Axle-to-axle distance (m) |
| `vehicle.length` | 0.30 | Vehicle length (m) |
| `vehicle.width` | 0.20 | Vehicle width (m) |
| `vehicle.rear_axle_to_back` | 0.06 | Rear axle to rear bumper (m) |
| `vehicle.max_steering_angle` | 30.0 | Max steering angle (deg) |

### Motion

| Parameter | Default | Description |
|-----------|---------|-------------|
| `motion.step_size` | 0.05 | Distance per motion primitive (m) |
| `motion.num_steering_angles` | 5 | Steering discretization count |
| `motion.allow_reverse` | true | Enable reverse driving |

### Search

| Parameter | Default | Description |
|-----------|---------|-------------|
| `search.xy_resolution` | 0.05 | Grid cell size for closed set (m) |
| `search.theta_resolution` | 5.0 | Heading bin size (deg) |
| `search.shot_distance` | 2.0 | Max RS expansion distance (m) |
| `search.max_iterations` | 100000 | Iteration limit |
| `search.timeout` | 5.0 | Planning timeout (s) |

### Cost Function

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cost.steering_penalty` | 1.5 | Steering magnitude penalty |
| `cost.reversing_penalty` | 2.0 | Reverse motion multiplier |
| `cost.steering_change_penalty` | 1.5 | Steering change penalty |
| `cost.direction_switch_penalty` | 10.0 | Forward/reverse switch penalty |
| `cost.non_straight_penalty` | 0.03 | Curved motion penalty |
| `cost.change_direction_penalty` | 0.05 | Steering sign reversal penalty |
| `cost.cost_penalty_alpha` | 1.5 | Obstacle proximity weight |

### Heuristic

| Parameter | Default | Description |
|-----------|---------|-------------|
| `heuristic.type` | "max" | `euclidean`, `reeds_shepp`, `dijkstra`, `max` |
| `heuristic.cost_alpha` | 1.0 | Cost-aware Dijkstra weight |

### Goal

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal.xy_tolerance` | 0.05 | Position tolerance (m) |
| `goal.theta_tolerance` | 10.0 | Heading tolerance (deg) |

### Smoother

| Parameter | Default | Description |
|-----------|---------|-------------|
| `smoother.enabled` | true | Enable post-smoothing |
| `smoother.w_smooth` | 0.4 | Smoothness weight |
| `smoother.w_obstacle` | 0.3 | Obstacle avoidance weight |
| `smoother.w_original` | 0.3 | Original path fidelity |
| `smoother.obstacle_margin` | 0.15 | Safety margin (m) |
| `smoother.max_iterations` | 200 | Smoother iterations |

## Directory Structure

```
src/
  hybrid_astar_planner/          # Core planner plugin
    include/hybrid_astar_planner/
      types.hpp                  # Config structs, enums, utilities
      state.hpp                  # State representation (x, y, theta, g, h)
      motion_model.hpp           # Bicycle model, motion primitives
      collision_checker.hpp      # Footprint-based collision checking
      cost_function.hpp          # 7-term Smac cost function
      heuristic_calculator.hpp   # max(RS, 2D Dijkstra) heuristic
      analytic_expander.hpp      # OMPL Reeds-Shepp expansion
      search_engine.hpp          # A* search loop, node pool
      path_smoother.hpp          # Gradient descent smoother
      hybrid_astar_planner.hpp   # Nav2 plugin entry point
    src/
      hybrid_astar_planner.cpp   # Plugin: configure(), createPlan()
      search_engine.cpp          # Core A* loop with analytical expansion
      motion_model.cpp           # Arc-based motion primitives
      collision_checker.cpp      # Costmap collision + cache
      cost_function.cpp          # Smac cost computation
      heuristic_calculator.cpp   # 2D BFS + RS heuristic
      analytic_expander.cpp      # OMPL RS curve generation
      path_smoother.cpp          # Gradient descent smoothing
    test/
      test_motion_model.cpp      # Motion model unit tests
      test_cost_function.cpp     # Cost function unit tests
    config/
      planner_params.yaml        # Default parameter reference
    CMakeLists.txt
    package.xml
    hybrid_astar_planner_plugin.xml

  hybrid_astar_bringup/          # Launch & configuration
    launch/
      planner_demo.launch.py     # Main launch file
    config/
      nav2_params.yaml           # Nav2 + planner parameters
      planner_only.rviz          # RViz config
    maps/
      test_field.yaml            # 10m x 10m test map metadata
      test_field.pgm             # 200x200 occupancy grid image
    scripts/
      run_and_visualize.sh       # All-in-one demo script
      visualize_planner.py       # Matplotlib path visualizer
    CMakeLists.txt
    package.xml

  hybrid_astar_description/      # Robot URDF
    urdf/
      ackermann_vehicle.urdf.xacro
    CMakeLists.txt
    package.xml

  hybrid_astar_gazebo/           # Simulation
    worlds/
      test_field.world
    CMakeLists.txt
    package.xml
```

## Docker Setup

```bash
# Pull ROS2 Humble image
docker run -it --name ros2_planner osrf/ros:humble-desktop-full bash

# Inside container: install Nav2
apt update && apt install -y \
  ros-humble-nav2-core ros-humble-nav2-costmap-2d ros-humble-nav2-util \
  ros-humble-nav2-planner ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
  ros-humble-ompl libompl-dev ros-humble-tf2-ros

# Clone and build
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/anmolsureka30/ROS2-planner.git .
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Run
bash src/hybrid_astar_bringup/scripts/run_and_visualize.sh
```

## Performance

Tested on 200x200 map (10m x 10m at 0.05m resolution) with 2 obstacles:

| Scenario | Path Length | Time | Nodes | Method |
|----------|-----------|------|-------|--------|
| (2,2) to (7,7) diagonal | 8.11m | 0.53s | 43,848 | RS shot at 2.99m |
| (8,8) to (2,2) reverse | 9.55m | 0.37s | 26,438 | RS shot at 2.99m |

## Algorithm Overview

1. **State Space**: Continuous (x, y, theta) with discrete grid for duplicate detection
2. **Motion Primitives**: Bicycle-model arcs at multiple steering angles (forward + reverse)
3. **Collision Check**: Rotated rectangular footprint checked against Nav2 costmap
4. **Heuristic**: `max(Reeds-Shepp optimal length, 2D cost-aware Dijkstra distance)` - admissible
5. **Analytical Expansion**: Every ~20 nodes near goal, attempt OMPL Reeds-Shepp connection
6. **Path Smoothing**: Gradient descent minimizing curvature + obstacle proximity + path deviation

## License

Apache-2.0
