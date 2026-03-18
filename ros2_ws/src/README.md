# ROS 2 Path Planner Workspace

ROS 2 middleware for Hybrid A\* and RRT\* family path planners. Each planner runs as an independent lifecycle node, receives maps via `/map`, and serves planning requests on `/plan_path`. Both are plug-and-play — swap the planner by changing which launch file you use.

## Package Architecture

```
ros2_ws/src/
├── av_planner_interfaces/      # Custom action & message definitions
│   ├── action/PlanPath.action   #   Goal: start + goal poses
│   └── msg/PlanningStats.msg    #   Result: search_time, path_length, nodes_expanded
│
├── planner_common/             # Shared utilities (both planners depend on this)
│   ├── adapters/
│   │   ├── map_adapter.py       #   OccupancyGrid → MapHandler adapter (duck-typed)
│   │   └── converters.py        #   State ↔ PoseStamped, Path conversions
│   └── nodes/
│       └── mock_map_node.py     #   Publishes OccupancyGrid from PNG (for testing)
│
├── hybrid_astar_planner/       # Hybrid A* planner node
│   ├── nodes/planner_node.py    #   Lifecycle node wrapping Hybrid A*
│   ├── config/params.yaml       #   Vehicle, motion, heuristic parameters
│   └── launch/planner.launch.xml
│
└── rrt_planner/                # RRT* family planner node
    ├── nodes/rrt_planner_node.py #  Lifecycle node wrapping RRT*/Informed/BI-RRT*
    ├── config/params.yaml        #  Algorithm selection, iterations, goal_radius
    └── launch/rrt_planner.launch.xml
```

### Dependency Graph

```
av_planner_interfaces  (defines PlanPath.action, PlanningStats.msg)
         ↑
    ┌────┴────┐
    │         │
planner_common   (MapAdapter, converters, mock_map_node)
    ↑         ↑
    │         │
hybrid_astar_planner    rrt_planner
(independent)           (independent)
```

Both planner packages depend on `planner_common` and `av_planner_interfaces`, but **not on each other**. Either can be built, deployed, or removed independently.

---

## Prerequisites

- **ROS 2** Humble or Iron (tested on Humble)
- **Python 3.10+**
- **colcon** build tool
- Python packages: `numpy`, `scipy`, `Pillow`, `PyYAML`

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Python dependencies (if not already installed)
pip install numpy scipy Pillow PyYAML
```

---

## Build

```bash
cd ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Quick Start

### Option A: Hybrid A\* Planner

```bash
# Terminal 1: Launch
ros2 launch hybrid_astar_planner planner.launch.xml

# Terminal 2: Configure and activate
ros2 lifecycle set /hybrid_astar_node configure
ros2 lifecycle set /hybrid_astar_node activate

# Terminal 2: Send a planning request
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 50.0}, orientation: {w: 1.0}}},
    goal: {pose: {position: {x: 90.0, y: 50.0}, orientation: {w: 1.0}}}}"
```

### Option B: RRT\* Family Planner

```bash
# Terminal 1: Launch (defaults to RRT*)
ros2 launch rrt_planner rrt_planner.launch.xml

# Terminal 2: Configure and activate
ros2 lifecycle set /rrt_planner_node configure
ros2 lifecycle set /rrt_planner_node activate

# Terminal 2: Send a planning request (same action interface!)
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0}, orientation: {w: 1.0}}},
    goal: {pose: {position: {x: 80.0, y: 80.0}, orientation: {z: 0.38, w: 0.92}}}}"
```

### Switch RRT\* Algorithm (no restart needed)

```bash
# Switch to Informed RRT* (takes effect on next map update)
ros2 param set /rrt_planner_node algorithm informed_rrt_star

# Switch to BI-RRT*
ros2 param set /rrt_planner_node algorithm bi_rrt_star

# Switch back to base RRT*
ros2 param set /rrt_planner_node algorithm rrt_star
```

---

## Connecting to a Real Pipeline

Both planners are designed to work with any perception/control pipeline that publishes an `OccupancyGrid` on `/map`.

### Input: Map (from perception)

```
Topic:    /map
Type:     nav_msgs/OccupancyGrid
QoS:      RELIABLE, TRANSIENT_LOCAL, depth=1
```

Your perception stack publishes an OccupancyGrid. The planner node subscribes, builds the internal map representation, and is ready to plan.

### Output: Path (to controller)

```
Topic:    /planned_path
Type:     nav_msgs/Path
```

After each successful planning request, the path is also published on `/planned_path` for visualization (RViz) or direct consumption by a controller.

### Output: Stats

```
Topic:    /planning_stats
Type:     av_planner_interfaces/PlanningStats
Fields:   search_time, path_length, nodes_expanded, analytical_expansion
```

### Planning Request (from mission planner / user)

```
Action:   /plan_path
Type:     av_planner_interfaces/action/PlanPath
Goal:     start (PoseStamped), goal (PoseStamped)
Result:   path (Path), stats (PlanningStats), success (bool), message (string)
Feedback: status (string)
```

### Integration Example

```
┌─────────────┐     /map        ┌──────────────────┐     /planned_path     ┌────────────┐
│ Perception  │ ──────────────→ │ Planner Node     │ ──────────────────→  │ Controller │
│ (SLAM/LiDAR)│  OccupancyGrid  │ (Hybrid A* or    │  nav_msgs/Path       │ (MPC/PID)  │
└─────────────┘                 │  RRT* family)    │                      └────────────┘
                                └──────────────────┘
                                       ↑
                                /plan_path (action)
                                       │
                                ┌──────────────┐
                                │ Mission      │
                                │ Planner /    │
                                │ Operator     │
                                └──────────────┘
```

### Without Perception (Standalone Testing)

Launch with `use_mock_map:=true` (default). The `mock_map_node` loads a PNG image and publishes it as an OccupancyGrid:

```bash
ros2 launch rrt_planner rrt_planner.launch.xml use_mock_map:=true
```

With real perception:

```bash
ros2 launch rrt_planner rrt_planner.launch.xml use_mock_map:=false
# Your perception node publishes on /map — the planner picks it up automatically
```

---

## Lifecycle States

Both planner nodes use the ROS 2 lifecycle pattern:

| State | What happens |
|-------|-------------|
| **Unconfigured** | Node created, parameters declared |
| **Configured** | Core algorithm imported, map subscriber + action server created |
| **Active** | Ready to receive maps and planning requests |
| **Deactivated** | Paused, no new requests accepted |

```bash
# Transition commands
ros2 lifecycle set /<node_name> configure
ros2 lifecycle set /<node_name> activate
ros2 lifecycle set /<node_name> deactivate
ros2 lifecycle set /<node_name> cleanup
```

---

## Available Algorithms

| Algorithm | Package | How to Select |
|-----------|---------|---------------|
| Hybrid A\* | `hybrid_astar_planner` | Launch `planner.launch.xml` |
| RRT\* | `rrt_planner` | `algorithm: rrt_star` (default) |
| Informed RRT\* | `rrt_planner` | `algorithm: informed_rrt_star` |
| BI-RRT\* | `rrt_planner` | `algorithm: bi_rrt_star` |

---

## Parameters

### Hybrid A\* (`hybrid_astar_planner`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `project_root` | (required) | Path to `hybrid_astar_planner/` project |
| `frame_id` | `"map"` | TF frame for published paths |
| `vehicle.wheel_base` | `2.5` | Vehicle wheelbase (m) |
| `vehicle.width` | `2.0` | Vehicle width (m) |
| `motion.step_size` | `2.0` | Motion primitive step (m) |
| `heuristic.type` | `"max"` | Heuristic: `euclidean`, `dubins`, `2d_astar`, `max` |
| `search.max_iterations` | `200000` | Max search nodes |

### RRT\* Family (`rrt_planner`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `project_root` | (required) | Path to `hybrid_astar_planner/` project |
| `algorithm` | `"rrt_star"` | Algorithm: `rrt_star`, `informed_rrt_star`, `bi_rrt_star` |
| `max_iterations` | `10000` | Sampling iterations |
| `step_size` | `2.0` | Steer distance (m) |
| `goal_radius` | `5.0` | Goal region radius (m) |
| `goal_bias` | `0.10` | Probability of sampling goal |
| `gamma` | `20.0` | RGG constant for rewire radius |
| `safety_margin` | `0.0` | Obstacle inflation (BI-RRT\* only, m) |

---

## Package Descriptions

### `av_planner_interfaces`
CMake package that generates ROS 2 interface types. Contains `PlanPath.action` (goal/result/feedback for planning requests) and `PlanningStats.msg` (timing and quality metrics). Used by both planner packages.

### `planner_common`
Python package with algorithm-agnostic utilities shared by both planners:
- **MapAdapter**: Converts `nav_msgs/OccupancyGrid` into a `MapHandler`-compatible object that the core Python algorithms expect. Uses duck-typing (same interface, different initialization).
- **Converters**: Pure data transformations between ROS types (`PoseStamped`, `Quaternion`) and core types (`State`).
- **mock_map_node**: Loads a PNG image, converts it to `OccupancyGrid` format, and publishes on `/map` with `TRANSIENT_LOCAL` QoS for standalone testing.

### `hybrid_astar_planner`
Lifecycle node wrapping the Hybrid A\* algorithm. On `configure`, imports the core algorithm from `src/hybrid_astar/` and creates the planner pipeline (MotionModel, VehicleFootprint, HeuristicCalculator, HybridAStar). On map receipt, rebuilds the planner with the new map.

### `rrt_planner`
Lifecycle node wrapping the RRT\* algorithm family. Supports three algorithms selectable via the `algorithm` parameter. On `configure`, imports all three planner classes. On map receipt, instantiates the selected planner. Algorithm switching via `ros2 param set` takes effect on the next map update.
