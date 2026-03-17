# Hybrid A* Path Planner with ROS 2 Integration

An autonomous vehicle path planner implementing the Hybrid A* algorithm (Dolgov et al. 2010) with a standalone ROS 2 middleware package for integration with real-world perception and control pipelines.

## Architecture

```
src/                              Pure Python algorithm (zero ROS imports)
  state.py                        SE(2) state: (x, y, theta, g, h, parent)
  map_handler.py                  Occupancy grid, collision checking, cost fields
  hybrid_astar/
    hybrid_astar.py               Main Hybrid A* planner (f = g + h)
    motion_model.py               Bicycle kinematics, vehicle footprint
    heuristic.py                  Euclidean, Dubins, 2D A*, Max heuristics
    reeds_shepp.py                48-path Reeds-Shepp curves
    visualizer.py                 Matplotlib visualization

ros2_ws/src/                      ROS 2 middleware (thin wrapper around algorithm)
  av_planner_interfaces/          Custom msg/action definitions
    msg/PlanningStats.msg         Planning metrics
    action/PlanPath.action        Start+Goal -> Path+Stats
  av_planner/                     Python ROS 2 nodes
    adapters/
      map_adapter.py              OccupancyGrid -> MapHandler-compatible
      converters.py               State <-> PoseStamped conversions
    nodes/
      planner_node.py             Lifecycle node with /plan_path action server
      mock_map_node.py            Test OccupancyGrid publisher from PNG

config/planner_config.yaml        Algorithm parameters (standalone mode)
maps/                             Test map images (PNG)
main.py                           Standalone entry point (no ROS)
benchmark.py                      Performance benchmarking
```

The core algorithm has **zero ROS dependencies**. The ROS 2 package is a middleware layer that translates between ROS messages and the algorithm's Python objects.

## Algorithm

Hybrid A* with:
- **Bicycle kinematic model** — exact arc geometry
- **Reeds-Shepp analytical expansion** — 48 path types for guaranteed convergence
- **Dual heuristic** — h = max(RS distance, cost-aware 2D Dijkstra)
- **Cost function** — f = g + h with steering, reversing, steering change, and direction switch penalties

## Standalone Usage (No ROS)

```bash
pip install -r requirements.txt
python main.py
```

Click on the map to set start and goal poses.

## ROS 2 Usage

### Prerequisites

- **ROS 2 Humble** (or later)
- Python packages: `numpy`, `scipy`, `Pillow`, `PyYAML`

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-geometry-msgs \
  ros-humble-nav-msgs python3-numpy python3-scipy python3-pil
```

### Setup

1. Edit `ros2_ws/src/av_planner/config/params.yaml`:
   - Set `project_root` to the absolute path of this repo on your machine
   - Set `map_image_path` to the absolute path of a map PNG (e.g., `<repo>/maps/map_open.png`)

2. Build:
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# Launch (standalone with mock map)
ros2 launch av_planner planner.launch.xml

# Configure and activate the lifecycle node
ros2 lifecycle set /planner_node configure
ros2 lifecycle set /planner_node activate

# Send a planning request
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 50.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 90.0, y: 50.0}, orientation: {w: 1.0}}}}"

# Verify
ros2 topic echo /planned_path --once
ros2 topic echo /planning_stats --once
```

### Integration with Real Systems

Launch without the mock map and provide a real `nav_msgs/OccupancyGrid` on `/map`:

```bash
ros2 launch av_planner planner.launch.xml use_mock_map:=false
```

## ROS 2 Interface

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/map` | `nav_msgs/OccupancyGrid` | Input |
| `/planned_path` | `nav_msgs/Path` | Output |
| `/planning_stats` | `PlanningStats` | Output |

### Action

| Action | Type |
|--------|------|
| `/plan_path` | `av_planner_interfaces/action/PlanPath` |

**Goal**: `start` (PoseStamped) + `goal` (PoseStamped)
**Result**: `path` (Path) + `stats` (PlanningStats) + `success` (bool) + `message` (string)
**Feedback**: `status` (string)

## Parameters

All parameters are in `ros2_ws/src/av_planner/config/params.yaml`. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicle.wheel_base` | 2.5 | Axle-to-axle distance (m) |
| `motion.max_steering_angle` | 40.0 | Max steering (degrees) |
| `motion.step_size` | 2.0 | Arc length per step (m) |
| `search.xy_resolution` | 1.0 | Grid cell size (m) |
| `search.theta_resolution` | 5.0 | Heading bin (degrees) |
| `cost.steering_penalty` | 1.5 | Steering magnitude cost |
| `cost.reversing_penalty` | 2.0 | Reverse motion multiplier |
| `heuristic.type` | "max" | max(RS, 2D Dijkstra) |

## Documentation

See `docs/ros2_guide.pdf` for a detailed explanation of ROS 2 concepts as applied to this project.

## License

Apache-2.0
