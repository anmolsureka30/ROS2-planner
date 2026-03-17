# Hybrid A* Path Planner — ROS 2 Package

An independent ROS 2 Python package wrapping the pure-Python Hybrid A* path planner. Designed as a standalone planning module that can plug into any perception/control pipeline via standard ROS 2 topics and actions.

**Not tied to Nav2, Gazebo, or any simulator.**

## Architecture

```
av_planner_interfaces/        Custom msg/action definitions
  msg/PlanningStats.msg        Planning metrics
  action/PlanPath.action       Start+Goal → Path+Stats

av_planner/                    Python nodes + adapters
  adapters/
    map_adapter.py             OccupancyGrid → MapHandler-compatible
    converters.py              State ↔ PoseStamped, Path conversions
  nodes/
    planner_node.py            Lifecycle node: /map subscriber + /plan_path action server
    mock_map_node.py           Publishes test OccupancyGrid from PNG
```

The core algorithm lives in `../src/` (pure Python, zero ROS imports). The ROS nodes are thin wrappers.

## Prerequisites

- **ROS 2 Humble** (or later)
- Python packages: `numpy`, `scipy`, `Pillow`, `PyYAML`

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-geometry-msgs \
  ros-humble-nav-msgs python3-numpy python3-scipy python3-pil
```

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### 1. Launch (standalone with mock map)

```bash
ros2 launch av_planner planner.launch.xml
```

This starts:
- `mock_map_node` — publishes OccupancyGrid from `maps/map_open.png`
- `planner_node` — lifecycle node (starts unconfigured)

### 2. Configure and activate the planner

```bash
ros2 lifecycle set /planner_node configure
ros2 lifecycle set /planner_node activate
```

### 3. Send a planning request

```bash
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 50.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 90.0, y: 50.0}, orientation: {w: 1.0}}}}"
```

### 4. Verify

```bash
ros2 topic echo /planned_path --once    # See the planned path
ros2 topic echo /planning_stats --once  # See planning metrics
```

## Integration with Real Systems

When connecting to a real perception/control stack:

1. **Disable mock map**: Launch with `use_mock_map:=false`
   ```bash
   ros2 launch av_planner planner.launch.xml use_mock_map:=false
   ```

2. **Provide a real map**: Your SLAM or map_server publishes `nav_msgs/OccupancyGrid` on `/map`

3. **Send planning goals**: Your mission planner calls the `/plan_path` action

4. **Consume the path**: Your controller subscribes to `/planned_path` (`nav_msgs/Path`)

### Path subscriber example

```python
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Path

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.create_subscription(Path, '/planned_path', self.path_cb, 10)

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

## ROS 2 Interface

### Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Reliable, Transient Local | Input: occupancy grid |
| `/planned_path` | `nav_msgs/Path` | Reliable | Output: planned path |
| `/planning_stats` | `av_planner_interfaces/PlanningStats` | Reliable | Output: planning metrics |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/plan_path` | `av_planner_interfaces/action/PlanPath` | Plan from start to goal |

### Action Fields

**Goal**: `start` (PoseStamped) + `goal` (PoseStamped)
**Result**: `path` (Path) + `stats` (PlanningStats) + `success` (bool) + `message` (string)
**Feedback**: `status` (string)

## Parameters

All parameters are in `config/params.yaml`:

| Group | Parameter | Default | Description |
|-------|-----------|---------|-------------|
| Vehicle | `wheel_base` | 2.5 | Axle-to-axle distance (m) |
| Vehicle | `length` | 4.5 | Vehicle length (m) |
| Vehicle | `width` | 2.0 | Vehicle width (m) |
| Motion | `max_steering_angle` | 40.0 | Max steering (degrees) |
| Motion | `step_size` | 2.0 | Arc length per step (m) |
| Motion | `num_steering_angles` | 5 | Steering discretization |
| Search | `xy_resolution` | 1.0 | Grid cell size (m) |
| Search | `theta_resolution` | 5.0 | Heading bin (degrees) |
| Search | `shot_distance` | 30.0 | RS expansion range (m) |
| Cost | `steering_penalty` | 1.5 | Steering magnitude cost |
| Cost | `reversing_penalty` | 2.0 | Reverse motion multiplier |
| Cost | `direction_switch_penalty` | 10.0 | Gear change cost |
| Heuristic | `type` | "max" | max(RS, 2D Dijkstra) |

## Algorithm

Hybrid A* (Dolgov et al. 2010) with:
- **Bicycle kinematic model** — exact arc geometry
- **Reeds-Shepp analytical expansion** — 48 path types for guaranteed convergence
- **Dual heuristic** — h = max(RS distance, cost-aware 2D Dijkstra)
- **Cost function** — f = g + h with 4 penalties (steering, reversing, steering change, direction switch)

## License

Apache-2.0
