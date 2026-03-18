# Hybrid A* Path Planner

A complete implementation of the Hybrid A* algorithm for kinematically feasible path planning in obstacle environments.

## 📋 Overview

This implementation provides a production-ready Hybrid A* path planner with:

- ✅ **Complete SE(2) search**: Plans in (x, y, θ) space
- ✅ **Vehicle kinematics**: Bicycle model with steering constraints
- ✅ **Multiple heuristics**: Euclidean, Reeds-Shepp, 2D A*, and max
- ✅ **Analytical expansion**: Direct shots to goal using Reeds-Shepp paths
- ✅ **Collision checking**: Full vehicle footprint collision detection
- ✅ **Configurable parameters**: All parameters adjustable via YAML config
- ✅ **Visualization tools**: Beautiful plots and animations
- ✅ **Interactive mode**: Test custom start/goal configurations

## 🗂️ Project Structure

```
hybrid_astar_planner/
├── config/
│   └── planner_config.yaml          # Configuration file
├── src/
│   ├── __init__.py
│   ├── state.py                     # State representation
│   ├── map_handler.py               # Map loading and processing
│   ├── motion_model.py              # Vehicle kinematics
│   ├── reeds_shepp.py               # Analytical path computation
│   ├── heuristic.py                 # Heuristic calculations
│   ├── hybrid_astar.py              # Main planner algorithm
│   └── visualizer.py                # Visualization tools
├── maps/
│   └── map_maze.png                 # Obstacle map
├── results/                         # Output directory
├── main.py                          # Main execution script
├── requirements.txt                 # Python dependencies
└── README.md                        # This file
```

## 🚀 Quick Start

### Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Run default scenarios
python main.py

# Run interactive mode
python main.py --interactive
```

### Basic Usage

```python
from src import State, MapHandler, MotionModel, VehicleFootprint
from src import HeuristicCalculator, HybridAStar, Visualizer
import numpy as np

# Load map
map_handler = MapHandler("maps/map_maze.png", resolution=0.2)

# Setup vehicle
motion_model = MotionModel(wheel_base=2.5, max_steering_angle=40.0)
vehicle_footprint = VehicleFootprint(length=4.5, width=2.0)

# Setup heuristic
heuristic = HeuristicCalculator(map_handler, heuristic_type="max")

# Create planner
planner = HybridAStar(map_handler, motion_model, vehicle_footprint, heuristic)

# Define start and goal
start = State(x=10.0, y=10.0, theta=np.radians(0))
goal = State(x=90.0, y=90.0, theta=np.radians(45))

# Plan path
result = planner.plan(start, goal)

if result:
    path, info = result
    print(f"Path found! Length: {info['path_length']:.2f} m")
```

## ⚙️ Configuration

All parameters are configurable in `config/planner_config.yaml`:

### Map Settings
- `resolution`: Meters per pixel (default: 0.2)
- `obstacle_threshold`: Grayscale threshold for obstacles (default: 128)

### Vehicle Parameters
- `wheel_base`: Distance between axles in meters (default: 2.5)
- `length`: Total vehicle length (default: 4.5)
- `width`: Vehicle width (default: 2.0)
- `rear_axle_to_back`: Rear overhang (default: 1.0)

### Motion Model
- `max_steering_angle`: Maximum steering in degrees (default: 40)
- `step_size`: Motion primitive length in meters (default: 0.5)
- `num_steering_angles`: Discrete steering angles (default: 3)
- `allow_reverse`: Enable reverse motion (default: true)

### Search Parameters
- `xy_resolution`: Grid discretization in meters (default: 0.5)
- `theta_resolution`: Angular discretization in degrees (default: 5)
- `shot_distance`: Analytical expansion threshold (default: 10.0)

### Cost Parameters
- `steering_penalty`: Turning cost multiplier (default: 1.5)
- `reversing_penalty`: Reverse cost multiplier (default: 2.0)
- `steering_change_penalty`: Direction change penalty (default: 15.0)

### Heuristic Settings
- `type`: Heuristic type
  - `"euclidean"`: Fast, least accurate
  - `"dubins"`: Non-holonomic distance
  - `"2d_astar"`: With obstacles
  - `"max"`: Best of both (recommended)
- `turning_radius`: For Reeds-Shepp paths (default: 5.0)

## 🎯 Key Features

### 1. **State Representation (SE(2))**
States are represented as (x, y, θ) poses in the configuration space.

### 2. **Kinematic Constraints**
Uses bicycle model:
```
x' = x + d * cos(θ)
y' = y + d * sin(θ)
θ' = θ + d/L * tan(φ)
```

### 3. **Motion Primitives**
- Forward/Reverse straight
- Forward/Reverse left turn
- Forward/Reverse right turn

### 4. **Heuristic Functions**

**Euclidean Distance** (Fast):
```
h = sqrt((x_goal - x)² + (y_goal - y)²)
```

**Reeds-Shepp Distance** (Non-holonomic):
- Computes shortest path considering vehicle kinematics
- Ignores obstacles (admissible)

**2D A\*** (With obstacles):
- Computes shortest holonomic path
- Considers obstacles (admissible)

**Max Heuristic** (Recommended):
```
h = max(h_reeds_shepp, h_2d_astar)
```

### 5. **Analytical Expansion**
When close to goal, attempts direct Reeds-Shepp path:
- If collision-free → terminate (optimal!)
- Greatly speeds up search in open areas

### 6. **Collision Checking**
- Full vehicle footprint checking
- Dense point sampling for accuracy
- Bresenham line algorithm for efficiency

## 📊 Output Format

### Planning Result
```python
path, info = planner.plan(start, goal)

# path: List of State objects
# info: Dictionary with:
{
    'success': True/False,
    'path_length': float,        # meters
    'nodes_expanded': int,
    'nodes_visited': int,
    'search_time': float,        # seconds
    'analytical': True/False     # if analytical shot succeeded
}
```

### Visualization
Results are automatically saved to `results/` directory:
- PNG images with complete planning visualization
- Vehicle footprints along path
- Planning statistics overlay
- Start/goal markers with heading arrows

## 🔧 Advanced Usage

### Custom Heuristic
```python
from src.heuristic import HeuristicCalculator

# Fast planning
heuristic = HeuristicCalculator(map_handler, heuristic_type="euclidean")

# Accurate planning
heuristic = HeuristicCalculator(map_handler, heuristic_type="2d_astar")

# Balanced (recommended)
heuristic = HeuristicCalculator(map_handler, heuristic_type="max")
```

### Adjusting Cost Functions
```python
# More penalty for turning
planner = HybridAStar(..., steering_penalty=2.5)

# Heavily penalize reverse
planner = HybridAStar(..., reversing_penalty=5.0)

# Discourage direction changes
planner = HybridAStar(..., steering_change_penalty=30.0)
```

### Different Vehicle Types

**Small car:**
```yaml
vehicle:
  wheel_base: 2.5
  length: 4.0
  width: 1.8
```

**Large truck:**
```yaml
vehicle:
  wheel_base: 4.0
  length: 8.0
  width: 2.5
```

## 📈 Performance Tips

1. **Faster Search**:
   - Use `heuristic_type: "euclidean"`
   - Increase `xy_resolution` (coarser grid)
   - Increase `theta_resolution` (fewer angle bins)

2. **Better Paths**:
   - Use `heuristic_type: "max"`
   - Decrease `xy_resolution` (finer grid)
   - Decrease `theta_resolution` (more angle bins)

3. **Smoother Paths**:
   - Decrease `step_size`
   - Increase `num_steering_angles`
   - Lower `steering_penalty`

## 🐛 Troubleshooting

**No path found:**
- Check if start/goal are collision-free
- Increase `shot_distance`
- Reduce vehicle size
- Coarsen grid resolution

**Path too rough:**
- Decrease `step_size`
- Increase `num_steering_angles`
- Use smoother heuristic

**Too slow:**
- Use "euclidean" heuristic
- Increase `xy_resolution`
- Decrease `shot_distance`

## 📚 Algorithm Details

### Core Algorithm
```
1. Initialize open set with start state
2. While open set not empty:
   a. Pop state with lowest f = g + h
   b. Check goal reached
   c. Try analytical expansion (Reeds-Shepp shot)
   d. Expand neighbors using motion primitives
   e. Prune states with higher cost in same cell
3. Reconstruct path from goal to start
```

### Cost Function
```
f(state) = g(state) + h(state)

g(state) = accumulated cost from start
         = sum of motion costs with penalties

h(state) = heuristic estimate to goal
         = max(h_nonholonomic, h_holonomic)
```

## 🎓 References

1. **Original Paper**:
   Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2010). 
   "Practical search techniques in path planning for autonomous driving."

2. **Reeds-Shepp Paths**:
   Reeds, J. A., & Shepp, L. A. (1990). 
   "Optimal paths for a car that goes both forwards and backwards."

3. **Hybrid A\***:
   Pivtoraiko, M., Knepper, R. A., & Kelly, A. (2009).
   "Differentially constrained mobile robot motion planning in state lattices."

## ROS 2 Integration

The project includes a complete ROS 2 middleware layer in `ros2_ws/` that wraps both Hybrid A\* and RRT\* family planners as independent, pluggable nodes.

### Supported Algorithms via ROS 2

| Algorithm | Package | Selection |
|-----------|---------|-----------|
| Hybrid A\* | `hybrid_astar_planner` | Launch `planner.launch.xml` |
| RRT\* | `rrt_planner` | `algorithm: rrt_star` |
| Informed RRT\* | `rrt_planner` | `algorithm: informed_rrt_star` |
| BI-RRT\* | `rrt_planner` | `algorithm: bi_rrt_star` |

### Quick Start

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch Hybrid A*
ros2 launch hybrid_astar_planner planner.launch.xml

# OR launch RRT*
ros2 launch rrt_planner rrt_planner.launch.xml
```

Both planners serve the same `/plan_path` action and publish on `/planned_path`, making them plug-and-play interchangeable for downstream perception/control nodes.

See [`ros2_ws/src/README.md`](ros2_ws/src/README.md) for full documentation including pipeline integration, parameter reference, and lifecycle management.

---

## RRT\* Family (Sampling-Based Planners)

In addition to Hybrid A\*, this project implements three RRT\* variants in `src/rrt_star/`:

| Algorithm | Key Feature |
|-----------|-------------|
| **RRT\*** | Asymptotically optimal rewiring |
| **Informed RRT\*** | Ellipsoidal sampling after first solution |
| **BI-RRT\*** | Bidirectional search + pruning + informed optimization |

```bash
# Interactive mode (choose algorithm)
python main.py

# Real-time tree growth simulation
python scripts/rrt_simulation.py --algorithm rrt_star
python scripts/rrt_simulation.py --compare
```

---

## 📄 License

MIT License - Feel free to use for research or commercial applications.

## 📧 Contact

For questions or issues, please open a GitHub issue.