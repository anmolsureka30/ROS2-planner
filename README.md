# Autonomous Vehicle Path Planning System

A modular path planning system for autonomous vehicles with multiple algorithm families, interactive visualization, and a ROS 2 middleware layer for real-world deployment.

## Algorithms

| Algorithm | Type | Key Feature | Optimal |
|-----------|------|-------------|---------|
| **Hybrid A*** | Graph-based, lattice search | Non-holonomic, analytical expansion | Resolution-optimal |
| **RRT*** | Sampling-based, tree search | Rewiring for cost reduction | Asymptotically optimal |
| **Informed RRT*** | Sampling-based, focused | Ellipsoidal sampling after first solution | Asymptotically optimal |
| **BI-RRT*** | Sampling-based, bidirectional | Fast initial solution + pruning + optimization | Asymptotically optimal |

## Project Structure

```
src/                                 # Core algorithms (zero ROS dependency)
  state.py                           # SE(2) state: (x, y, theta, g, h)
  map_handler.py                     # Occupancy grid, collision checking, cost fields
  hybrid_astar/                      # Hybrid A* algorithm
    hybrid_astar.py                  #   A* search with Reeds-Shepp analytical expansion
    motion_model.py                  #   Bicycle kinematics, vehicle footprint
    reeds_shepp.py                   #   48-path Reeds-Shepp curves
    heuristic.py                     #   Euclidean, Dubins, 2D A*, max heuristics
    visualizer.py                    #   Matplotlib visualization
  rrt_star/                          # RRT* family algorithms
    rrt_star.py                      #   Base RRT* with X_soln re-evaluation
    informed_rrt_star.py             #   Informed RRT* (ellipsoidal sampling)
    bi_rrt_star.py                   #   BI-RRT* (bidirectional + pruning + informed)
    node.py, tree.py                 #   Tree data structures
    collision_checker.py             #   MapHandler adapter with safety margin
    steering.py                      #   Straight-line steer function
    nearest_neighbor.py              #   KD-tree wrapper (scipy.spatial)
    samplers.py                      #   Uniform, GoalBiased, Informed samplers

config/planner_config.yaml           # All parameters for all algorithms
maps/                                # Obstacle map images (PNG)
scripts/                             # RRT* simulation and benchmarking
ros2_ws/                             # ROS 2 middleware (see below)
main.py                              # Interactive entry point
benchmark.py                         # Performance benchmarking
```

## Quick Start

### Installation

```bash
pip install -r requirements.txt
```

Dependencies: `numpy`, `matplotlib`, `Pillow`, `PyYAML`, `scipy`

### Run Interactive Mode

```bash
python main.py
```

Click on the map to set start/goal, select an algorithm (Hybrid A*, RRT*, Informed RRT*, BI-RRT*), and view results with timing and cost statistics.

### Run Benchmarks

```bash
python benchmark.py
```

### Run RRT* Simulation (live tree growth)

```bash
python scripts/rrt_simulation.py --algorithm rrt_star
python scripts/rrt_simulation.py --algorithm informed_rrt_star
python scripts/rrt_simulation.py --algorithm bi_rrt_star
python scripts/rrt_simulation.py --compare
```

## Algorithm Details

### Hybrid A*

Graph-based search in SE(2) space using bicycle kinematics.

- **Cost function**: `f(s) = g(s) + h(s)` with penalties for steering, reversing, and direction changes
- **Heuristic**: `h = max(Reeds-Shepp distance, cost-aware 2D Dijkstra)` — tight, admissible
- **Analytical expansion**: When near the goal, attempts direct Reeds-Shepp connection (48 path types)
- **Vehicle model**: Bicycle kinematics with exact arc geometry

### RRT*

Sampling-based asymptotically optimal planner (Karaman & Frazzoli, IJRR 2011).

- Maintains X_soln (all goal-reaching nodes) and re-evaluates best cost after every rewire
- KD-tree nearest neighbors with hybrid buffer for O(log n) queries
- Convergence detection: stops early when cost plateaus (configurable patience)

### Informed RRT*

Extension of RRT* with focused sampling (Gammell et al., IROS 2014).

- Before first solution: goal-biased uniform sampling
- After first solution: samples from prolate hyperellipsoid containing all potentially improving states
- Ellipse shrinks as better solutions are found, focusing exploration

### BI-RRT*

Three-phase pipeline (Fan et al., Heliyon 2024).

1. **Bidirectional RRT-Connect**: Two trees grow toward each other for fast initial solution
2. **Path pruning**: Greedy farthest-visible waypoint skipping
3. **Informed optimization**: Informed RRT* with pruned cost as initial ellipse bound

## Configuration

All parameters in `config/planner_config.yaml`:

### Vehicle
| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_base` | 2.5 m | Distance between axles |
| `length` | 4.5 m | Vehicle length |
| `width` | 2.0 m | Vehicle width |

### RRT* Family
| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iterations` | 10000 | Maximum sampling iterations |
| `step_size` | 2.0 m | Steer distance |
| `goal_radius` | 5.0 m | Goal region radius |
| `goal_bias` | 0.15 | Probability of sampling goal |
| `gamma` | 20.0 | RGG constant for rewiring radius |
| `convergence_patience` | 2000 | Stop if no improvement for N iters (0 = disabled) |

### Hybrid A*
| Parameter | Default | Description |
|-----------|---------|-------------|
| `xy_resolution` | 1.0 m | Grid cell size |
| `theta_resolution` | 5.0 deg | Heading discretization |
| `shot_distance` | 30.0 m | Analytical expansion range |
| `heuristic.type` | "max" | Heuristic: euclidean, dubins, 2d_astar, max |

## ROS 2 Integration

The project includes a complete ROS 2 middleware layer in `ros2_ws/` with 4 packages:

| Package | Role |
|---------|------|
| `av_planner_interfaces` | PlanPath action + PlanningStats message definitions |
| `planner_common` | MapAdapter, converters, mock_map_node, path_visualizer_node |
| `hybrid_astar_planner` | Lifecycle node wrapping Hybrid A* |
| `rrt_planner` | Lifecycle node wrapping RRT*/Informed RRT*/BI-RRT* |

Both planners serve the same `/plan_path` action topic — swap by launching a different planner. Downstream perception/control nodes work unchanged.

### Build & Run

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Hybrid A*
ros2 launch hybrid_astar_planner planner.launch.xml

# OR RRT* family
ros2 launch rrt_planner rrt_planner.launch.xml
```

### Visualize Results

```bash
ros2 run planner_common path_visualizer_node
```

### Switch Algorithm (live, no restart)

```bash
ros2 param set /rrt_planner_node algorithm informed_rrt_star
ros2 param set /rrt_planner_node algorithm bi_rrt_star
```

See [ros2_ws/src/README.md](ros2_ws/src/README.md) for full ROS 2 documentation.
See [Jetson_run.md](Jetson_run.md) for Jetson deployment guide.

## References

1. Dolgov et al. (2010). "Practical search techniques in path planning for autonomous driving."
2. Reeds & Shepp (1990). "Optimal paths for a car that goes both forwards and backwards."
3. Karaman & Frazzoli (2011). "Sampling-based algorithms for optimal motion planning." IJRR.
4. Gammell, Srinivasa & Barfoot (2014). "Informed RRT*." IROS.
5. Fan et al. (2024). "BI-RRT*: An improved path planning algorithm." Heliyon.
