# Jetson Setup & Run Guide

Complete guide for running all path planners (Hybrid A*, RRT*, Informed RRT*, BI-RRT*) on Jetson with ROS 2.

---

## 1. Prerequisites (one-time)

```bash
# Check if ROS 2 is installed
ros2 --version

# If not installed (Ubuntu 22.04 = Humble):
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-tk
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Python dependencies
pip3 install numpy matplotlib Pillow PyYAML scipy
```

## 2. Clone & Build

```bash
cd ~
git clone https://github.com/anmolsureka30/ROS2-planner.git
cd ROS2-planner

# Update config paths to your Jetson
sed -i 's|/Users/anmolsureka/Documents/hybrid_astar_planner|'$(pwd)'|g' \
    ros2_ws/src/hybrid_astar_planner/config/params.yaml \
    ros2_ws/src/rrt_planner/config/params.yaml

# Verify paths
grep "project_root\|map_image_path" ros2_ws/src/*/config/params.yaml

# Build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Expected output: `Summary: 4 packages finished`

---

## 3. Test Hybrid A* (4 terminals)

### Terminal 1 — Launch
```bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash
ros2 launch hybrid_astar_planner planner.launch.xml
```

### Terminal 2 — Configure + Activate
```bash
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash
ros2 lifecycle set /hybrid_astar_node configure
ros2 lifecycle set /hybrid_astar_node activate
```

### Terminal 3 — Visualizer (shows map + path)
```bash
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash
ros2 run planner_common path_visualizer_node
```

### Terminal 4 — Send goals
```bash
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash

# Straight run: (10,50) -> (90,50)
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 50.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 90.0, y: 50.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# Diagonal with heading: (10,10) -> (85,85)
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 85.0, y: 85.0, z: 0.0}, orientation: {z: 0.38, w: 0.92}}}}" \
  --feedback
```

---

## 4. Test RRT* (kill Terminal 1 first with Ctrl+C)

### Terminal 1 — Launch RRT*
```bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash
ros2 launch rrt_planner rrt_planner.launch.xml
```

### Terminal 2 — Configure + Activate
```bash
ros2 lifecycle set /rrt_planner_node configure
ros2 lifecycle set /rrt_planner_node activate
```

### Terminal 3 — Visualizer (same command, no change needed)
```bash
ros2 run planner_common path_visualizer_node
```

### Terminal 4 — Send goal
```bash
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 80.0, y: 80.0, z: 0.0}, orientation: {z: 0.38, w: 0.92}}}}" \
  --feedback
```

---

## 5. Test Informed RRT* (no restart needed)

Switch the algorithm live, then send a new goal:

```bash
# In any terminal
ros2 param set /rrt_planner_node algorithm informed_rrt_star

# Send goal (Terminal 4)
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 80.0, y: 80.0, z: 0.0}, orientation: {z: 0.38, w: 0.92}}}}" \
  --feedback
```

---

## 6. Test BI-RRT* (no restart needed)

```bash
# Switch algorithm
ros2 param set /rrt_planner_node algorithm bi_rrt_star

# Send goal (Terminal 4)
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {pose: {position: {x: 80.0, y: 80.0, z: 0.0}, orientation: {z: 0.38, w: 0.92}}}}" \
  --feedback
```

---

## 7. Change Map (live)

```bash
# Kill mock_map_node (Ctrl+C in Terminal 1), then:
ros2 run planner_common mock_map_node --ros-args \
  -p map_image_path:="$HOME/ROS2-planner/maps/map_warehouse.png" \
  -p resolution:=0.5 \
  -p obstacle_threshold:=128
```

Available maps: `map_open.png`, `map_maze.png`, `map_maze_gen.png`, `map_parking.png`, `map_warehouse.png`

---

## 8. Standalone Mode (no ROS, matplotlib)

```bash
cd ~/ROS2-planner

# Interactive Hybrid A* (click start/goal)
python3 main.py

# Benchmark
python3 benchmark.py

# RRT* simulation (live tree growth)
python3 scripts/rrt_simulation.py --algorithm rrt_star
python3 scripts/rrt_simulation.py --algorithm informed_rrt_star
python3 scripts/rrt_simulation.py --algorithm bi_rrt_star
python3 scripts/rrt_simulation.py --compare
```

---

## 9. Verification Checklist

```bash
# Check all 4 packages built
ros2 pkg list | grep -E "av_planner|hybrid_astar|rrt_planner|planner_common"

# Check nodes running
ros2 node list

# Check lifecycle state
ros2 lifecycle get /hybrid_astar_node   # or /rrt_planner_node

# Check action server
ros2 action list    # should show /plan_path

# Check topics
ros2 topic list
ros2 topic echo /planned_path --once
ros2 topic echo /planning_stats --once

# Check parameters
ros2 param list /rrt_planner_node
```

---

## Architecture

```
+------------------+    /map (OccupancyGrid)    +-------------------------+
|  mock_map_node   | -------------------------> |  hybrid_astar_node      |
|  (or perception) |    TRANSIENT_LOCAL QoS     |  OR rrt_planner_node    |
+------------------+                            +----------+--------------+
                                                           |
                     /plan_path (Action)                   |
 +----------+     <-- goal/feedback/result -->             |
 |  Client   |                                  +----------v--------------+
 | (you/ctrl)|                                  | /planned_path (Path)    |
 +----------+                                   | /planning_stats         |
                                                +----------+--------------+
                                                           |
 +------------------+                           +----------v--------------+
 | path_visualizer  | <-- /map, /planned_path   | Controller (future)     |
 | (matplotlib)     |                           | subscribes /planned_path|
 +------------------+                           +-------------------------+
```

Both planners serve the same `/plan_path` action topic -- swap by launching a different planner. The visualizer and any downstream controller work unchanged.
