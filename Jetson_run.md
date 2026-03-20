Jetson Setup & Run Guide
1. Prerequisites (one-time)

# ── Terminal 1: Install ROS 2 (if not already installed) ──

# Check if ROS 2 is installed
ros2 --version

# If not installed — Ubuntu 22.04 = Humble, Ubuntu 24.04 = Jazzy
# For Jetson (Ubuntu 22.04 typically):
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install Python dependencies for the core algorithms
pip3 install numpy matplotlib Pillow PyYAML scipy
2. Clone & Build

# ── Terminal 1: Clone and build ──

# Clone the repo
cd ~
git clone https://github.com/anmolsureka30/ROS2-planner.git
cd ROS2-planner

# IMPORTANT: Update config files with YOUR Jetson path
# Replace the Mac path with your actual path
sed -i 's|/Users/anmolsureka/Documents/hybrid_astar_planner|'$(pwd)'|g' \
    ros2_ws/src/hybrid_astar_planner/config/params.yaml \
    ros2_ws/src/rrt_planner/config/params.yaml

# Verify the paths are correct
grep "project_root\|map_image_path" ros2_ws/src/*/config/params.yaml

# Build the ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
After colcon build, you should see:


Starting >>> av_planner_interfaces
Starting >>> planner_common
Finished <<< av_planner_interfaces
Starting >>> hybrid_astar_planner
Starting >>> rrt_planner
Finished <<< planner_common
Finished <<< hybrid_astar_planner
Finished <<< rrt_planner

Summary: 4 packages finished
3. Run — Hybrid A* Planner (4 terminals)

# ══════════════════════════════════════════════════
# TERMINAL 1: Launch planner + mock map
# ══════════════════════════════════════════════════
cd ~/ROS2-planner/ros2_ws
source install/setup.bash

ros2 launch hybrid_astar_planner planner.launch.xml

# Expected output:
#   [mock_map_node] Publishing map from .../maps/map_open.png (200x200, res=0.5m)
#   [hybrid_astar_node] Configuring...

# ══════════════════════════════════════════════════
# TERMINAL 2: Configure + Activate the lifecycle node
# ══════════════════════════════════════════════════
source /opt/ros/humble/setup.bash

# Step 1: Configure (loads core imports, creates subscribers/publishers)
ros2 lifecycle set /planner_node configure

# Step 2: Activate (enables planning)
ros2 lifecycle set /planner_node activate

# Check state
ros2 lifecycle get /planner_node
# Expected: "active [3]"

# ══════════════════════════════════════════════════
# TERMINAL 3: Send a planning request
# ══════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash

# Plan: (10, 50, 0°) → (90, 50, 0°) — straight across open map
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 50.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    goal: {pose: {position: {x: 90.0, y: 50.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback

# Expected in Terminal 1:
#   Planning: (10.0, 50.0, 0°) → (90.0, 50.0, 0°)
#   Path found: 80.0m, 27 nodes, 0.189s

# More complex: diagonal with heading
ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    goal: {pose: {position: {x: 85.0, y: 85.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.38, w: 0.92}}}}" \
  --feedback

# ══════════════════════════════════════════════════
# TERMINAL 4: Monitor topics (verify data flow)
# ══════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash

# List all active topics
ros2 topic list

# Expected:
#   /map
#   /plan_path/_action/feedback
#   /plan_path/_action/status
#   /planned_path
#   /planning_stats

# Watch the planned path output
ros2 topic echo /planned_path --once

# Watch stats
ros2 topic echo /planning_stats --once

# Check node graph
ros2 node list
#   /mock_map_node
#   /planner_node
4. Run — RRT* Planner (same 4-terminal pattern)

# ══════════════════════════════════════════════════
# TERMINAL 1: Launch RRT* planner
# ══════════════════════════════════════════════════
cd ~/ROS2-planner/ros2_ws
source install/setup.bash

# Default: rrt_star algorithm
ros2 launch rrt_planner rrt_planner.launch.xml

# ══════════════════════════════════════════════════
# TERMINAL 2: Configure + Activate
# ══════════════════════════════════════════════════
source /opt/ros/humble/setup.bash

ros2 lifecycle set /rrt_planner_node configure
ros2 lifecycle set /rrt_planner_node activate

# ══════════════════════════════════════════════════
# TERMINAL 3: Send planning request
# ══════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
cd ~/ROS2-planner/ros2_ws && source install/setup.bash

ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath \
  "{start: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    goal: {pose: {position: {x: 80.0, y: 80.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.38, w: 0.92}}}}" \
  --feedback
5. Switch RRT* Algorithm (live, no restart)

# ── In any terminal ──
source /opt/ros/humble/setup.bash

# Switch to Informed RRT*
ros2 param set /rrt_planner_node algorithm informed_rrt_star

# Switch to BI-RRT*
ros2 param set /rrt_planner_node algorithm bi_rrt_star

# Switch back to RRT*
ros2 param set /rrt_planner_node algorithm rrt_star

# Then send another goal — it uses the new algorithm
6. Change Map (live)

# ── In Terminal 3 ──
# Kill mock_map_node (Ctrl+C in Terminal 1), then relaunch with different map:

ros2 run planner_common mock_map_node --ros-args \
  -p map_image_path:="$HOME/ROS2-planner/maps/map_warehouse.png" \
  -p resolution:=0.5 \
  -p obstacle_threshold:=128

# The planner auto-rebuilds when it receives the new /map message
7. Run Standalone (no ROS — matplotlib simulation)

# ── Single terminal ──
cd ~/ROS2-planner

# Interactive Hybrid A* (click start/goal on map)
python3 main.py

# Benchmark all scenarios
python3 benchmark.py

# RRT* simulation with live tree growth
python3 scripts/rrt_simulation.py --algorithm rrt_star
python3 scripts/rrt_simulation.py --algorithm informed_rrt_star
python3 scripts/rrt_simulation.py --algorithm bi_rrt_star
python3 scripts/rrt_simulation.py --compare
8. Verify Everything Works (checklist)

# ── Run these in Terminal 4 to verify ──
source /opt/ros/humble/setup.bash

# 1. Check all 4 packages built
ros2 pkg list | grep -E "av_planner|hybrid_astar|rrt_planner|planner_common"
# Expected: 4 packages

# 2. Check nodes are running
ros2 node list
# Expected: /mock_map_node + /planner_node (or /rrt_planner_node)

# 3. Check lifecycle state
ros2 lifecycle get /planner_node   # or /rrt_planner_node
# Expected: "active [3]"

# 4. Check action server is available
ros2 action list
# Expected: /plan_path

# 5. Check topics
ros2 topic list
ros2 topic info /map              # Should show 1 publisher
ros2 topic info /planned_path     # Should show 1 publisher

# 6. Check parameters
ros2 param list /planner_node     # Hybrid A*
ros2 param list /rrt_planner_node # RRT*
Architecture — What's Communicating

┌─────────────────┐    /map (OccupancyGrid)    ┌──────────────────────┐
│  mock_map_node  │ ──────────────────────────▶ │  planner_node        │
│  (or perception)│    TRANSIENT_LOCAL QoS      │  (Hybrid A* or RRT*) │
└─────────────────┘                             └────────┬─────────────┘
                                                         │
                    /plan_path (Action)                   │
 ┌──────────┐    ◀─── goal/feedback/result ───▶          │
 │  Client   │                                           │
 │ (you/ctrl)│                                 ┌─────────▼───────────┐
 └──────────┘                                  │ /planned_path (Path)│
                                               │ /planning_stats     │
                                               └─────────────────────┘
                                                         │
                                          ┌──────────────▼──────────────┐
                                          │  Controller node (future)   │
                                          │  subscribes to /planned_path│
                                          └─────────────────────────────┘
When you connect real perception/controller later, you just:

Replace mock_map_node with your perception node publishing /map
Have your controller subscribe to /planned_path
The planner node stays unchanged — true plug-and-play