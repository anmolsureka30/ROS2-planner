#!/bin/bash
# All-in-one: Start planner, transition nodes, run visualizer, output images
# Usage: bash run_and_visualize.sh [sx sy syaw_deg gx gy gyaw_deg]
#   Default: (2,2,0°) -> (7,7,90°)

source /opt/ros/humble/setup.bash
source /root/ros2_ws/ros2_ws/install/setup.bash

PARAMS="$(ros2 pkg prefix hybrid_astar_bringup)/share/hybrid_astar_bringup/config/nav2_params.yaml"
MAP="$(ros2 pkg prefix hybrid_astar_bringup)/share/hybrid_astar_bringup/maps/test_field.yaml"

cleanup() {
    echo "Shutting down..."
    kill $TF_PID $MAP_PID $PLANNER_PID 2>/dev/null
    wait $TF_PID $MAP_PID $PLANNER_PID 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "=== Step 1: Starting nodes ==="
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id base_link &
TF_PID=$!

ros2 run nav2_map_server map_server --ros-args \
    --params-file "$PARAMS" -p yaml_filename:="$MAP" -p use_sim_time:=false &
MAP_PID=$!

ros2 run nav2_planner planner_server --ros-args \
    --params-file "$PARAMS" -p use_sim_time:=false &
PLANNER_PID=$!

echo "Waiting 6s for DDS..."
sleep 6

echo "=== Step 2: Lifecycle transitions ==="

wait_for_state() {
    local node=$1 target_id=$2 transition=$3
    for attempt in 1 2 3 4 5 6 7 8; do
        state=$(ros2 lifecycle get "$node" 2>&1 | head -1)
        state_id=$(echo "$state" | grep -oP '\[\K[0-9]+')
        if [ "$state_id" = "$target_id" ]; then
            echo "  $node -> $state [OK]"
            return 0
        fi
        echo "  $node: $state, applying $transition (try $attempt)..."
        ros2 lifecycle set "$node" "$transition" 2>&1 | grep -v "^$"
        sleep 3
    done
    echo "  FAILED: $node did not reach state $target_id"
    return 1
}

wait_for_state /map_server 2 configure
wait_for_state /map_server 3 activate
wait_for_state /planner_server 2 configure
wait_for_state /planner_server 3 activate

echo ""
echo "=== Step 3: Verify ==="
echo "  map_server:     $(ros2 lifecycle get /map_server 2>&1 | head -1)"
echo "  planner_server: $(ros2 lifecycle get /planner_server 2>&1 | head -1)"

# Check both are active
ms=$(ros2 lifecycle get /map_server 2>&1 | grep -oP '\[\K[0-9]+')
ps=$(ros2 lifecycle get /planner_server 2>&1 | grep -oP '\[\K[0-9]+')
if [ "$ms" != "3" ] || [ "$ps" != "3" ]; then
    echo "ERROR: Not all nodes active! map=$ms planner=$ps"
    exit 1
fi

echo ""
echo "=== Step 4: Planning + Visualization ==="
if [ $# -ge 6 ]; then
    python3 /root/visualize_planner.py "$1" "$2" "$3" "$4" "$5" "$6"
else
    python3 /root/visualize_planner.py
fi

echo ""
echo "=== Images saved! Copy to Mac with: ==="
echo "  docker cp bb70e7a4468f:/root/plan_diagonal.png ~/Desktop/ && open ~/Desktop/plan_diagonal.png"
echo "  docker cp bb70e7a4468f:/root/plan_reverse.png ~/Desktop/ && open ~/Desktop/plan_reverse.png"
echo ""
echo "Nodes still running. Send more goals or Ctrl+C to stop."
wait
