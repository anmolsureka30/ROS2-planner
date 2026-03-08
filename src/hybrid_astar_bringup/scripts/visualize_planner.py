#!/usr/bin/env python3
"""
Interactive Hybrid A* Planner Visualizer
========================================
Runs INSIDE the Docker container. Saves output as images since there's no display.

Usage (inside Docker):
  python3 /root/visualize_planner.py

This script:
  1. Subscribes to /map, /plan, /global_costmap/costmap topics
  2. Sends planning goals via the /compute_path_to_pose action
  3. Renders the map + path + start/goal as a PNG image
  4. Saves to /root/planner_output.png (docker cp to view on host)

Interactive mode (if display available):
  - Click on the map to set start (left-click) and goal (right-click)
"""

import sys
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

# Try matplotlib with non-interactive backend for headless mode
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch


class PlannerVisualizer(Node):
    def __init__(self):
        super().__init__('planner_visualizer')

        self.map_data = None
        self.map_info = None
        self.costmap_data = None
        self.path_poses = None
        self.start = None
        self.goal = None

        # QoS for map topics (transient local)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, map_qos)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        # Action client
        self.plan_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')

        self.get_logger().info('Planner Visualizer started')
        self.get_logger().info('Waiting for map and planner...')

    def map_callback(self, msg):
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.get_logger().info(f'Map received: {w}x{h} @ {msg.info.resolution}m/cell')

    def costmap_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        self.costmap_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.get_logger().info(f'Costmap received: {w}x{h}')

    def path_callback(self, msg):
        self.path_poses = []
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            qz = ps.pose.orientation.z
            qw = ps.pose.orientation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            self.path_poses.append((x, y, yaw))
        self.get_logger().info(f'Path received: {len(self.path_poses)} waypoints')

    def send_goal(self, sx, sy, syaw, gx, gy, gyaw):
        """Send a planning request."""
        self.start = (sx, sy, syaw)
        self.goal = (gx, gy, gyaw)
        self.path_poses = None

        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Planner action server not available!')
            return False

        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True

        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = sx
        goal_msg.start.pose.position.y = sy
        goal_msg.start.pose.orientation.z = math.sin(syaw / 2.0)
        goal_msg.start.pose.orientation.w = math.cos(syaw / 2.0)

        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = gx
        goal_msg.goal.pose.position.y = gy
        goal_msg.goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal_msg.goal.pose.orientation.w = math.cos(gyaw / 2.0)

        self.get_logger().info(
            f'Planning: ({sx:.1f},{sy:.1f},{math.degrees(syaw):.0f}deg) -> '
            f'({gx:.1f},{gy:.1f},{math.degrees(gyaw):.0f}deg)')

        future = self.plan_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        if result_future.result() is None:
            self.get_logger().error('Planning timed out!')
            return False

        result = result_future.result().result
        self.path_poses = []
        for ps in result.path.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            qz = ps.pose.orientation.z
            qw = ps.pose.orientation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            self.path_poses.append((x, y, yaw))

        self.get_logger().info(f'Path: {len(self.path_poses)} waypoints')
        return True

    def render(self, output_path='/root/planner_output.png'):
        """Render the map, costmap, path, start/goal to a PNG file."""
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))

        if self.map_data is not None and self.map_info is not None:
            res = self.map_info.resolution
            ox = self.map_info.origin.position.x
            oy = self.map_info.origin.position.y
            h, w = self.map_data.shape

            # Create display image: white=free, black=occupied, gray=unknown
            display = np.ones((h, w, 3), dtype=np.float32)
            occupied = self.map_data > 50
            unknown = self.map_data < 0
            free = (self.map_data >= 0) & (self.map_data <= 50)

            display[occupied] = [0.15, 0.15, 0.15]  # dark gray/black
            display[unknown] = [0.7, 0.7, 0.7]       # gray
            display[free] = [1.0, 1.0, 1.0]           # white

            # Overlay costmap if available (inflation zones in light blue)
            if self.costmap_data is not None:
                ch, cw = self.costmap_data.shape
                if ch == h and cw == w:
                    inflation = (self.costmap_data > 0) & (self.costmap_data < 100) & free
                    cost_norm = self.costmap_data[inflation].astype(float) / 100.0
                    display[inflation, 0] = 1.0 - cost_norm * 0.6  # R
                    display[inflation, 1] = 1.0 - cost_norm * 0.3  # G
                    display[inflation, 2] = 1.0                     # B

            extent = [ox, ox + w * res, oy, oy + h * res]
            ax.imshow(display, origin='lower', extent=extent, aspect='equal')

            ax.set_xlabel('X (meters)', fontsize=12)
            ax.set_ylabel('Y (meters)', fontsize=12)
            ax.set_title('Hybrid A* Path Planner', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3, linestyle='--')
        else:
            ax.text(0.5, 0.5, 'No map received', transform=ax.transAxes,
                    ha='center', va='center', fontsize=16, color='red')

        # Draw path
        if self.path_poses and len(self.path_poses) > 1:
            px = [p[0] for p in self.path_poses]
            py = [p[1] for p in self.path_poses]

            # Detect forward/reverse segments
            for i in range(len(self.path_poses) - 1):
                x1, y1, yaw1 = self.path_poses[i]
                x2, y2, _ = self.path_poses[i + 1]
                dx, dy = x2 - x1, y2 - y1
                heading_x, heading_y = math.cos(yaw1), math.sin(yaw1)
                dot = dx * heading_x + dy * heading_y
                color = '#2196F3' if dot >= 0 else '#FF5722'  # blue=forward, orange=reverse
                ax.plot([x1, x2], [y1, y2], color=color, linewidth=2.5, alpha=0.9)

            # Direction arrows every N waypoints
            arrow_step = max(1, len(self.path_poses) // 20)
            for i in range(0, len(self.path_poses), arrow_step):
                x, y, yaw = self.path_poses[i]
                arrow_len = 0.15
                ax.annotate('', xy=(x + arrow_len * math.cos(yaw),
                                     y + arrow_len * math.sin(yaw)),
                            xytext=(x, y),
                            arrowprops=dict(arrowstyle='->', color='#1565C0',
                                          lw=1.5, mutation_scale=10))

        # Draw start
        if self.start:
            sx, sy, syaw = self.start
            ax.plot(sx, sy, 'o', color='#4CAF50', markersize=14, markeredgecolor='white',
                    markeredgewidth=2, zorder=10)
            arrow_len = 0.3
            ax.annotate('', xy=(sx + arrow_len * math.cos(syaw),
                                 sy + arrow_len * math.sin(syaw)),
                        xytext=(sx, sy),
                        arrowprops=dict(arrowstyle='->', color='#4CAF50', lw=3))
            ax.text(sx, sy + 0.3, 'START', fontsize=10, ha='center',
                    color='#4CAF50', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

        # Draw goal
        if self.goal:
            gx, gy, gyaw = self.goal
            ax.plot(gx, gy, '*', color='#F44336', markersize=18, markeredgecolor='white',
                    markeredgewidth=1.5, zorder=10)
            arrow_len = 0.3
            ax.annotate('', xy=(gx + arrow_len * math.cos(gyaw),
                                 gy + arrow_len * math.sin(gyaw)),
                        xytext=(gx, gy),
                        arrowprops=dict(arrowstyle='->', color='#F44336', lw=3))
            ax.text(gx, gy + 0.3, 'GOAL', fontsize=10, ha='center',
                    color='#F44336', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

        # Legend
        if self.path_poses:
            path_len = 0.0
            for i in range(1, len(self.path_poses)):
                dx = self.path_poses[i][0] - self.path_poses[i-1][0]
                dy = self.path_poses[i][1] - self.path_poses[i-1][1]
                path_len += math.sqrt(dx*dx + dy*dy)

            legend_elements = [
                mpatches.Patch(color='#2196F3', label=f'Forward path'),
                mpatches.Patch(color='#FF5722', label=f'Reverse path'),
                mpatches.Patch(color='#4CAF50', label='Start'),
                mpatches.Patch(color='#F44336', label='Goal'),
            ]
            ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
            ax.text(0.98, 0.02,
                    f'Path: {path_len:.2f}m | Waypoints: {len(self.path_poses)}',
                    transform=ax.transAxes, ha='right', va='bottom', fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Saved visualization to {output_path}')
        return output_path


def main():
    rclpy.init()
    node = PlannerVisualizer()

    # Spin briefly to receive map
    print('\nWaiting for map topic (3s)...')
    end_time = time.time() + 3.0
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.map_data is None:
        print('WARNING: No map received. Is map_server active?')

    # Parse command-line goals or use defaults
    scenarios = []
    if len(sys.argv) >= 7:
        # Custom: python3 visualize_planner.py sx sy syaw gx gy gyaw
        sx, sy, syaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
        gx, gy, gyaw = float(sys.argv[4]), float(sys.argv[5]), math.radians(float(sys.argv[6]))
        scenarios.append((sx, sy, syaw, gx, gy, gyaw, '/root/planner_output.png'))
    else:
        # Default test scenarios
        scenarios = [
            # (sx, sy, syaw, gx, gy, gyaw, output_file)
            (2.0, 2.0, 0.0,           7.0, 7.0, math.pi/2, '/root/plan_diagonal.png'),
            (2.0, 5.0, 0.0,           8.0, 5.0, 0.0,       '/root/plan_straight.png'),
            (8.0, 8.0, math.pi,       2.0, 2.0, -math.pi/2,'/root/plan_reverse.png'),
        ]

    for i, (sx, sy, syaw, gx, gy, gyaw, outfile) in enumerate(scenarios):
        print(f'\n--- Scenario {i+1}/{len(scenarios)} ---')
        success = node.send_goal(sx, sy, syaw, gx, gy, gyaw)
        if success:
            # Small spin to process any remaining callbacks
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.render(outfile)
            print(f'  -> Saved: {outfile}')
        else:
            print(f'  -> Planning failed!')

    # Also render just the map if no path
    if not scenarios:
        node.render('/root/planner_map.png')

    node.destroy_node()
    rclpy.shutdown()

    print('\n=== Done! ===')
    print('To view on your Mac, run:')
    for _, _, _, _, _, _, f in scenarios:
        mac_name = f.split('/')[-1]
        print(f'  docker cp bb70e7a4468f:{f} ~/Desktop/{mac_name} && open ~/Desktop/{mac_name}')


if __name__ == '__main__':
    main()
