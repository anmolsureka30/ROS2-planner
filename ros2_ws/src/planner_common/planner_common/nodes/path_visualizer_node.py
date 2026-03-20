"""
Path Visualizer Node — displays map + planned path using matplotlib.

Subscribes to:
  /map            (nav_msgs/OccupancyGrid)  — obstacle map
  /planned_path   (nav_msgs/Path)           — planner output

Updates the plot each time a new path is published.
Run in a separate terminal to monitor planning results visually.
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Falls back gracefully on headless systems
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Path


class PathVisualizerNode(Node):
    """Subscribes to /map and /planned_path, plots them with matplotlib."""

    def __init__(self):
        super().__init__('path_visualizer_node')

        self._map_data = None
        self._map_info = None
        self._path_count = 0

        # QoS: transient_local for map (get last published), volatile for path
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(Path, '/planned_path', self._on_path, path_qos)

        # Setup matplotlib figure
        self._fig, self._ax = plt.subplots(1, 1, figsize=(10, 10))
        self._fig.canvas.manager.set_window_title('Path Planner Visualizer')
        plt.ion()  # Interactive mode
        self._ax.set_xlabel('X (m)')
        self._ax.set_ylabel('Y (m)')
        self._ax.set_title('Waiting for /map ...')
        self._fig.tight_layout()
        plt.show(block=False)
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

        # Timer to keep matplotlib responsive
        self.create_timer(0.1, self._tick)

        self.get_logger().info('Visualizer ready. Waiting for /map and /planned_path ...')

    def _on_map(self, msg: OccupancyGrid):
        """Store map data for plotting."""
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        # Convert: -1/100 → obstacle (1), 0 → free (0)
        self._map_data = np.where((grid >= 50) | (grid < 0), 1, 0).astype(np.uint8)
        self._map_info = msg.info
        self._draw_map()
        self.get_logger().info(f'Map received: {w}x{h}')

    def _on_path(self, msg: Path):
        """Plot the new path on the map."""
        if self._map_data is None:
            self.get_logger().warn('Path received but no map yet')
            return

        if len(msg.poses) == 0:
            self.get_logger().warn('Empty path received')
            return

        self._path_count += 1
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]

        # Redraw map (clears old path)
        self._draw_map()

        # Plot path
        self._ax.plot(xs, ys, 'r-', linewidth=2.5, label='Planned Path', zorder=3)

        # Start marker (green circle with heading arrow)
        sx, sy = xs[0], ys[0]
        q = msg.poses[0].pose.orientation
        s_yaw = 2.0 * np.arctan2(q.z, q.w)
        self._ax.plot(sx, sy, 'o', color='limegreen', markersize=14,
                      markeredgecolor='black', markeredgewidth=1.5, zorder=5)
        self._ax.annotate('', xy=(sx + 3*np.cos(s_yaw), sy + 3*np.sin(s_yaw)),
                          xytext=(sx, sy),
                          arrowprops=dict(arrowstyle='->', color='limegreen',
                                          lw=2.5), zorder=5)

        # Goal marker (red triangle with heading arrow)
        gx, gy = xs[-1], ys[-1]
        q = msg.poses[-1].pose.orientation
        g_yaw = 2.0 * np.arctan2(q.z, q.w)
        self._ax.plot(gx, gy, '^', color='red', markersize=14,
                      markeredgecolor='black', markeredgewidth=1.5, zorder=5)
        self._ax.annotate('', xy=(gx + 3*np.cos(g_yaw), gy + 3*np.sin(g_yaw)),
                          xytext=(gx, gy),
                          arrowprops=dict(arrowstyle='->', color='red',
                                          lw=2.5), zorder=5)

        # Path length
        length = sum(np.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i])
                      for i in range(len(xs)-1))

        self._ax.set_title(
            f'Path #{self._path_count}  |  '
            f'{len(msg.poses)} waypoints  |  '
            f'{length:.1f}m',
            fontsize=13, fontweight='bold'
        )
        self._ax.legend(loc='lower right', fontsize=10)

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

        self.get_logger().info(
            f'Path #{self._path_count}: {len(msg.poses)} poses, {length:.1f}m')

    def _draw_map(self):
        """Draw the occupancy grid on the axes."""
        self._ax.clear()
        info = self._map_info
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width * info.resolution
        h = info.height * info.resolution

        self._ax.imshow(
            self._map_data, cmap='gray_r', origin='lower',
            extent=[ox, ox + w, oy, oy + h],
            alpha=0.85, zorder=1
        )
        self._ax.set_xlabel('X (m)', fontsize=11)
        self._ax.set_ylabel('Y (m)', fontsize=11)
        self._ax.set_xlim(ox, ox + w)
        self._ax.set_ylim(oy, oy + h)
        self._ax.set_aspect('equal')
        self._ax.grid(True, alpha=0.15)

    def _tick(self):
        """Keep matplotlib event loop alive."""
        try:
            self._fig.canvas.flush_events()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
