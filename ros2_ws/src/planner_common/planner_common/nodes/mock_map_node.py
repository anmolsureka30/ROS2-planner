"""
Mock Map Publisher — publishes OccupancyGrid from a PNG file.

For standalone testing without a perception stack.
Publishes on /map with TRANSIENT_LOCAL QoS so late-joining nodes get the map.
"""

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class MockMapNode(Node):
    """Publishes a static OccupancyGrid loaded from a PNG image."""

    def __init__(self):
        super().__init__('mock_map_node')

        self.declare_parameter('map_image_path', '')
        self.declare_parameter('resolution', 0.5)
        self.declare_parameter('obstacle_threshold', 128)
        self.declare_parameter('publish_rate', 0.1)  # Hz (heartbeat)

        map_path = self.get_parameter('map_image_path').value
        if not map_path:
            self.get_logger().error('Parameter "map_image_path" is required')
            return

        resolution = self.get_parameter('resolution').value
        threshold = self.get_parameter('obstacle_threshold').value

        # Load PNG and convert to OccupancyGrid
        self._grid_msg = self._load_map(map_path, resolution, threshold)
        if self._grid_msg is None:
            return

        # Publisher with transient_local durability (late joiners get last msg)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Publish once immediately, then heartbeat
        self._publish_map()
        rate = self.get_parameter('publish_rate').value
        if rate > 0:
            self._timer = self.create_timer(1.0 / rate, self._publish_map)

        self.get_logger().info(
            f'Publishing map from {map_path} '
            f'({self._grid_msg.info.width}x{self._grid_msg.info.height}, '
            f'res={resolution}m)'
        )

    def _load_map(self, path: str, resolution: float,
                  threshold: int) -> OccupancyGrid:
        """Load PNG image and convert to OccupancyGrid message."""
        try:
            img = Image.open(path).convert('L')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return None

        img_array = np.array(img)
        height, width = img_array.shape

        # Convert to OccupancyGrid format:
        # PNG: 0=black=obstacle, 255=white=free
        # OccupancyGrid: 0=free, 100=occupied
        grid = np.zeros(height * width, dtype=np.int8)

        # Flip vertically (PNG has top-left origin, OccupancyGrid has bottom-left)
        img_flipped = np.flipud(img_array)
        flat = img_flipped.flatten()

        # Pixels below threshold → occupied (100), otherwise → free (0)
        grid[flat < threshold] = 100
        grid[flat >= threshold] = 0

        # Build message
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info = MapMetaData()
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin = Pose()
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.tolist()
        return msg

    def _publish_map(self):
        """Publish the stored OccupancyGrid."""
        self._grid_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
