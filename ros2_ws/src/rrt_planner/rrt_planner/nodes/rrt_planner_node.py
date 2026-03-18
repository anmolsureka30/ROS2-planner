"""
RRT* Family Planner — ROS 2 Lifecycle Node

Thin middleware wrapper around the core RRT*, Informed RRT*, and BI-RRT*
algorithms. The core algorithm code lives in src/rrt_star/ with zero ROS
dependencies. This node:
  1. Subscribes to /map (OccupancyGrid)
  2. Converts it to a MapAdapter (duck-typed MapHandler)
  3. Instantiates the selected RRT* planner
  4. Serves /plan_path action requests
  5. Publishes /planned_path and /planning_stats

Usage:
  ros2 launch rrt_planner rrt_planner.launch.xml
  ros2 lifecycle set /rrt_planner_node configure
  ros2 lifecycle set /rrt_planner_node activate
  ros2 action send_goal /plan_path av_planner_interfaces/action/PlanPath ...
"""

import sys
import threading
import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer

from av_planner_interfaces.action import PlanPath
from av_planner_interfaces.msg import PlanningStats


def _setup_core_imports(project_root: str):
    """Dynamically import core algorithm classes from the project root.

    Adds the project root to sys.path so that `from src.rrt_star import ...`
    works without modifying PYTHONPATH globally.

    Returns:
        Tuple of (CoreState, RRTStarPlanner, InformedRRTStarPlanner,
                  BIRRTStarPlanner, MapAdapter, pose_to_state,
                  states_to_path)
    """
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    from src.state import State as CoreState
    from src.rrt_star.rrt_star import RRTStarPlanner
    from src.rrt_star.informed_rrt_star import InformedRRTStarPlanner
    from src.rrt_star.bi_rrt_star import BIRRTStarPlanner

    # Shared adapters from planner_common
    from planner_common.adapters.map_adapter import MapAdapter
    from planner_common.adapters.converters import pose_to_state, states_to_path

    return (CoreState, RRTStarPlanner, InformedRRTStarPlanner,
            BIRRTStarPlanner, MapAdapter, pose_to_state, states_to_path)


PLANNER_MAP = {
    'rrt_star': 1,           # index into _core_classes
    'informed_rrt_star': 2,
    'bi_rrt_star': 3,
}


class RRTPlannerNode(LifecycleNode):
    """ROS 2 lifecycle node wrapping RRT* family planners."""

    def __init__(self):
        super().__init__('rrt_planner_node')
        self._declare_params()

        self._planner = None
        self._map_adapter = None
        self._core_classes = None
        self._plan_lock = threading.Lock()

    def _declare_params(self):
        """Declare all ROS parameters with defaults."""
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('project_root', '')
        self.declare_parameter('algorithm', 'rrt_star')

        # RRT* shared parameters
        self.declare_parameter('max_iterations', 10000)
        self.declare_parameter('step_size', 2.0)
        self.declare_parameter('goal_radius', 5.0)
        self.declare_parameter('goal_bias', 0.10)
        self.declare_parameter('gamma', 20.0)
        self.declare_parameter('dimension', 2)

        # BI-RRT* specific
        self.declare_parameter('max_bidir_iterations', 5000)
        self.declare_parameter('max_opt_iterations', 5000)
        self.declare_parameter('safety_margin', 0.0)

        # Map
        self.declare_parameter('obstacle_threshold', 50)

    # ── Lifecycle callbacks ──

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')

        project_root = self.get_parameter('project_root').value
        if not project_root:
            self.get_logger().error('project_root parameter not set')
            return TransitionCallbackReturn.FAILURE

        try:
            self._core_classes = _setup_core_imports(project_root)
        except Exception as e:
            self.get_logger().error(f'Failed to import core classes: {e}')
            return TransitionCallbackReturn.FAILURE

        # Callback groups: map updates and planning on separate threads
        self._map_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = MutuallyExclusiveCallbackGroup()

        # Map subscriber — TRANSIENT_LOCAL so late joiners get the last map
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos,
            callback_group=self._map_cb_group)

        # Publishers
        self._path_pub = self.create_publisher(Path, '/planned_path', 1)
        self._stats_pub = self.create_publisher(PlanningStats, '/planning_stats', 1)

        # Action server
        self._action_server = ActionServer(
            self, PlanPath, '/plan_path',
            execute_callback=self._execute_plan,
            callback_group=self._action_cb_group)

        self.get_logger().info('Configured. Waiting for map on /map...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Activated. Ready for planning requests.')
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivated.')
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        self.destroy_subscription(self._map_sub)
        self.destroy_publisher(self._path_pub)
        self.destroy_publisher(self._stats_pub)
        self._action_server.destroy()
        self._planner = None
        self._map_adapter = None
        return TransitionCallbackReturn.SUCCESS

    # ── Map callback ──

    def _on_map(self, msg: OccupancyGrid):
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution:.2f} m/px')

        with self._plan_lock:
            self._build_planner(msg)

    def _build_planner(self, msg: OccupancyGrid):
        """Build MapAdapter and instantiate the selected RRT* planner."""
        (CoreState, RRTStarPlanner, InformedRRTStarPlanner,
         BIRRTStarPlanner, MapAdapter, _, _) = self._core_classes

        # Build MapAdapter from OccupancyGrid
        self._map_adapter = MapAdapter(
            data=np.array(msg.data, dtype=np.int8),
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            obstacle_threshold=self.get_parameter('obstacle_threshold').value)

        # Build config dict matching what the planners expect
        alg_name = self.get_parameter('algorithm').value
        shared = {
            'max_iterations': self.get_parameter('max_iterations').value,
            'step_size': self.get_parameter('step_size').value,
            'goal_radius': self.get_parameter('goal_radius').value,
            'goal_bias': self.get_parameter('goal_bias').value,
            'gamma': self.get_parameter('gamma').value,
            'dimension': self.get_parameter('dimension').value,
        }

        config = {
            'rrt_star': dict(shared),
            'informed_rrt_star': dict(shared),
            'bi_rrt_star': {
                **shared,
                'max_bidir_iterations': self.get_parameter('max_bidir_iterations').value,
                'max_opt_iterations': self.get_parameter('max_opt_iterations').value,
                'safety_margin': self.get_parameter('safety_margin').value,
            },
        }

        # Select and instantiate planner
        planners = {
            'rrt_star': RRTStarPlanner,
            'informed_rrt_star': InformedRRTStarPlanner,
            'bi_rrt_star': BIRRTStarPlanner,
        }

        cls = planners.get(alg_name)
        if cls is None:
            self.get_logger().error(f'Unknown algorithm: {alg_name}')
            return

        self._planner = cls(self._map_adapter, config)

        world_w = msg.info.width * msg.info.resolution
        world_h = msg.info.height * msg.info.resolution
        self.get_logger().info(
            f'Planner ready: {alg_name} on {world_w:.0f}x{world_h:.0f}m map')

    # ── Action callback ──

    def _execute_plan(self, goal_handle):
        """Handle a PlanPath action request."""
        (CoreState, _, _, _, _, pose_to_state, states_to_path) = self._core_classes

        frame_id = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()

        # Check planner ready
        if self._planner is None:
            self.get_logger().warn('No planner available (map not received)')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'No map received yet'
            return result

        # Convert poses to core State
        start_state = pose_to_state(goal_handle.request.start, CoreState)
        goal_state = pose_to_state(goal_handle.request.goal, CoreState)

        alg_name = self.get_parameter('algorithm').value
        self.get_logger().info(
            f'Planning ({alg_name}): '
            f'({start_state.x:.1f}, {start_state.y:.1f}, '
            f'{np.degrees(start_state.theta):.0f}°) → '
            f'({goal_state.x:.1f}, {goal_state.y:.1f}, '
            f'{np.degrees(goal_state.theta):.0f}°)')

        # Publish feedback
        feedback = PlanPath.Feedback()
        feedback.status = f'Planning with {alg_name}...'
        goal_handle.publish_feedback(feedback)

        # Run planner (thread-safe)
        with self._plan_lock:
            plan_result = self._planner.plan(start_state, goal_state)

        # Build result
        result = PlanPath.Result()

        if plan_result is None:
            self.get_logger().warn('Planning failed — no path found')
            goal_handle.abort()
            result.success = False
            result.message = 'No path found'
            return result

        path, info = plan_result

        # Convert to ROS messages
        result.path = states_to_path(path, frame_id, stamp)
        result.success = True
        result.message = (
            f'{alg_name}: {info["path_length"]:.2f}m in '
            f'{info["search_time"]:.3f}s, {info["tree_size"]} nodes')

        # Build stats
        stats = PlanningStats()
        stats.header.frame_id = frame_id
        stats.header.stamp = stamp
        stats.search_time = info['search_time']
        stats.path_length = info['path_length']
        stats.nodes_expanded = info.get('nodes_expanded', 0)
        stats.analytical_expansion = False  # not applicable for RRT*
        result.stats = stats

        # Publish
        self._path_pub.publish(result.path)
        self._stats_pub.publish(stats)

        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
