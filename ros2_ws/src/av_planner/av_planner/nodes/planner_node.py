"""
Hybrid A* Planner — ROS 2 Lifecycle Node

Thin wrapper around the pure-Python Hybrid A* core.
Subscribes to /map, provides /plan_path action server, publishes path + stats.
"""

import sys
import os
import threading
import numpy as np

import rclpy
from rclpy.lifecycle import Node as LifecycleNode, TransitionCallbackReturn, State
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from av_planner_interfaces.msg import PlanningStats
from av_planner_interfaces.action import PlanPath

from av_planner.adapters.map_adapter import MapAdapter
from av_planner.adapters.converters import (
    pose_to_state, states_to_path, yaw_from_quaternion
)


def _setup_core_imports(project_root: str):
    """Add project root to sys.path so 'from src.*' imports resolve."""
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    from src.state import State as CoreState
    from src.hybrid_astar.motion_model import MotionModel, VehicleFootprint
    from src.hybrid_astar.heuristic import HeuristicCalculator
    from src.hybrid_astar.hybrid_astar import HybridAStar

    return CoreState, MotionModel, VehicleFootprint, HeuristicCalculator, HybridAStar


class PlannerNode(LifecycleNode):
    """Lifecycle node wrapping Hybrid A* planner."""

    def __init__(self):
        super().__init__('planner_node')

        # Declare all parameters
        self._declare_params()

        # Core planner components (initialized on map receipt)
        self._planner = None
        self._map_handler = None
        self._core_classes = None
        self._plan_lock = threading.Lock()

    def _declare_params(self):
        """Declare all ROS parameters with defaults."""
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('project_root', '')

        # Vehicle
        self.declare_parameter('vehicle.wheel_base', 2.5)
        self.declare_parameter('vehicle.length', 4.5)
        self.declare_parameter('vehicle.width', 2.0)
        self.declare_parameter('vehicle.rear_axle_to_back', 1.0)

        # Motion
        self.declare_parameter('motion.max_steering_angle', 40.0)
        self.declare_parameter('motion.step_size', 2.0)
        self.declare_parameter('motion.num_steering_angles', 5)
        self.declare_parameter('motion.allow_reverse', True)

        # Search
        self.declare_parameter('search.xy_resolution', 1.0)
        self.declare_parameter('search.theta_resolution', 5.0)
        self.declare_parameter('search.shot_distance', 30.0)
        self.declare_parameter('search.max_iterations', 200000)

        # Cost
        self.declare_parameter('cost.steering_penalty', 1.5)
        self.declare_parameter('cost.reversing_penalty', 2.0)
        self.declare_parameter('cost.steering_change_penalty', 1.5)
        self.declare_parameter('cost.direction_switch_penalty', 10.0)

        # Heuristic
        self.declare_parameter('heuristic.type', 'max')
        self.declare_parameter('heuristic.turning_radius', 2.98)
        self.declare_parameter('heuristic.cost_alpha', 1.0)

        # Map adapter
        self.declare_parameter('map.obstacle_threshold', 50)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Create subscriptions, publishers, and action server."""
        self.get_logger().info('Configuring...')

        # Import core planner classes
        project_root = self.get_parameter('project_root').value
        if not project_root:
            self.get_logger().error('Parameter "project_root" is required')
            return TransitionCallbackReturn.FAILURE
        try:
            self._core_classes = _setup_core_imports(project_root)
        except ImportError as e:
            self.get_logger().error(f'Failed to import core planner: {e}')
            return TransitionCallbackReturn.FAILURE

        # QoS for map: reliable + transient_local (get last published map)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Separate callback groups so map updates don't block planning
        self._map_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos,
            callback_group=self._map_cb_group
        )

        # Publishers
        self._path_pub = self.create_publisher(Path, '/planned_path', 1)
        self._stats_pub = self.create_publisher(PlanningStats, '/planning_stats', 1)

        # Action server
        self._action_server = ActionServer(
            self, PlanPath, '/plan_path', self._execute_plan,
            callback_group=self._action_cb_group
        )

        self.get_logger().info('Configured. Waiting for map on /map topic...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activated. Ready for planning requests.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivated.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        self._planner = None
        self._map_handler = None
        self.destroy_subscription(self._map_sub)
        self.destroy_publisher(self._path_pub)
        self.destroy_publisher(self._stats_pub)
        self._action_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    # --- Callbacks ---

    def _on_map(self, msg: OccupancyGrid):
        """Rebuild planner when a new map is received."""
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'res={msg.info.resolution:.3f} m/cell'
        )

        with self._plan_lock:
            self._build_planner(msg)

    def _build_planner(self, msg: OccupancyGrid):
        """Construct the full planner pipeline from OccupancyGrid."""
        CoreState, MotionModel, VehicleFootprint, HeuristicCalculator, HybridAStar = \
            self._core_classes

        # Create map adapter
        origin = msg.info.origin.position
        self._map_handler = MapAdapter(
            data=np.array(msg.data, dtype=np.int8),
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
            origin_x=origin.x,
            origin_y=origin.y,
            obstacle_threshold=self.get_parameter('map.obstacle_threshold').value
        )

        # Build motion model
        motion_model = MotionModel(
            wheel_base=self.get_parameter('vehicle.wheel_base').value,
            max_steering_angle=self.get_parameter('motion.max_steering_angle').value,
            step_size=self.get_parameter('motion.step_size').value,
            num_steering_angles=self.get_parameter('motion.num_steering_angles').value,
            allow_reverse=self.get_parameter('motion.allow_reverse').value
        )

        # Build vehicle footprint
        footprint = VehicleFootprint(
            length=self.get_parameter('vehicle.length').value,
            width=self.get_parameter('vehicle.width').value,
            rear_axle_to_back=self.get_parameter('vehicle.rear_axle_to_back').value
        )

        # Build heuristic
        heuristic = HeuristicCalculator(
            map_handler=self._map_handler,
            heuristic_type=self.get_parameter('heuristic.type').value,
            turning_radius=self.get_parameter('heuristic.turning_radius').value,
            cost_alpha=self.get_parameter('heuristic.cost_alpha').value
        )

        # Build planner
        self._planner = HybridAStar(
            map_handler=self._map_handler,
            motion_model=motion_model,
            vehicle_footprint=footprint,
            heuristic_calculator=heuristic,
            xy_resolution=self.get_parameter('search.xy_resolution').value,
            theta_resolution=self.get_parameter('search.theta_resolution').value,
            steering_penalty=self.get_parameter('cost.steering_penalty').value,
            reversing_penalty=self.get_parameter('cost.reversing_penalty').value,
            steering_change_penalty=self.get_parameter('cost.steering_change_penalty').value,
            direction_switch_penalty=self.get_parameter('cost.direction_switch_penalty').value,
            shot_distance=self.get_parameter('search.shot_distance').value,
            max_iterations=self.get_parameter('search.max_iterations').value
        )

        self.get_logger().info(
            f'Planner ready: map {self._map_handler.width:.1f}x'
            f'{self._map_handler.height:.1f}m'
        )

    def _execute_plan(self, goal_handle):
        """Action execute callback: plan path from start to goal."""
        CoreState = self._core_classes[0]
        frame_id = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()

        # Check planner is ready
        if self._planner is None:
            self.get_logger().warn('No map received yet — cannot plan')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'No map received'
            return result

        # Convert goal poses to States
        start_state = pose_to_state(goal_handle.request.start, CoreState)
        goal_state = pose_to_state(goal_handle.request.goal, CoreState)

        self.get_logger().info(
            f'Planning: ({start_state.x:.1f}, {start_state.y:.1f}, '
            f'{np.degrees(start_state.theta):.0f}°) → '
            f'({goal_state.x:.1f}, {goal_state.y:.1f}, '
            f'{np.degrees(goal_state.theta):.0f}°)'
        )

        # Publish feedback
        feedback = PlanPath.Feedback()
        feedback.status = 'planning'
        goal_handle.publish_feedback(feedback)

        # Run planner (blocking)
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

        # Convert path to ROS message
        result.path = states_to_path(path, frame_id, stamp)
        result.success = True
        result.message = (
            f'Path found: {info["path_length"]:.1f}m, '
            f'{info["nodes_expanded"]} nodes, '
            f'{info["search_time"]:.3f}s'
        )

        # Build stats
        stats = PlanningStats()
        stats.header.frame_id = frame_id
        stats.header.stamp = stamp
        stats.search_time = info.get('search_time', 0.0)
        stats.path_length = info.get('path_length', 0.0)
        stats.nodes_expanded = info.get('nodes_expanded', 0)
        stats.analytical_expansion = info.get('analytical', False)
        result.stats = stats

        # Publish path and stats
        self._path_pub.publish(result.path)
        self._stats_pub.publish(stats)

        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    # Use MultiThreadedExecutor so map callback and action can run concurrently
    from rclpy.executors import MultiThreadedExecutor
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
