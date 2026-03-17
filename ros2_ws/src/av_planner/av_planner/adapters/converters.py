"""
Type converters between ROS 2 messages and core planner types.
Zero algorithm logic — pure data transformation.
"""

import math
from typing import List

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (rotation about Z) from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create quaternion from yaw (rotation about Z only)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def pose_to_state(msg: PoseStamped, state_cls):
    """Convert PoseStamped to core State.

    Args:
        msg: ROS PoseStamped message.
        state_cls: The State class (passed to avoid importing core code here).

    Returns:
        A State instance with (x, y, theta).
    """
    x = msg.pose.position.x
    y = msg.pose.position.y
    theta = yaw_from_quaternion(msg.pose.orientation)
    return state_cls(x, y, theta)


def state_to_pose(state, frame_id: str, stamp: Time) -> PoseStamped:
    """Convert core State to PoseStamped."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(state.x)
    msg.pose.position.y = float(state.y)
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(float(state.theta))
    return msg


def states_to_path(states: List, frame_id: str, stamp: Time) -> Path:
    """Convert list of States to nav_msgs/Path."""
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = stamp
    for s in states:
        path_msg.poses.append(state_to_pose(s, frame_id, stamp))
    return path_msg
