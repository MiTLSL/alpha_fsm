from __future__ import annotations

import math


def make_pose_stamped(frame_id: str = "map", x: float = 0.0, y: float = 0.0, z: float = 0.0):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    return pose


def duration_to_sec(duration) -> float:
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) / 1e9


def yaw_from_pose(pose_stamped) -> float:
    q = pose_stamped.pose.orientation
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))
