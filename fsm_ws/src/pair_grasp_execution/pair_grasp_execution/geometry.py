from __future__ import annotations

import copy
import math


def make_pose_stamped(frame_id: str = "base_link"):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


def normalized_pose(pose_stamped, default_frame: str):
    pose = copy.deepcopy(pose_stamped)
    if not str(pose.header.frame_id):
        pose.header.frame_id = str(default_frame)
    q = pose.pose.orientation
    length = math.sqrt(float(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))
    if length <= 1e-9:
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
    else:
        q.x = float(q.x / length)
        q.y = float(q.y / length)
        q.z = float(q.z / length)
        q.w = float(q.w / length)
    return pose


def offset_pose_along_local_x(pose_stamped, offset_x: float, default_frame: str):
    pose = normalized_pose(pose_stamped, default_frame)
    dx, dy, dz = rotate_vector_by_quaternion(
        (float(offset_x), 0.0, 0.0),
        (
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        ),
    )
    pose.pose.position.x += dx
    pose.pose.position.y += dy
    pose.pose.position.z += dz
    return pose


def rotate_vector_by_quaternion(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


def bounded_scaling(value: float) -> float:
    return max(0.01, min(float(value), 1.0))
