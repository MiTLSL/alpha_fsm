from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .geometry import clamp


@dataclass(frozen=True)
class ChassisStatus:
    enabled: bool = False
    fault: bool = False
    heartbeat_ok: bool = False
    stale: bool = True
    message: str = ""


def map_nav2_status_to_error(status: int, goal_status, error_code) -> int:
    if int(status) == int(goal_status.STATUS_CANCELED):
        return int(error_code.E_NAV_GOAL_CANCELLED)
    if int(status) == int(goal_status.STATUS_ABORTED):
        return int(error_code.E_NAV_PATH_PLAN_FAIL)
    return int(error_code.E_NAV_UNKNOWN)


def parse_chassis_diagnostics(status_items: Iterable) -> ChassisStatus:
    items = list(status_items or [])
    if not items:
        return ChassisStatus(message="no chassis diagnostics")

    enabled = False
    fault = False
    heartbeat_ok = False
    stale = False
    messages = []
    saw_enabled = False
    saw_heartbeat = False

    for item in items:
        level = int(getattr(item, "level", 3))
        name = str(getattr(item, "name", "")).lower()
        message = str(getattr(item, "message", ""))
        if message:
            messages.append(message)
        values = {str(kv.key).lower(): str(kv.value).lower() for kv in getattr(item, "values", [])}

        if level >= 2 or values.get("fault") in _TRUE_VALUES or values.get("fault_a") in _TRUE_VALUES or values.get("fault_b") in _TRUE_VALUES:
            fault = True
        if level >= 3:
            stale = True

        if "enabled" in values:
            enabled = values["enabled"] in _TRUE_VALUES
            saw_enabled = True
        elif "enable" in values:
            enabled = values["enable"] in _TRUE_VALUES
            saw_enabled = True

        if "heartbeat_ok" in values:
            heartbeat_ok = values["heartbeat_ok"] in _TRUE_VALUES
            saw_heartbeat = True
        elif "heartbeat" in values:
            heartbeat_ok = values["heartbeat"] in _TRUE_VALUES or values["heartbeat"] not in _FALSE_VALUES
            saw_heartbeat = True

    if not saw_enabled:
        enabled = not fault
    if not saw_heartbeat:
        heartbeat_ok = not stale
    return ChassisStatus(
        enabled=bool(enabled),
        fault=bool(fault),
        heartbeat_ok=bool(heartbeat_ok),
        stale=bool(stale),
        message="; ".join(messages),
    )


def alignment_velocity(
    dist_error: float,
    yaw_error: float,
    *,
    linear_gain: float,
    angular_gain: float,
    max_linear_x: float,
    max_angular_z: float,
    min_linear_x: float = 0.0,
    min_angular_z: float = 0.0,
    dist_deadband: float = 0.0,
    yaw_deadband: float = 0.0,
) -> tuple[float, float]:
    linear = 0.0 if abs(dist_error) <= max(float(dist_deadband), 0.0) else float(linear_gain) * float(dist_error)
    angular = 0.0 if abs(yaw_error) <= max(float(yaw_deadband), 0.0) else float(angular_gain) * float(yaw_error)
    linear = clamp_with_min_abs(linear, float(min_linear_x), float(max_linear_x))
    angular = clamp_with_min_abs(angular, float(min_angular_z), float(max_angular_z))
    return linear, angular


def clamp_with_min_abs(value: float, min_abs: float, max_abs: float) -> float:
    max_abs = abs(float(max_abs))
    min_abs = min(abs(float(min_abs)), max_abs)
    value = clamp(float(value), -max_abs, max_abs)
    if abs(value) < 1e-9:
        return 0.0
    if abs(value) < min_abs:
        return math.copysign(min_abs, value)
    return value


def box_face_goal_pose(center, normal, offset_m: float, frame_id: str = "map"):
    from geometry_msgs.msg import PoseStamped

    nx, ny, _ = _normalized_xy((normal.x, normal.y, normal.z))
    goal = PoseStamped()
    goal.header.frame_id = str(frame_id)
    goal.pose.position.x = float(center.x) + nx * float(offset_m)
    goal.pose.position.y = float(center.y) + ny * float(offset_m)
    goal.pose.position.z = float(center.z)
    yaw = math.atan2(-ny, -nx)
    goal.pose.orientation.z = math.sin(yaw * 0.5)
    goal.pose.orientation.w = math.cos(yaw * 0.5)
    return goal


def _normalized_xy(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    norm = math.hypot(x, y)
    if norm < 1e-9:
        return 1.0, 0.0, 0.0
    return x / norm, y / norm, z


_TRUE_VALUES = {"1", "true", "yes", "on", "ok", "enabled"}
_FALSE_VALUES = {"0", "false", "no", "off", "lost", "stale", "disabled"}
