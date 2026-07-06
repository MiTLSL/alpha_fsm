from __future__ import annotations

import re
from dataclasses import dataclass

from .geometry import normalized_pose


@dataclass(frozen=True)
class BoxObstacle:
    object_id: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


_SLOT_RE = re.compile(r"wall_(?P<wall>\d+)_row_(?P<row>\d+)_col_(?P<col>\d+)")


def slot_indices(slot_id: str) -> tuple[int, int] | None:
    match = _SLOT_RE.fullmatch(str(slot_id))
    if match is None:
        return None
    return int(match.group("row")), int(match.group("col"))


def make_container_obstacles(config_get, frame: str) -> list[BoxObstacle]:
    if not bool(config_get("business.pair_grasp_execution.collision_scene.enable_container_obstacle", False)):
        return []
    center_x = float(config_get("business.pair_grasp_execution.collision_scene.container.center_x", 0.0))
    center_y = float(config_get("business.pair_grasp_execution.collision_scene.container.center_y", 0.0))
    width = float(config_get("business.pair_grasp_execution.collision_scene.container.width", 2.2))
    height = float(config_get("business.pair_grasp_execution.collision_scene.container.height", 2.4))
    length = float(config_get("business.pair_grasp_execution.collision_scene.container.length", 8.0))
    thickness = float(config_get("business.pair_grasp_execution.collision_scene.container.wall_thickness", 0.03))
    floor_z = float(config_get("business.pair_grasp_execution.collision_scene.container.floor_z", 0.0))
    del frame

    half_width = width * 0.5
    half_thickness = thickness * 0.5
    z_center = floor_z + height * 0.5
    return [
        BoxObstacle(
            "container_left_wall",
            (center_x, center_y + half_width + half_thickness, z_center),
            (length, thickness, height),
        ),
        BoxObstacle(
            "container_right_wall",
            (center_x, center_y - half_width - half_thickness, z_center),
            (length, thickness, height),
        ),
        BoxObstacle(
            "container_ceiling",
            (center_x, center_y, floor_z + height + half_thickness),
            (length, width + 2.0 * thickness, thickness),
        ),
    ]


def make_box_wall_opening_obstacles(pair, config_get, planning_frame: str) -> list[BoxObstacle]:
    if not bool(config_get("business.pair_grasp_execution.collision_scene.enable_static_box_wall_obstacles", False)):
        return []

    active_boxes = _active_pair_boxes(pair, planning_frame)
    if not active_boxes:
        return []

    inset = max(
        float(config_get("business.pair_grasp_execution.collision_scene.static_box_obstacle_inset", 0.002)),
        0.0,
    )
    margin = max(
        float(config_get("business.pair_grasp_execution.collision_scene.static_box_wall_margin", 0.0)),
        0.0,
    )
    floor_z = float(config_get("business.pair_grasp_execution.collision_scene.container.floor_z", 0.0))
    container_center_y = float(config_get("business.pair_grasp_execution.collision_scene.container.center_y", 0.0))
    container_width = float(config_get("business.pair_grasp_execution.collision_scene.container.width", 2.2))
    inner_y_min = container_center_y - container_width * 0.5
    inner_y_max = container_center_y + container_width * 0.5

    x_min = min(box["x_min"] for box in active_boxes) - margin
    x_max = max(box["x_max"] for box in active_boxes) + margin
    z_min = min(box["z_min"] for box in active_boxes) - margin
    z_max = max(box["z_max"] for box in active_boxes) + margin
    y_min = min(box["y_min"] for box in active_boxes) - margin
    y_max = max(box["y_max"] for box in active_boxes) + margin

    prefix = f"{str(pair.pair_id) or 'pair'}_static_wall"
    obstacles: list[BoxObstacle] = []
    _add_piece(obstacles, f"{prefix}_positive_y", x_min, x_max, y_max + inset, inner_y_max, z_min, z_max)
    _add_piece(obstacles, f"{prefix}_negative_y", x_min, x_max, inner_y_min, y_min - inset, z_min, z_max)

    if len(active_boxes) >= 2:
        sorted_boxes = sorted(active_boxes, key=lambda box: box["center_y"])
        between_y_min = sorted_boxes[0]["y_max"] + inset
        between_y_max = sorted_boxes[-1]["y_min"] - inset
        _add_piece(obstacles, f"{prefix}_between", x_min, x_max, between_y_min, between_y_max, z_min, z_max)

    _add_piece(obstacles, f"{prefix}_below", x_min, x_max, inner_y_min, inner_y_max, floor_z, z_min - inset)
    return obstacles


def selected_box_object_ids(pair) -> list[str]:
    ids = []
    if pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_LEFT_ONLY) and str(pair.left_slot_id):
        ids.append(_slot_object_id(pair.left_slot_id))
    if pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_RIGHT_ONLY) and str(pair.right_slot_id):
        ids.append(_slot_object_id(pair.right_slot_id))
    return ids


def pair_static_object_ids(pair) -> list[str]:
    pair_id = str(getattr(pair, "pair_id", "") or "pair")
    prefix = f"{pair_id}_static_wall"
    return [
        "container_left_wall",
        "container_right_wall",
        "container_ceiling",
        f"{prefix}_positive_y",
        f"{prefix}_negative_y",
        f"{prefix}_between",
        f"{prefix}_below",
        *selected_box_object_ids(pair),
    ]


def _active_pair_boxes(pair, planning_frame: str) -> list[dict]:
    result = []
    if pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_LEFT_ONLY) and str(pair.left_slot_id):
        result.append(_box_from_pose(pair.left_slot_id, pair.left_box_pose_robot, pair.left_box_size, planning_frame))
    if pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_RIGHT_ONLY) and str(pair.right_slot_id):
        result.append(_box_from_pose(pair.right_slot_id, pair.right_box_pose_robot, pair.right_box_size, planning_frame))
    return [box for box in result if box is not None]


def _box_from_pose(slot_id: str, pose_stamped, size, planning_frame: str):
    pose = normalized_pose(pose_stamped, planning_frame)
    length = _dimension_or_default(size, "x", 0.4)
    width = _dimension_or_default(size, "y", 0.4)
    height = _dimension_or_default(size, "z", 0.4)
    center_x = float(pose.pose.position.x)
    center_y = float(pose.pose.position.y)
    center_z = float(pose.pose.position.z)
    return {
        "object_id": _slot_object_id(slot_id),
        "center_x": center_x,
        "center_y": center_y,
        "center_z": center_z,
        "x_min": center_x - length * 0.5,
        "x_max": center_x + length * 0.5,
        "y_min": center_y - width * 0.5,
        "y_max": center_y + width * 0.5,
        "z_min": center_z - height * 0.5,
        "z_max": center_z + height * 0.5,
    }


def _slot_object_id(slot_id: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_]+", "_", str(slot_id))
    return f"box_{safe}"


def _dimension_or_default(size, attr: str, default: float) -> float:
    value = float(getattr(size, attr, 0.0))
    return value if value > 1e-6 else float(default)


def _add_piece(
    obstacles: list[BoxObstacle],
    object_id: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> None:
    if x_max < x_min:
        x_min, x_max = x_max, x_min
    if y_max < y_min:
        y_min, y_max = y_max, y_min
    if z_max < z_min:
        z_min, z_max = z_max, z_min
    if (x_max - x_min) < 1e-4 or (y_max - y_min) < 1e-4 or (z_max - z_min) < 1e-4:
        return
    obstacles.append(
        BoxObstacle(
            object_id,
            (0.5 * (x_min + x_max), 0.5 * (y_min + y_max), 0.5 * (z_min + z_max)),
            (x_max - x_min, y_max - y_min, z_max - z_min),
        )
    )
