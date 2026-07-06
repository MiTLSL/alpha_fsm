from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from typing import Any


@dataclass(frozen=True)
class BoxTruth:
    wall_index: int
    row: int
    col: int
    box_id: int
    center: tuple[float, float, float]
    nearest_face_center: tuple[float, float, float]
    nearest_face_normal: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class SceneTruth:
    frame_id: str
    rows: int
    cols: int
    boxes: tuple[BoxTruth, ...]


def _get(config: dict[str, Any], key: str, default: Any) -> Any:
    return config.get(key, default)


def _parse_missing_slots(value: Any) -> set[tuple[int, int]]:
    slots: set[tuple[int, int]] = set()
    for item in value or []:
        if isinstance(item, str):
            parts = [part.strip() for part in item.split(",", maxsplit=1)]
            if len(parts) != 2:
                continue
            slots.add((int(parts[0]), int(parts[1])))
        elif isinstance(item, (list, tuple)) and len(item) == 2:
            slots.add((int(item[0]), int(item[1])))
    return slots


def build_scene(config: dict[str, Any]) -> SceneTruth:
    rows = int(_get(config, "sim.scene.rows", 5))
    cols = int(_get(config, "sim.scene.cols", 5))
    wall_index = int(_get(config, "sim.scene.active_wall_index", 0))
    frame_id = str(_get(config, "sim.scene.source_frame", _get(config, "interfaces.frames.body", "body")))

    length = float(_get(config, "sim.scene.box_size.length", _get(config, "business.box_size.length", 0.4)))
    width = float(_get(config, "sim.scene.box_size.width", _get(config, "business.box_size.width", 0.4)))
    height = float(_get(config, "sim.scene.box_size.height", _get(config, "business.box_size.height", 0.4)))

    origin_x = float(_get(config, "sim.scene.grid_origin_map.x", 0.60))
    origin_y = float(_get(config, "sim.scene.grid_origin_map.y", 0.80))
    origin_z = float(_get(config, "sim.scene.grid_origin_map.z", 1.80))
    yaw = float(_get(config, "sim.scene.grid_origin_map.yaw", 0.0))
    wall_spacing_y = float(_get(config, "sim.scene.wall_spacing_y", 2.50))
    origin_y += wall_index * wall_spacing_y

    normal = (cos(yaw), sin(yaw), 0.0)
    col_axis = (sin(yaw), -cos(yaw), 0.0)
    missing = _parse_missing_slots(_get(config, "sim.scene.missing_slots", []))

    boxes: list[BoxTruth] = []
    for row in range(rows):
        for col in range(cols):
            if (row, col) in missing:
                continue
            center = (
                origin_x + col_axis[0] * width * col,
                origin_y + col_axis[1] * width * col,
                origin_z - height * row,
            )
            face_center = (
                center[0] + normal[0] * length * 0.5,
                center[1] + normal[1] * length * 0.5,
                center[2] + normal[2] * length * 0.5,
            )
            boxes.append(
                BoxTruth(
                    wall_index=wall_index,
                    row=row,
                    col=col,
                    box_id=wall_index * rows * cols + row * cols + col,
                    center=center,
                    nearest_face_center=face_center,
                    nearest_face_normal=normal,
                    size=(length, width, height),
                )
            )

    return SceneTruth(frame_id=frame_id, rows=rows, cols=cols, boxes=tuple(boxes))
