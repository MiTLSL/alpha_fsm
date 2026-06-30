from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class BoxTruth:
    detection_id: str
    frame_id: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0
    confidence: float = 1.0
    visible: bool = True


@dataclass(frozen=True)
class ViewFilter:
    max_distance_m: float
    horizontal_fov_rad: float
    z_min: float
    z_max: float


def load_box_truths(
    *,
    scene_file: str,
    boxes_json: str,
    default_frame: str,
    default_size: tuple[float, float, float],
) -> tuple[BoxTruth, ...]:
    records: list[dict[str, Any]] = []
    if scene_file:
        path = Path(scene_file).expanduser()
        if path.exists():
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            records.extend(_extract_box_records(data))
    if boxes_json:
        data = json.loads(boxes_json)
        records.extend(_extract_box_records(data))
    if not records:
        records = _default_wall_records(default_frame, default_size)
    return tuple(_box_from_record(record, default_frame, default_size) for record in records)


def filter_visible_boxes(boxes: tuple[BoxTruth, ...], view: ViewFilter) -> tuple[BoxTruth, ...]:
    visible: list[BoxTruth] = []
    half_fov = max(float(view.horizontal_fov_rad), 0.0) * 0.5
    for box in boxes:
        if not box.visible:
            continue
        x, y, z = box.center
        if z < view.z_min or z > view.z_max:
            continue
        distance = math.hypot(x, y)
        if distance > view.max_distance_m:
            continue
        angle = abs(math.atan2(y, x))
        if half_fov > 0.0 and angle > half_fov:
            continue
        visible.append(box)
    return tuple(visible)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _extract_box_records(data: Any) -> list[dict[str, Any]]:
    if isinstance(data, dict):
        if isinstance(data.get("boxes"), list):
            return [item for item in data["boxes"] if isinstance(item, dict)]
        if isinstance(data.get("isaac"), dict):
            return _extract_box_records(data["isaac"])
        if isinstance(data.get("scene"), dict):
            return _extract_box_records(data["scene"])
        return [data]
    if isinstance(data, list):
        return [item for item in data if isinstance(item, dict)]
    return []


def _box_from_record(record: dict[str, Any], default_frame: str, default_size: tuple[float, float, float]) -> BoxTruth:
    center_data = record.get("center", record.get("position", {}))
    size_data = record.get("size", {})
    detection_id = str(record.get("detection_id", record.get("id", record.get("prim_path", "isaac_box"))))
    return BoxTruth(
        detection_id=detection_id.replace("/", "_").strip("_") or "isaac_box",
        frame_id=str(record.get("frame_id", default_frame)),
        center=(
            _number(center_data, "x", 0.0),
            _number(center_data, "y", 0.0),
            _number(center_data, "z", 0.0),
        ),
        size=(
            _number(size_data, "x", default_size[0], aliases=("length",)),
            _number(size_data, "y", default_size[1], aliases=("width",)),
            _number(size_data, "z", default_size[2], aliases=("height",)),
        ),
        yaw=float(record.get("yaw", 0.0)),
        confidence=float(record.get("confidence", 1.0)),
        visible=bool(record.get("visible", True)),
    )


def _number(data: Any, key: str, default: float, aliases: tuple[str, ...] = ()) -> float:
    if isinstance(data, dict):
        for name in (key, *aliases):
            if name in data:
                return float(data[name])
    if isinstance(data, (list, tuple)):
        index = {"x": 0, "y": 1, "z": 2}[key]
        if index < len(data):
            return float(data[index])
    return float(default)


def _default_wall_records(default_frame: str, default_size: tuple[float, float, float]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    length, width, height = default_size
    origin_x = 0.75
    origin_y = 0.80
    origin_z = 1.80
    for row in range(5):
        for col in range(5):
            records.append(
                {
                    "id": f"isaac_box_r{row}_c{col}",
                    "frame_id": default_frame,
                    "center": {
                        "x": origin_x,
                        "y": origin_y - width * col,
                        "z": origin_z - height * row,
                    },
                    "size": {"x": length, "y": width, "z": height},
                    "yaw": 0.0,
                    "confidence": 1.0,
                }
            )
    return records
