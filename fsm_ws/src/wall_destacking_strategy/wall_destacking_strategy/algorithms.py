from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import sqrt


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class PlaneFitResult:
    normal: Point3
    offset: float
    centroid: Point3
    inlier_indices: tuple[int, ...]
    mean_abs_error: float
    confidence: float


@dataclass(frozen=True)
class GridAssignment:
    row: int
    col: int
    source_index: int
    point: Point3


@dataclass(frozen=True)
class AABB:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


def fit_wall_plane_ransac(
    points: list[Point3] | tuple[Point3, ...],
    distance_threshold: float = 0.03,
    min_inliers: int = 8,
) -> PlaneFitResult:
    pts = tuple((float(x), float(y), float(z)) for x, y, z in points)
    if len(pts) < 3:
        raise ValueError("at least 3 points are required")

    best: tuple[int, float, Point3, float, tuple[int, ...]] | None = None
    threshold = max(float(distance_threshold), 1e-6)
    for i, j, k in combinations(range(len(pts)), 3):
        normal = _plane_normal(pts[i], pts[j], pts[k])
        norm = _length(normal)
        if norm <= 1e-9:
            continue
        normal = _orient_normal_x_positive(_scale(normal, 1.0 / norm))
        offset = -_dot(normal, pts[i])
        distances = tuple(abs(_dot(normal, pt) + offset) for pt in pts)
        inliers = tuple(index for index, distance in enumerate(distances) if distance <= threshold)
        if len(inliers) < int(min_inliers):
            continue
        mean_error = sum(distances[index] for index in inliers) / len(inliers)
        score = (len(inliers), -mean_error)
        if best is None or score > (best[0], -best[1]):
            best = (len(inliers), mean_error, normal, offset, inliers)

    if best is None:
        raise ValueError("no plane has enough inliers")

    _, _, normal, _, inliers = best
    centroid = _centroid(tuple(pts[index] for index in inliers))
    offset = -_dot(normal, centroid)
    mean_error = sum(abs(_dot(normal, pts[index]) + offset) for index in inliers) / len(inliers)
    confidence = (len(inliers) / len(pts)) * max(0.0, 1.0 - mean_error / threshold)
    return PlaneFitResult(
        normal=normal,
        offset=float(offset),
        centroid=centroid,
        inlier_indices=inliers,
        mean_abs_error=float(mean_error),
        confidence=float(min(confidence, 1.0)),
    )


def assign_grid_indices_by_yz(points: list[Point3] | tuple[Point3, ...], rows: int = 5, cols: int = 5) -> tuple[GridAssignment, ...]:
    expected = int(rows) * int(cols)
    pts = tuple((float(x), float(y), float(z)) for x, y, z in points)
    if len(pts) < expected:
        raise ValueError(f"need at least {expected} points, got {len(pts)}")
    indexed = list(enumerate(pts))
    indexed.sort(key=lambda item: float(item[1][2]), reverse=True)
    assignments: list[GridAssignment] = []
    for row in range(int(rows)):
        row_items = indexed[row * int(cols) : (row + 1) * int(cols)]
        row_items.sort(key=lambda item: float(item[1][1]), reverse=True)
        for col, (source_index, point) in enumerate(row_items):
            assignments.append(GridAssignment(row=row, col=col, source_index=source_index, point=point))
    assignments.sort(key=lambda item: (item.row, item.col))
    return tuple(assignments)


def point_in_aabb(point: Point3, box: AABB, margin: float = 0.0) -> bool:
    x, y, z = point
    pad = max(float(margin), 0.0)
    return (
        box.x_min - pad <= float(x) <= box.x_max + pad
        and box.y_min - pad <= float(y) <= box.y_max + pad
        and box.z_min - pad <= float(z) <= box.z_max + pad
    )


def _plane_normal(a: Point3, b: Point3, c: Point3) -> Point3:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    return (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )


def _orient_normal_x_positive(normal: Point3) -> Point3:
    return _scale(normal, -1.0) if normal[0] < 0.0 else normal


def _centroid(points: tuple[Point3, ...]) -> Point3:
    count = len(points)
    return (
        sum(point[0] for point in points) / count,
        sum(point[1] for point in points) / count,
        sum(point[2] for point in points) / count,
    )


def _dot(a: Point3, b: Point3) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def _length(value: Point3) -> float:
    return sqrt(_dot(value, value))


def _scale(value: Point3, factor: float) -> Point3:
    return (float(value[0] * factor), float(value[1] * factor), float(value[2] * factor))
