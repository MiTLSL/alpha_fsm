from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
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


@dataclass(frozen=True)
class GlobalGraspAction:
    phase: int
    columns: tuple[int, ...]
    cost: int
    phase_move: bool
    same_layer_adjacent: bool
    before_heights: tuple[int, ...]
    after_heights: tuple[int, ...]

    @property
    def is_dual(self) -> bool:
        return len(self.columns) == 2


@dataclass(frozen=True)
class GlobalGraspPlan:
    actions: tuple[GlobalGraspAction, ...]
    total_cost: int
    grasp_count: int
    phase_moves: int
    expanded_states: int
    searched_edges: int


def plan_global_grasp_sequence(
    heights: list[int] | tuple[int, ...],
    current_phase: int,
    *,
    rows: int = 5,
    left_phase_cols: list[int] | tuple[int, ...] = (0, 1, 2),
    right_phase_cols: list[int] | tuple[int, ...] = (2, 3, 4),
    allow_single_arm: bool = True,
    max_adjacent_height_delta: int = 1,
    cost_dual: int = 1,
    cost_same_layer_adjacent: int = 3,
    cost_single: int = 5,
    cost_phase_move: int = 10,
) -> GlobalGraspPlan:
    start_heights = tuple(max(0, int(value)) for value in heights)
    start = (start_heights, int(current_phase))
    dist: dict[tuple[tuple[int, ...], int], tuple[int, int, int]] = {start: (0, 0, 0)}
    prev: dict[tuple[tuple[int, ...], int], tuple[tuple[tuple[int, ...], int], GlobalGraspAction]] = {}
    heap: list[tuple[int, int, int, int, tuple[tuple[int, ...], int]]] = [(0, 0, 0, 0, start)]
    push_index = 0
    expanded_states = 0
    searched_edges = 0
    goal: tuple[tuple[int, ...], int] | None = None
    inf_key = (10**12, 10**12, 10**12)

    while heap:
        total_cost, phase_moves, grasp_count, _, state = heappop(heap)
        if dist.get(state, inf_key) != (total_cost, phase_moves, grasp_count):
            continue
        expanded_states += 1
        state_heights, phase = state
        if all(value == 0 for value in state_heights):
            goal = state
            break
        for action in build_global_grasp_actions(
            state_heights,
            phase,
            rows=rows,
            left_phase_cols=left_phase_cols,
            right_phase_cols=right_phase_cols,
            allow_single_arm=allow_single_arm,
            max_adjacent_height_delta=max_adjacent_height_delta,
            cost_dual=cost_dual,
            cost_same_layer_adjacent=cost_same_layer_adjacent,
            cost_single=cost_single,
            cost_phase_move=cost_phase_move,
        ):
            searched_edges += 1
            next_state = (action.after_heights, int(action.phase))
            next_key = (
                total_cost + int(action.cost),
                phase_moves + (1 if action.phase_move else 0),
                grasp_count + 1,
            )
            if next_key >= dist.get(next_state, inf_key):
                continue
            dist[next_state] = next_key
            prev[next_state] = (state, action)
            push_index += 1
            heappush(heap, (next_key[0], next_key[1], next_key[2], push_index, next_state))

    if goal is None:
        return GlobalGraspPlan(
            actions=(),
            total_cost=-1,
            grasp_count=0,
            phase_moves=0,
            expanded_states=expanded_states,
            searched_edges=searched_edges,
        )

    actions: list[GlobalGraspAction] = []
    cursor = goal
    while cursor != start:
        parent, action = prev[cursor]
        actions.append(action)
        cursor = parent
    actions.reverse()
    total_cost, phase_moves, grasp_count = dist[goal]
    return GlobalGraspPlan(
        actions=tuple(actions),
        total_cost=int(total_cost),
        grasp_count=int(grasp_count),
        phase_moves=int(phase_moves),
        expanded_states=int(expanded_states),
        searched_edges=int(searched_edges),
    )


def build_global_grasp_actions(
    heights: list[int] | tuple[int, ...],
    current_phase: int,
    *,
    rows: int = 5,
    left_phase_cols: list[int] | tuple[int, ...] = (0, 1, 2),
    right_phase_cols: list[int] | tuple[int, ...] = (2, 3, 4),
    allow_single_arm: bool = True,
    max_adjacent_height_delta: int = 1,
    cost_dual: int = 1,
    cost_same_layer_adjacent: int = 3,
    cost_single: int = 5,
    cost_phase_move: int = 10,
) -> tuple[GlobalGraspAction, ...]:
    before = tuple(max(0, int(value)) for value in heights)
    cols_by_phase = (
        (0, _normal_phase_cols(left_phase_cols, len(before))),
        (1, _normal_phase_cols(right_phase_cols, len(before))),
    )
    actions: list[GlobalGraspAction] = []
    for target_phase, phase_cols in cols_by_phase:
        available_cols = [col for col in phase_cols if before[col] > 0]
        for cols in combinations(available_cols, 2):
            action = _make_global_grasp_action(
                before,
                int(current_phase),
                int(target_phase),
                tuple(cols),
                rows=rows,
                max_adjacent_height_delta=max_adjacent_height_delta,
                cost_dual=cost_dual,
                cost_same_layer_adjacent=cost_same_layer_adjacent,
                cost_single=cost_single,
                cost_phase_move=cost_phase_move,
            )
            if action is not None:
                actions.append(action)
        if allow_single_arm:
            for col in available_cols:
                action = _make_global_grasp_action(
                    before,
                    int(current_phase),
                    int(target_phase),
                    (col,),
                    rows=rows,
                    max_adjacent_height_delta=max_adjacent_height_delta,
                    cost_dual=cost_dual,
                    cost_same_layer_adjacent=cost_same_layer_adjacent,
                    cost_single=cost_single,
                    cost_phase_move=cost_phase_move,
                )
                if action is not None:
                    actions.append(action)
    return tuple(actions)


def _make_global_grasp_action(
    before: tuple[int, ...],
    current_phase: int,
    target_phase: int,
    columns: tuple[int, ...],
    *,
    rows: int,
    max_adjacent_height_delta: int,
    cost_dual: int,
    cost_same_layer_adjacent: int,
    cost_single: int,
    cost_phase_move: int,
) -> GlobalGraspAction | None:
    after = list(before)
    for col in columns:
        if col < 0 or col >= len(after) or after[col] <= 0:
            return None
        after[col] -= 1
    after_tuple = tuple(after)
    if not _global_heights_safe(after_tuple, max_adjacent_height_delta):
        return None
    phase_move = int(target_phase) != int(current_phase)
    same_layer_adjacent = False
    if len(columns) == 2:
        row_a = int(rows) - int(before[columns[0]])
        row_b = int(rows) - int(before[columns[1]])
        same_layer_adjacent = row_a == row_b and abs(int(columns[0]) - int(columns[1])) == 1
    if phase_move:
        cost = int(cost_phase_move)
    elif len(columns) == 1:
        cost = int(cost_single)
    elif same_layer_adjacent:
        cost = int(cost_same_layer_adjacent)
    else:
        cost = int(cost_dual)
    return GlobalGraspAction(
        phase=int(target_phase),
        columns=tuple(int(col) for col in columns),
        cost=int(cost),
        phase_move=bool(phase_move),
        same_layer_adjacent=bool(same_layer_adjacent),
        before_heights=before,
        after_heights=after_tuple,
    )


def _normal_phase_cols(cols: list[int] | tuple[int, ...], col_count: int) -> tuple[int, ...]:
    return tuple(sorted({int(col) for col in cols if 0 <= int(col) < int(col_count)}))


def _global_heights_safe(heights: tuple[int, ...], max_adjacent_height_delta: int) -> bool:
    max_delta = int(max_adjacent_height_delta)
    return all(abs(int(heights[col]) - int(heights[col + 1])) <= max_delta for col in range(len(heights) - 1))


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
