from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from fsm_core.state_context import ErrorReportData
from .data import BoxDetection, GraspPair, NavResult, PairGraspResult, WallGrid


@dataclass
class WallDestackingContext:
    task_id: str = ""
    wall_index: int = 0
    current_wall_grid: WallGrid | None = None
    new_wall_detected: bool = False
    empty_confirm_count: int = 0
    current_phase: str = "LEFT_PHASE"
    last_local_detections: list[BoxDetection] = field(default_factory=list)
    visible_slot_ids: list[str] = field(default_factory=list)
    current_grasp_pair: GraspPair | None = None
    last_pair_grasp_result: PairGraspResult | None = None
    pair_retry_count: int = 0
    last_error: ErrorReportData | None = None
    wall_mapping_engine: Any = None
    phase_perception_engine: Any = None
    pair_selection_engine: Any = None
    wall_recovery_engine: Any = None
    nav_action_client: Any = None
    pair_grasp_action_client: Any = None
    global_scan_client: Any = None
    local_scan_client: Any = None
    last_nav_result: NavResult | None = None
    config: Any = None
