from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ErrorReportData:
    error_code: int
    level: int = 0
    source: int = 0
    source_node: str = ""
    source_fsm: str = ""
    source_state: str = ""
    message: str = ""
    recommended_recovery: str = ""
    extra_json: str = ""


@dataclass
class StateContext:
    task_id: str = ""
    node_name: str = ""
    current_state: str = ""
    current_fsm: str = ""
    retry_count: int = 0
    wall_index: int = 0
    phase: int = 0
    last_error: ErrorReportData | None = None
    error_history: list[ErrorReportData] = field(default_factory=list)
    config: Any = None
