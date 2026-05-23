from dataclasses import dataclass, field
from typing import Any

from fsm_core.state_context import ErrorReportData


@dataclass
class TaskContext:
    task_id: str = ""
    task_start_time: float = 0.0
    system_mode: str = "BOOTING"
    safety_status: Any = None
    task_state: str = "WAIT_TASK"
    pause_requested: bool = False
    cancel_requested: bool = False
    wall_destacking_action_client: Any = None
    wall_destacking_goal_handle: Any = None
    wall_destacking_result: Any = None
    last_error: ErrorReportData | None = None
    error_history: list[ErrorReportData] = field(default_factory=list)
    config: Any = None
