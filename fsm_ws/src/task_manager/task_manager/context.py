from dataclasses import dataclass, field
from typing import Any

from fsm_core.state_context import ErrorReportData


@dataclass
class TaskContext:
    task_id: str = ""
    task_params_json: str = "{}"
    task_start_time: float = 0.0
    system_mode: str = "BOOTING"
    safety_status: Any = None
    perception_health: Any = None
    task_state: str = "WAIT_TASK"
    pending_start: dict[str, str] | None = None
    clear_error_stage_reached: int = 0
    pause_requested: bool = False
    resume_requested: bool = False
    cancel_requested: bool = False
    wall_destacking_action_client: Any = None
    wall_destacking_goal_handle: Any = None
    wall_destacking_goal_future: Any = None
    wall_destacking_result_future: Any = None
    wall_destacking_cancel_future: Any = None
    wall_destacking_result: Any = None
    wall_index: int = 0
    phase: int = 0
    last_error: ErrorReportData | None = None
    error_history: list[ErrorReportData] = field(default_factory=list)
    config: Any = None
