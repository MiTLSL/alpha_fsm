import unittest
from types import SimpleNamespace

from fsm_core.constants import GraspMode, SlotStatus
from fsm_core.error_code import ErrorCode
from fsm_msgs.msg import GridSlotState, SafetyStatus
from fsm_test.mocks.common import make_pose_stamped
from task_manager.context import TaskContext
from task_manager.node import (
    SYSTEM_AUTO_MODE,
    SYSTEM_BOOTING,
    SYSTEM_E_STOP,
    SYSTEM_FAULT,
    SYSTEM_MANUAL_MODE,
    SYSTEM_PAUSED,
    SYSTEM_SHUTDOWN,
    SYSTEM_SELF_CHECK,
    SYSTEM_STANDBY,
    TASK_ACCEPT,
    TASK_CANCEL,
    TASK_COMPLETE,
    TASK_FAIL,
    TASK_PREPARE,
    TASK_RUN,
    TASK_VALIDATE,
    TASK_VERIFY,
    TASK_WAIT,
    TaskManagerNodeMixin,
)
from wall_destacking_strategy.node import _StrategyFailure, WallDestackingStrategyNodeMixin


class _NoopLogger:
    def info(self, *args, **kwargs):
        return None

    def warning(self, *args, **kwargs):
        return None

    def error(self, *args, **kwargs):
        return None

    def debug(self, *args, **kwargs):
        return None


class _TaskManagerHarness(TaskManagerNodeMixin):
    def __init__(self):
        self.config = {}
        self.ctx = TaskContext(config=self.config)
        self._system_state = SYSTEM_BOOTING
        self._task_state = TASK_WAIT
        self._ready_state = self._system_state
        self._state_enter_monotonic = 0.0
        self._task_state_enter_monotonic = 0.0
        self._last_error_code = 0
        self._event_log = []

    def get_name(self):
        return "task_manager_harness"

    def get_logger(self):
        return _NoopLogger()

    def publish_state_heartbeat(self):
        return None

    def publish_task_state(self):
        return None

    def _publish_log_event(self, *args, **kwargs):
        self._event_log.append(("log", args, kwargs))

    def _publish_error(self, error):
        self._event_log.append(("error", int(error.error_code)))

    def _cancel_wall_destacking_goal(self):
        return None

    def _send_wall_destacking_goal(self):
        return None

    def _call_base_recovery(self, command: int, timeout_sec: float):
        del command, timeout_sec
        return True, "ok", 0


class _StrategyHarness(WallDestackingStrategyNodeMixin):
    def __init__(self, config=None):
        self.config = config or {}
        self._rows = int(self.config.get("business.grid_shape.rows", 5))
        self._cols = int(self.config.get("business.grid_shape.cols", 5))
        self._left_phase_cols = [0, 1, 2]
        self._right_phase_cols = [2, 3, 4]
        self._allow_single_arm = bool(self.config.get("business.allow_single_arm_grasp", True))
        self._max_adjacent_height_delta = int(self.config.get("business.max_adjacent_column_height_delta", 1))
        self._grid_slots = []
        self._slot_sizes_by_id = {}
        self._current_task_id = "task001"
        self._current_wall_index = 0
        self._current_phase = 0
        self._pair_sequence = 0
        self._active_substate_pub = None
        self._active_substate_fsm = ""
        self._active_substate_state = "IDLE"
        self._active_substate_extra = {}
        self._active_substate_enter_monotonic = 0.0
        self._wall_state = "IDLE"
        self._ready_state = self._wall_state
        self._state_enter_monotonic = 0.0
        self._last_error_code = 0

    def get_name(self):
        return "strategy_harness"

    def publish_state_heartbeat(self):
        return None

    def _publish_active_substate(self):
        return None


def _make_slot(row: int, col: int, x: float = 0.6, y: float | None = None, z: float | None = None) -> GridSlotState:
    slot = GridSlotState()
    slot.slot_id = f"wall_0_row_{row}_col_{col}"
    slot.wall_index = 0
    slot.row_index = row
    slot.col_index = col
    slot.status = int(SlotStatus.OCCUPIED)
    slot.visible = True
    slot.confidence = 1.0
    slot.retry_count = 0
    slot.last_error_code = 0
    slot.latest_pose_robot = make_pose_stamped(
        "base_link",
        x,
        (2 - col) * 0.4 if y is None else y,
        1.8 - row * 0.4 if z is None else z,
    )
    slot.expected_pose_robot = make_pose_stamped("base_link", x, 0.0, 0.0)
    return slot


def _make_slots_for_heights(heights: list[int]) -> list[GridSlotState]:
    slots = []
    for row in range(5):
        for col, height in enumerate(heights):
            top_row = 5 - int(height)
            slot = _make_slot(row, col)
            if row < top_row:
                slot.status = int(SlotStatus.REMOVED)
                slot.visible = False
            else:
                slot.status = int(SlotStatus.OCCUPIED)
                slot.visible = row == top_row
            slots.append(slot)
    return slots


class TestTaskManagerStates(unittest.TestCase):
    def test_robot_system_boot_self_check_standby_fault_estop_and_reserved_modes(self):
        harness = _TaskManagerHarness()

        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_SELF_CHECK)
        self.assertEqual(harness._task_state, TASK_WAIT)

        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_STANDBY)

        harness._system_state = SYSTEM_AUTO_MODE
        harness._task_state = TASK_FAIL
        harness.ctx.last_error = SimpleNamespace(error_code=int(ErrorCode.E_TASK_CHILD_FAILED))
        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_FAULT)

        harness._system_state = SYSTEM_AUTO_MODE
        harness._task_state = TASK_RUN
        status = SafetyStatus()
        status.estop = True
        status.estop_source = "hardware"
        harness.ctx.safety_status = status
        harness._sync_estop_state()
        self.assertEqual(harness._system_state, SYSTEM_E_STOP)
        self.assertEqual(harness._task_state, TASK_FAIL)

        for reserved_state in (SYSTEM_MANUAL_MODE, SYSTEM_SHUTDOWN):
            harness._system_state = reserved_state
            harness._task_state = TASK_RUN
            harness._tick_system_fsm()
            self.assertEqual(harness._system_state, reserved_state)
            self.assertEqual(harness._task_state, TASK_RUN)

    def test_task_fsm_happy_path_visits_accept_validate_prepare_run_verify_complete_and_wait(self):
        harness = _TaskManagerHarness()
        harness._set_system_state(SYSTEM_STANDBY)

        accepted, _ = harness._request_start(SimpleNamespace(task_id="task001", params_json="{}"))
        self.assertTrue(accepted)
        self.assertEqual(harness._system_state, SYSTEM_AUTO_MODE)
        self.assertEqual(harness._task_state, TASK_ACCEPT)

        harness._accept_task()
        self.assertEqual(harness._task_state, TASK_VALIDATE)
        self.assertEqual(harness.ctx.task_id, "task001")

        harness._validate_task()
        self.assertEqual(harness._task_state, TASK_PREPARE)

        harness._prepare_task()
        self.assertEqual(harness._task_state, TASK_RUN)

        harness._task_state = TASK_VERIFY
        harness.ctx.wall_destacking_result = SimpleNamespace(success=True, error_code=0, failure_reason="")
        harness._verify_task_result()
        self.assertEqual(harness._task_state, TASK_COMPLETE)

        harness._complete_task()
        self.assertEqual(harness._task_state, TASK_WAIT)
        self.assertEqual(harness.ctx.task_id, "")
        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_STANDBY)

    def test_task_pause_resume_returns_to_auto_mode_and_restarts_task(self):
        harness = _TaskManagerHarness()
        harness._set_system_state(SYSTEM_AUTO_MODE)
        harness._set_task_state(TASK_PREPARE)
        harness.ctx.task_id = "task_pause"
        harness.ctx.task_params_json = '{"phase":"demo"}'

        accepted, _ = harness._request_pause(SimpleNamespace())
        self.assertTrue(accepted)
        self.assertEqual(harness._system_state, SYSTEM_PAUSED)
        self.assertTrue(harness.ctx.pause_requested)
        self.assertEqual(harness.ctx.paused_task_id, "task_pause")

        harness._settle_pause_if_needed()
        self.assertEqual(harness._task_state, TASK_WAIT)

        accepted, _ = harness._request_resume(SimpleNamespace())
        self.assertTrue(accepted)
        self.assertEqual(harness._system_state, SYSTEM_AUTO_MODE)
        self.assertEqual(harness._task_state, TASK_ACCEPT)
        self.assertEqual(harness.ctx.pending_start["task_id"], "task_pause")

    def test_task_cancel_path_enters_cancel_then_wait(self):
        harness = _TaskManagerHarness()
        harness._set_system_state(SYSTEM_AUTO_MODE)
        harness._set_task_state(TASK_RUN)
        harness.ctx.task_id = "task_cancel"
        harness.ctx.task_params_json = "{}"

        accepted, _ = harness._request_cancel(SimpleNamespace())
        self.assertTrue(accepted)
        self.assertEqual(harness._task_state, TASK_CANCEL)

        harness._cancel_task()
        self.assertEqual(harness._task_state, TASK_WAIT)
        self.assertFalse(harness.ctx.cancel_requested)
        self.assertEqual(harness.ctx.last_error.error_code, int(ErrorCode.E_MAN_CANCELLED))

        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_STANDBY)

    def test_task_validate_fail_promotes_system_fault(self):
        harness = _TaskManagerHarness()
        harness._set_system_state(SYSTEM_AUTO_MODE)
        harness._set_task_state(TASK_FAIL)
        harness.ctx.last_error = SimpleNamespace(error_code=int(ErrorCode.E_TASK_VALIDATE_FAIL))

        harness._tick_system_fsm()
        self.assertEqual(harness._system_state, SYSTEM_FAULT)


class TestPairSelectionStateMachine(unittest.TestCase):
    def test_l0_pair_01_selects_global_optimal_non_adjacent_dual_pair(self):
        harness = _StrategyHarness(
            {
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 1.5,
                "business.left_arm_workspace.y_min": -1.5,
                "business.left_arm_workspace.y_max": 1.5,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 2.5,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 1.5,
                "business.right_arm_workspace.y_min": -1.5,
                "business.right_arm_workspace.y_max": 1.5,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 2.5,
            }
        )
        harness._grid_slots = [_make_slot(row, col) for row in range(5) for col in range(5)]

        pair = harness._run_pair_selection_fsm(current_phase=0, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(pair.grasp_mode, int(GraspMode.DUAL))
        self.assertEqual(pair.left_slot_id, "wall_0_row_0_col_0")
        self.assertEqual(pair.right_slot_id, "wall_0_row_0_col_2")
        self.assertEqual(pair.phase, 0)
        self.assertTrue(pair.pair_id.startswith("task001_w0_p0_"))

    def test_l0_pair_02_single_slot_rejected_when_single_arm_disabled(self):
        harness = _StrategyHarness(
            {
                "business.allow_single_arm_grasp": False,
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 1.5,
                "business.left_arm_workspace.y_min": -1.5,
                "business.left_arm_workspace.y_max": 1.5,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 2.5,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 1.5,
                "business.right_arm_workspace.y_min": -1.5,
                "business.right_arm_workspace.y_max": 1.5,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 2.5,
            }
        )
        harness._grid_slots = [_make_slot(0, 0)]

        with self.assertRaises(_StrategyFailure) as ctx:
            harness._run_pair_selection_fsm(current_phase=0, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(ctx.exception.error_code, int(ErrorCode.E_PAIR_SINGLE_NOT_ALLOWED))

    def test_l0_pair_03_no_reachable_candidate_returns_3310(self):
        harness = _StrategyHarness(
            {
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 0.5,
                "business.left_arm_workspace.y_min": -0.2,
                "business.left_arm_workspace.y_max": 0.2,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 1.0,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 0.5,
                "business.right_arm_workspace.y_min": -0.2,
                "business.right_arm_workspace.y_max": 0.2,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 1.0,
            }
        )
        harness._grid_slots = [
            _make_slot(row, col, x=1.5, y=(2 - col) * 0.4, z=1.8 - row * 0.4)
            for row in range(5)
            for col in range(5)
        ]

        with self.assertRaises(_StrategyFailure) as ctx:
            harness._run_pair_selection_fsm(current_phase=0, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(ctx.exception.error_code, int(ErrorCode.E_PAIR_NO_REACHABLE))

    def test_l0_pair_04_keeps_current_phase_when_safe_and_downgrades_unsafe_dual_to_single(self):
        harness = _StrategyHarness(
            {
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 1.5,
                "business.left_arm_workspace.y_min": -1.5,
                "business.left_arm_workspace.y_max": 1.5,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 2.5,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 1.5,
                "business.right_arm_workspace.y_min": -1.5,
                "business.right_arm_workspace.y_max": 1.5,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 2.5,
            }
        )
        harness._grid_slots = _make_slots_for_heights([0, 1, 1, 2, 1])

        pair = harness._run_pair_selection_fsm(current_phase=0, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(pair.phase, 0)
        self.assertEqual(pair.grasp_mode, int(GraspMode.LEFT_ONLY))
        self.assertEqual(pair.left_slot_id, "wall_0_row_4_col_1")
        self.assertEqual(harness._simulate_candidate_heights([0, 1, 1, 2, 1], {"left_slot": harness._grid_slots[4 * 5 + 1]}), [0, 0, 1, 2, 1])

    def test_l0_pair_05_switches_phase_when_current_phase_would_break_column_height_safety(self):
        harness = _StrategyHarness(
            {
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 1.5,
                "business.left_arm_workspace.y_min": -1.5,
                "business.left_arm_workspace.y_max": 1.5,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 2.5,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 1.5,
                "business.right_arm_workspace.y_min": -1.5,
                "business.right_arm_workspace.y_max": 1.5,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 2.5,
            }
        )
        harness._grid_slots = _make_slots_for_heights([0, 0, 1, 2, 1])

        pair = harness._run_pair_selection_fsm(current_phase=0, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(pair.phase, 1)
        self.assertEqual(pair.grasp_mode, int(GraspMode.DUAL))
        self.assertEqual(pair.left_slot_id, "wall_0_row_4_col_2")
        self.assertEqual(pair.right_slot_id, "wall_0_row_3_col_3")

    def test_l0_pair_06_right_phase_can_cover_overlap_column_two(self):
        harness = _StrategyHarness(
            {
                "business.left_arm_workspace.x_min": 0.0,
                "business.left_arm_workspace.x_max": 1.5,
                "business.left_arm_workspace.y_min": -1.5,
                "business.left_arm_workspace.y_max": 1.5,
                "business.left_arm_workspace.z_min": 0.0,
                "business.left_arm_workspace.z_max": 2.5,
                "business.right_arm_workspace.x_min": 0.0,
                "business.right_arm_workspace.x_max": 1.5,
                "business.right_arm_workspace.y_min": -1.5,
                "business.right_arm_workspace.y_max": 1.5,
                "business.right_arm_workspace.z_min": 0.0,
                "business.right_arm_workspace.z_max": 2.5,
            }
        )
        harness._grid_slots = _make_slots_for_heights([0, 0, 1, 1, 0])

        pair = harness._run_pair_selection_fsm(current_phase=1, fixed_place_pose_robot=make_pose_stamped("base_link", 0.5, 0.0, 0.8))

        self.assertEqual(pair.phase, 1)
        self.assertEqual(pair.grasp_mode, int(GraspMode.DUAL))
        self.assertEqual(pair.left_slot_id, "wall_0_row_4_col_2")
        self.assertEqual(pair.right_slot_id, "wall_0_row_4_col_3")

    def test_l0_pair_07_single_side_success_only_removes_successful_slot(self):
        from fsm_core.constants import ResultCode

        harness = _StrategyHarness()
        harness._grid_slots = [_make_slot(0, 0), _make_slot(0, 1)]
        pair = SimpleNamespace(
            grasp_mode=int(GraspMode.DUAL),
            left_slot_id=harness._grid_slots[0].slot_id,
            right_slot_id=harness._grid_slots[1].slot_id,
        )
        result = SimpleNamespace(result_code=int(ResultCode.SUCCESS_LEFT_ONLY))

        removed = harness._mark_pair_removed(pair, result)

        self.assertEqual(removed, 1)
        self.assertEqual(harness._grid_slots[0].status, int(SlotStatus.REMOVED))
        self.assertEqual(harness._grid_slots[1].status, int(SlotStatus.OCCUPIED))
