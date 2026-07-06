from __future__ import annotations

import json
import math
import time
from itertools import combinations

from .algorithms import AABB, assign_grid_indices_by_yz, fit_wall_plane_ransac, plan_global_grasp_sequence, point_in_aabb


class _StrategyCancelled(Exception):
    pass


class _StrategyFailure(Exception):
    def __init__(self, error_code: int, reason: str, recovery: dict | None = None):
        super().__init__(reason)
        self.error_code = int(error_code)
        self.reason = reason
        self.recovery = recovery


class WallDestackingStrategyNodeMixin:
    def _init_strategy_runtime(self) -> None:
        from fsm_core.ros2_helpers import get_action_name, get_topic_name, make_qos_profile
        from fsm_msgs.action import ExecutePairGrasp, NavigateToPose
        from fsm_msgs.msg import FsmStateSnapshot, GraspPair, WallGridSnapshot
        from rclpy.action import ActionClient

        from .context import WallDestackingContext

        self.ctx = WallDestackingContext(config=self.config)
        self._strategy_goal_active = False
        self._wall_state = "IDLE"
        self._state_enter_monotonic = time.monotonic()
        self._goal_start_monotonic = self._state_enter_monotonic
        self._current_task_id = ""
        self._current_wall_index = 0
        self._current_phase = 0
        self._last_error_code = 0
        self._total_boxes_picked = 0
        self._pair_sequence = 0
        self._grid_slots = []
        self._slot_sizes_by_id = {}
        self._wall_plane_estimate = None
        self._wall_frame_pose_robot = None
        self._last_detections_msg = None
        self._last_detections_monotonic = 0.0
        self._last_detection_count = 0
        self._last_perception_error = 0
        self._last_perception_health_monotonic = 0.0
        self._invalid_detection_frame_warnings = set()
        self._estop = False
        self._active_nav_goal_handle = None
        self._active_grasp_goal_handle = None
        self._active_substate_fsm = ""
        self._active_substate_state = "IDLE"
        self._active_substate_enter_monotonic = time.monotonic()
        self._active_substate_extra = {}
        self._last_nav_feedback_states = []
        self._last_nav_alignment_error_current = float("nan")
        self._last_grasp_feedback_states = []
        self._last_grasp_pressure_min_left = 0.0
        self._last_grasp_pressure_min_right = 0.0
        self._last_recovery_action = "NONE"
        self._recovery_counters = {}
        self._refresh_strategy_config()

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            get_action_name(self, "navigate_to_pose", "/navigate_to_pose"),
            callback_group=self._action_group,
        )
        self._grasp_client = ActionClient(
            self,
            ExecutePairGrasp,
            get_action_name(self, "execute_pair_grasp", "/execute_pair_grasp"),
            callback_group=self._action_group,
        )
        self._grid_snapshot_pub = self.create_publisher(
            WallGridSnapshot,
            get_topic_name(self, "fsm_grid_snapshot", "/fsm/grid_snapshot"),
            make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1),
        )
        self._grasp_pair_pub = self.create_publisher(
            GraspPair,
            get_topic_name(self, "fsm_grasp_pair", "/fsm/grasp_pair"),
            make_qos_profile("RELIABLE", "VOLATILE", 5),
        )
        self._active_substate_pub = self.create_publisher(
            FsmStateSnapshot,
            get_topic_name(self, "fsm_active_substate", "/fsm/active_substate"),
            make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1),
        )

    def on_config_reloaded(self) -> None:
        self._refresh_strategy_config()

    def _refresh_strategy_config(self) -> None:
        self._rows = int(self.config.get("business.grid_shape.rows", 5))
        self._cols = int(self.config.get("business.grid_shape.cols", 5))
        self._left_phase_cols = [int(col) for col in self.config.get("business.left_phase_cols", [0, 1, 2])]
        self._right_phase_cols = [int(col) for col in self.config.get("business.right_phase_cols", [2, 3, 4])]
        self._allow_single_arm = bool(self.config.get("business.allow_single_arm_grasp", True))
        self._max_adjacent_height_delta = int(self.config.get("business.max_adjacent_column_height_delta", 1))

    def on_detections(self, msg):
        self._last_detections_msg = msg
        self._last_detections_monotonic = time.monotonic()
        self._last_detection_count = len(msg.detections)

    def on_perception_health(self, msg):
        self._last_perception_error = int(msg.error_code)
        self._last_perception_health_monotonic = time.monotonic()

    def on_safety_status(self, msg):
        self._estop = bool(msg.estop)
        if self._estop and self._strategy_goal_active:
            self._set_active_substate("WallDestackingFSM", "ESTOP_CANCEL_CHILDREN", {"estop_source": str(msg.estop_source)})
            self._cancel_active_children()

    def handle_goal(self, goal_request):
        del goal_request
        from rclpy.action import GoalResponse

        if self._strategy_goal_active or self._estop:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

        self._set_active_substate("WallDestackingFSM", "CANCEL_REQUESTED", {})
        self._cancel_active_children()
        return CancelResponse.ACCEPT

    async def execute_wall_destacking(self, goal_handle):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import RunWallDestacking

        self._strategy_goal_active = True
        self._goal_start_monotonic = time.monotonic()
        self._current_task_id = goal_handle.request.task_id or f"task_{int(time.time())}"
        self._current_wall_index = int(goal_handle.request.start_wall_index)
        self._current_phase = 0
        self._last_error_code = 0
        self._total_boxes_picked = 0
        self._pair_sequence = 0
        self._grid_slots = []
        self._slot_sizes_by_id = {}
        self._wall_plane_estimate = None
        self._wall_frame_pose_robot = None
        self._last_nav_feedback_states = []
        self._last_grasp_feedback_states = []
        self._last_grasp_pressure_min_left = 0.0
        self._last_grasp_pressure_min_right = 0.0
        self._last_recovery_action = "NONE"
        self._recovery_counters = {}
        self.ctx.task_id = self._current_task_id
        self.ctx.wall_index = self._current_wall_index

        try:
            options = self._parse_options(goal_handle.request.config_overrides_json)
            dry_run = bool(options.get("dry_run", False))

            self._set_wall_state("NAVIGATE_TO_OBSERVATION_POSE")
            self._publish_feedback(goal_handle, "NAVIGATE_TO_OBSERVATION_POSE", 0)
            await self._navigate_to_observation(goal_handle)

            self._set_wall_state("RUN_WALL_MAPPING")
            self._publish_feedback(goal_handle, "RUN_WALL_MAPPING", 0)
            self._grid_slots = await self._run_wall_mapping_fsm(goal_handle)
            self._publish_grid_snapshot()

            self._current_phase = 0
            self.ctx.current_phase = self._phase_state_name(self._current_phase)
            self._set_wall_state("NAVIGATE_TO_PHASE_WORKPOSE")
            self._publish_feedback(goal_handle, "NAVIGATE_TO_PHASE_WORKPOSE", self._wall_progress_percent())
            await self._navigate_to_phase_workpose(goal_handle, self._current_phase)

            while self._wall_has_occupied_slots():
                phase = int(self._current_phase)
                phase_state = self._phase_state_name(phase)
                self.ctx.current_phase = phase_state

                self._check_cancel_or_estop(goal_handle)
                self._set_wall_state("RUN_PHASE_PERCEPTION")
                self._publish_feedback(goal_handle, "RUN_PHASE_PERCEPTION", self._wall_progress_percent())
                await self._run_phase_perception_fsm(goal_handle, phase)

                self._set_wall_state("RUN_PAIR_SELECTION")
                self._publish_feedback(goal_handle, "RUN_PAIR_SELECTION", self._wall_progress_percent())
                pair_msg = self._run_pair_selection_fsm(phase, goal_handle.request.fixed_place_pose_robot)
                if pair_msg is None:
                    raise _StrategyFailure(int(ErrorCode.E_PAIR_NO_CANDIDATE), "no safe pair candidate in wall")

                selected_phase = int(pair_msg.phase)
                if selected_phase != phase:
                    self._set_wall_state("DECIDE_NEXT_PHASE")
                    self._publish_feedback(goal_handle, "DECIDE_NEXT_PHASE", self._wall_progress_percent())
                    self._current_phase = selected_phase
                    self.ctx.current_phase = self._phase_state_name(selected_phase)
                    self._set_wall_state("NAVIGATE_TO_PHASE_WORKPOSE")
                    self._publish_feedback(goal_handle, "NAVIGATE_TO_PHASE_WORKPOSE", self._wall_progress_percent())
                    await self._navigate_to_phase_workpose(goal_handle, selected_phase)
                    continue

                self.ctx.current_grasp_pair = pair_msg
                self._set_wall_state("DECIDE_NEXT_PAIR")
                self._grasp_pair_pub.publish(pair_msg)
                self._publish_feedback(goal_handle, "DECIDE_NEXT_PAIR", self._wall_progress_percent())

                self._set_wall_state("WAIT_PAIR_GRASP_RESULT")
                self._publish_feedback(goal_handle, "WAIT_PAIR_GRASP_RESULT", self._wall_progress_percent())
                grasp_result = await self._execute_pair_grasp(goal_handle, pair_msg, dry_run)
                removed = self._mark_pair_removed(pair_msg, grasp_result.result)
                self._total_boxes_picked += removed
                if not grasp_result.success:
                    code = int(grasp_result.result.error_code or ErrorCode.E_GRASP_UNKNOWN)
                    reason = grasp_result.result.failed_stage or "pair grasp failed"
                    retry_count = self._record_pair_failure(pair_msg, code, grasp_result.result)
                    max_retry = int(self.config.get("business.max_retry_per_slot", 3))
                    recovery = await self._run_wall_recovery_fsm(
                        code,
                        reason,
                        retry_count=retry_count,
                        max_attempts=max_retry,
                    )
                    self._publish_grid_snapshot()
                    if self._should_retry_pair_grasp(recovery, retry_count, max_retry):
                        self._set_wall_state("RETRY_PAIR_GRASP")
                        self._publish_feedback(goal_handle, "RETRY_PAIR_GRASP", self._wall_progress_percent())
                        continue
                    raise _StrategyFailure(code, reason, recovery)

                self._set_wall_state("UPDATE_GRID_AFTER_GRASP")
                self._publish_grid_snapshot()
                self._publish_feedback(goal_handle, "UPDATE_GRID_AFTER_GRASP", self._wall_progress_percent())

            self._set_wall_state("WALL_DONE")
            self._publish_feedback(goal_handle, "WALL_DONE", 100)
            goal_handle.succeed()
            result = RunWallDestacking.Result()
            result.success = True
            result.walls_completed = 1
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = 0
            result.failure_reason = ""
            return result
        except _StrategyCancelled:
            self._set_wall_state("CANCELLED")
            self._set_active_substate("WallDestackingFSM", "CANCELLED", {"total_boxes_picked": int(self._total_boxes_picked)})
            goal_handle.canceled()
            result = RunWallDestacking.Result()
            result.success = False
            result.walls_completed = 0
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = int(ErrorCode.E_TASK_CANCELLED)
            result.failure_reason = "cancelled"
            return result
        except _StrategyFailure as exc:
            self._last_error_code = int(exc.error_code)
            recovery = exc.recovery or await self._run_wall_recovery_fsm(int(exc.error_code), exc.reason)
            self._set_wall_state("FAILED")
            goal_handle.abort()
            result = RunWallDestacking.Result()
            result.success = False
            result.walls_completed = 0
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = int(exc.error_code)
            result.failure_reason = f"{exc.reason}; recovery_action={recovery['recovery_action']}"
            return result
        except Exception as exc:  # pragma: no cover - ROS2 运行期兜底
            self._last_error_code = int(ErrorCode.E_WALL_UNKNOWN)
            recovery = await self._run_wall_recovery_fsm(int(ErrorCode.E_WALL_UNKNOWN), str(exc))
            self._set_wall_state("FAILED")
            goal_handle.abort()
            result = RunWallDestacking.Result()
            result.success = False
            result.walls_completed = 0
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = int(ErrorCode.E_WALL_UNKNOWN)
            result.failure_reason = f"{exc}; recovery_action={recovery['recovery_action']}"
            self.get_logger().error(f"wall destacking failed: {exc}")
            return result
        finally:
            self._active_nav_goal_handle = None
            self._active_grasp_goal_handle = None
            self._strategy_goal_active = False

    def publish_state_heartbeat(self) -> None:
        if self._fsm_state_pub is None:
            return
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = "WallDestackingFSM"
        msg.current_state = self._wall_state
        msg.task_id = self._current_task_id
        msg.wall_index = int(self._current_wall_index)
        msg.phase = int(self._current_phase)
        msg.state_elapsed_sec = float(time.monotonic() - self._state_enter_monotonic)
        msg.last_error_code = int(self._last_error_code)
        msg.extra_json = json.dumps(
            {
                "detection_count": self._last_detection_count,
                "total_boxes_picked": self._total_boxes_picked,
                "last_recovery_action": self._last_recovery_action,
            },
            sort_keys=True,
        )
        self._fsm_state_pub.publish(msg)

    def _parse_options(self, raw_json: str) -> dict:
        if not raw_json:
            return {}
        data = json.loads(raw_json)
        return data if isinstance(data, dict) else {}

    def _set_wall_state(self, state: str) -> None:
        if self._wall_state == state:
            return
        self._wall_state = state
        self._ready_state = state
        self._state_enter_monotonic = time.monotonic()
        self.publish_state_heartbeat()

    def _set_active_substate(self, fsm_name: str, state: str, extra: dict | None = None) -> None:
        extra = dict(extra or {})
        changed = fsm_name != self._active_substate_fsm or state != self._active_substate_state
        if changed:
            self._active_substate_fsm = fsm_name
            self._active_substate_state = state
            self._active_substate_enter_monotonic = time.monotonic()
        self._active_substate_extra = extra
        self._publish_active_substate()

    def _publish_active_substate(self) -> None:
        if self._active_substate_pub is None:
            return
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = self._active_substate_fsm
        msg.current_state = self._active_substate_state
        msg.parent_fsm = "WallDestackingFSM"
        msg.parent_state = self._wall_state
        msg.task_id = self._current_task_id
        msg.wall_index = int(self._current_wall_index)
        msg.phase = int(self._current_phase)
        msg.state_elapsed_sec = float(time.monotonic() - self._active_substate_enter_monotonic)
        msg.last_error_code = int(self._last_error_code)
        msg.extra_json = json.dumps(self._active_substate_extra, ensure_ascii=False, sort_keys=True)
        self._active_substate_pub.publish(msg)

    def _publish_feedback(self, goal_handle, state: str, progress_percent: int) -> None:
        from fsm_msgs.action import RunWallDestacking

        feedback = RunWallDestacking.Feedback()
        feedback.current_wall_index = int(self._current_wall_index)
        feedback.current_phase = int(self._current_phase)
        feedback.current_state = state
        feedback.phase_progress_percent = int(max(0, min(100, progress_percent)))
        feedback.elapsed_sec = float(time.monotonic() - self._goal_start_monotonic)
        goal_handle.publish_feedback(feedback)

    def _check_cancel_or_estop(self, goal_handle) -> None:
        from fsm_core.error_code import ErrorCode

        if goal_handle.is_cancel_requested:
            self._cancel_active_children()
            raise _StrategyCancelled()
        if self._estop:
            self._cancel_active_children()
            raise _StrategyFailure(int(ErrorCode.E_SAFETY_ESTOP_HW), "estop active")

    async def _wait_for_future(self, future, goal_handle, timeout_sec: float, child_goal_handle=None):
        from fsm_core.error_code import ErrorCode

        deadline = time.monotonic() + float(timeout_sec)
        while not future.done():
            if goal_handle.is_cancel_requested:
                if child_goal_handle is not None:
                    self._cancel_goal_handle(child_goal_handle)
                raise _StrategyCancelled()
            if self._estop:
                if child_goal_handle is not None:
                    self._cancel_goal_handle(child_goal_handle)
                raise _StrategyFailure(int(ErrorCode.E_SAFETY_ESTOP_HW), "estop active")
            if time.monotonic() >= deadline:
                if child_goal_handle is not None:
                    self._cancel_goal_handle(child_goal_handle)
                raise _StrategyFailure(int(ErrorCode.E_COMM_ACTION_TIMEOUT), "action future timeout")
            await self._sleep(0.02)
        return future.result()

    async def _wait_for_action_server(self, client, goal_handle, timeout_sec: float, action_name: str) -> None:
        from fsm_core.error_code import ErrorCode

        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            self._check_cancel_or_estop(goal_handle)
            if client.server_is_ready() or client.wait_for_server(timeout_sec=0.0):
                return
            await self._sleep(0.05)
        raise _StrategyFailure(int(ErrorCode.E_COMM_ACTION_TIMEOUT), f"{action_name} action server unavailable")

    async def _navigate_to_observation(self, goal_handle):
        from fsm_core.constants import GoalType, Phase

        return await self._navigate_to_workpose(
            goal_handle=goal_handle,
            goal_type=GoalType.OBSERVATION,
            phase=int(Phase.LEFT),
            pose_prefix="business.observation_pose",
            require_fine_alignment=False,
            timeout_key="WallDestackingFSM_NAVIGATE_TO_OBSERVATION_POSE",
        )

    async def _navigate_to_phase_workpose(self, goal_handle, phase: int):
        from fsm_core.constants import GoalType

        goal_type = GoalType.LEFT_PHASE if int(phase) == 0 else GoalType.RIGHT_PHASE
        pose_prefix = "business.left_phase_workpose" if int(phase) == 0 else "business.right_phase_workpose"
        return await self._navigate_to_workpose(
            goal_handle=goal_handle,
            goal_type=goal_type,
            phase=int(phase),
            pose_prefix=pose_prefix,
            require_fine_alignment=True,
            timeout_key="WallDestackingFSM_NAVIGATE_TO_PHASE_WORKPOSE",
        )

    async def _navigate_to_workpose(
        self,
        goal_handle,
        goal_type: str,
        phase: int,
        pose_prefix: str,
        require_fine_alignment: bool,
        timeout_key: str,
    ):
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import NavigateToPose

        self._last_nav_feedback_states = []
        self._last_nav_alignment_error_current = float("nan")
        await self._wait_for_action_server(
            self._nav_client,
            goal_handle,
            float(self.config.get("fsm.action_send_goal_timeout_sec", 2.0)),
            "navigate_to_pose",
        )
        goal = NavigateToPose.Goal()
        goal.goal_type = str(goal_type)
        goal.target_pose = self._pose_from_business_prefix(pose_prefix, "map")
        goal.wall_frame_pose = self._identity_pose("map")
        goal.phase = int(phase)
        goal.desired_distance_to_wall = float(self.config.get("business.desired_distance_to_wall", 0.6))
        goal.desired_yaw_to_wall = float(self.config.get("business.desired_yaw_to_wall", 0.0))
        goal.desired_lateral_offset = float(goal.target_pose.pose.position.y)
        goal.require_fine_alignment = bool(require_fine_alignment)
        goal.timeout_sec = float(self.config.get(f"fsm.state_timeouts.{timeout_key}", 60.0))
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        nav_goal_handle = await self._wait_for_future(
            send_future,
            goal_handle,
            float(self.config.get("fsm.action_send_goal_timeout_sec", 2.0)),
        )
        if not nav_goal_handle.accepted:
            raise _StrategyFailure(int(ErrorCode.E_NAV_GOAL_REJECTED), "navigate_to_pose goal rejected")
        self._active_nav_goal_handle = nav_goal_handle
        result_future = nav_goal_handle.get_result_async()
        result_wrapper = await self._wait_for_future(
            result_future,
            goal_handle,
            float(goal.timeout_sec) + 2.0,
            child_goal_handle=nav_goal_handle,
        )
        self._active_nav_goal_handle = None
        if result_wrapper.status == GoalStatus.STATUS_CANCELED:
            raise _StrategyCancelled()
        result = result_wrapper.result
        if not result.success:
            raise _StrategyFailure(int(result.error_code or ErrorCode.E_NAV_UNKNOWN), result.failure_reason or "navigation failed")
        return result

    def _on_nav_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        state = str(feedback.current_state)
        if state and (not self._last_nav_feedback_states or self._last_nav_feedback_states[-1] != state):
            self._last_nav_feedback_states.append(state)
        self._last_nav_alignment_error_current = float(feedback.alignment_error_current)
        self._set_active_substate(
            "NavigationClient",
            state or "FEEDBACK",
            {
                "distance_remaining": float(feedback.distance_remaining),
                "estimated_time_remaining": float(feedback.estimated_time_remaining),
                "alignment_error_current": float(feedback.alignment_error_current),
                "feedback_states": list(self._last_nav_feedback_states),
            },
        )

    async def _execute_pair_grasp(self, goal_handle, pair_msg, dry_run: bool):
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import ExecutePairGrasp

        self._last_grasp_feedback_states = []
        self._last_grasp_pressure_min_left = 0.0
        self._last_grasp_pressure_min_right = 0.0
        await self._wait_for_action_server(
            self._grasp_client,
            goal_handle,
            float(self.config.get("fsm.action_send_goal_timeout_sec", 2.0)),
            "execute_pair_grasp",
        )
        goal = ExecutePairGrasp.Goal()
        goal.grasp_pair = pair_msg
        goal.timeout_sec = float(self.config.get("fsm.state_timeouts.WallDestackingFSM_WAIT_PAIR_GRASP_RESULT", 120.0))
        goal.dry_run = bool(dry_run)
        send_future = self._grasp_client.send_goal_async(goal, feedback_callback=self._on_grasp_feedback)
        grasp_goal_handle = await self._wait_for_future(
            send_future,
            goal_handle,
            float(self.config.get("fsm.action_send_goal_timeout_sec", 2.0)),
        )
        if not grasp_goal_handle.accepted:
            raise _StrategyFailure(int(ErrorCode.E_GRASP_GOAL_REJECTED), "execute_pair_grasp goal rejected")
        self._active_grasp_goal_handle = grasp_goal_handle
        result_future = grasp_goal_handle.get_result_async()
        result_wrapper = await self._wait_for_future(
            result_future,
            goal_handle,
            float(self.config.get("fsm.state_timeouts.WallDestackingFSM_WAIT_PAIR_GRASP_RESULT", 120.0)) + 2.0,
            child_goal_handle=grasp_goal_handle,
        )
        self._active_grasp_goal_handle = None
        if result_wrapper.status == GoalStatus.STATUS_CANCELED:
            raise _StrategyCancelled()
        return result_wrapper.result

    def _on_grasp_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        state = str(feedback.current_state)
        if state and (not self._last_grasp_feedback_states or self._last_grasp_feedback_states[-1] != state):
            self._last_grasp_feedback_states.append(state)
        self._last_grasp_pressure_min_left = min(float(self._last_grasp_pressure_min_left), float(feedback.vacuum_left_kpa))
        self._last_grasp_pressure_min_right = min(float(self._last_grasp_pressure_min_right), float(feedback.vacuum_right_kpa))
        self._set_active_substate(
            "PairGraspClient",
            state or "FEEDBACK",
            {
                "stage": str(feedback.current_stage),
                "progress_percent": float(feedback.progress_percent),
                "vacuum_left_kpa": float(feedback.vacuum_left_kpa),
                "vacuum_right_kpa": float(feedback.vacuum_right_kpa),
                "pressure_min_left_kpa": float(self._last_grasp_pressure_min_left),
                "pressure_min_right_kpa": float(self._last_grasp_pressure_min_right),
                "feedback_states": list(self._last_grasp_feedback_states),
            },
        )

    async def _run_wall_mapping_fsm(self, goal_handle):
        from fsm_core.error_code import ErrorCode

        expected = self._rows * self._cols
        frames_target = int(self.config.get("business.mapping_window.frames", 10))
        timeout_sec = float(self.config.get("business.mapping_window.timeout_sec", 3.0))
        deadline = time.monotonic() + timeout_sec
        window_start = time.monotonic()
        seen_sequences = set()
        valid_frames = []
        received_frames = 0
        invalid_frames = 0
        last_valid_count = 0

        self._set_active_substate("WallMappingFSM", "START_WINDOW", {"expected_detections": expected, "timeout_sec": timeout_sec})
        await self._sleep(0.01)
        self._set_active_substate("WallMappingFSM", "COLLECT_FRAMES", {"target_frames": frames_target})

        while time.monotonic() < deadline and len(valid_frames) < frames_target:
            self._check_cancel_or_estop(goal_handle)
            blocking_error = self._blocking_perception_error_code()
            if blocking_error:
                self._set_active_substate("WallMappingFSM", "MAPPING_ERROR", {"error_code": blocking_error})
                raise _StrategyFailure(blocking_error, f"perception health error {blocking_error}")

            msg = self._last_detections_msg
            if msg is not None and self._last_detections_monotonic >= window_start:
                sequence = int(getattr(msg, "frame_seq", 0))
                if sequence not in seen_sequences:
                    seen_sequences.add(sequence)
                    received_frames += 1
                    valid = self._valid_detections(msg)
                    last_valid_count = max(last_valid_count, len(valid))
                    if valid:
                        valid_frames.append(valid)
                    elif len(msg.detections) > 0:
                        invalid_frames += 1
                    self._set_active_substate(
                        "WallMappingFSM",
                        "COLLECT_FRAMES",
                        {
                            "received_frames": received_frames,
                            "valid_frames": len(valid_frames),
                            "last_valid_count": len(valid),
                            "invalid_frames": invalid_frames,
                        },
                    )
            await self._sleep(0.05)

        if not valid_frames:
            code = ErrorCode.E_MAP_GLOBAL_SCAN_FAIL if received_frames == 0 else ErrorCode.E_MAP_NO_DETECTION
            self._set_active_substate(
                "WallMappingFSM",
                "MAPPING_ERROR",
                {"error_code": int(code), "received_frames": received_frames, "invalid_frames": invalid_frames},
            )
            raise _StrategyFailure(int(code), "no valid mapping detection stream")

        self._set_active_substate("WallMappingFSM", "FILTER_DETECTIONS", {"valid_frames": len(valid_frames)})
        best_detections = max(valid_frames, key=len)
        if len(best_detections) < expected:
            self._set_active_substate(
                "WallMappingFSM",
                "MAPPING_ERROR",
                {"error_code": int(ErrorCode.E_MAP_INSUFFICIENT_DETECTION), "best_valid_count": len(best_detections), "expected": expected},
            )
            raise _StrategyFailure(
                int(ErrorCode.E_MAP_INSUFFICIENT_DETECTION),
                f"need {expected} detections to build grid, got {len(best_detections)}",
            )

        try:
            plane = self._estimate_wall_frame(best_detections)
        except _StrategyFailure as exc:
            self._set_active_substate("WallMappingFSM", "MAPPING_ERROR", {"error_code": int(exc.error_code), "reason": exc.reason})
            raise
        except Exception as exc:
            self._set_active_substate("WallMappingFSM", "MAPPING_ERROR", {"error_code": int(ErrorCode.E_MAP_WALL_FRAME_FAIL), "reason": str(exc)})
            raise _StrategyFailure(int(ErrorCode.E_MAP_WALL_FRAME_FAIL), f"wall frame fit failed: {exc}") from exc
        self._set_active_substate(
            "WallMappingFSM",
            "ESTIMATE_WALL_FRAME",
            {
                "normal": [round(value, 6) for value in plane.normal],
                "inliers": len(plane.inlier_indices),
                "mean_abs_error": float(plane.mean_abs_error),
                "confidence": float(plane.confidence),
            },
        )
        await self._sleep(0.01)
        self._set_active_substate(
            "WallMappingFSM",
            "BUILD_5X5_GRID",
            {"detections": len(best_detections), "wall_confidence": float(plane.confidence)},
        )
        try:
            slots = self._build_grid_slots(best_detections, self._current_wall_index)
        except Exception as exc:
            self._set_active_substate("WallMappingFSM", "MAPPING_ERROR", {"error_code": int(ErrorCode.E_MAP_GRID_BUILD_FAIL), "reason": str(exc)})
            raise _StrategyFailure(int(ErrorCode.E_MAP_GRID_BUILD_FAIL), f"grid build failed: {exc}") from exc

        self._set_active_substate("WallMappingFSM", "INIT_GRID_SLOTS", {"slot_count": len(slots)})
        if len(slots) != expected:
            self._set_active_substate("WallMappingFSM", "MAPPING_ERROR", {"error_code": int(ErrorCode.E_MAP_GRID_INCOMPLETE), "slot_count": len(slots)})
            raise _StrategyFailure(int(ErrorCode.E_MAP_GRID_INCOMPLETE), f"grid slots incomplete: {len(slots)}/{expected}")
        self._set_active_substate("WallMappingFSM", "VALIDATE_GRID", {"slot_count": len(slots), "occupied": len(slots)})
        await self._sleep(0.01)
        self._set_active_substate("WallMappingFSM", "CHECK_NEW_WALL", {"wall_index": int(self._current_wall_index)})
        await self._sleep(0.01)
        self._set_active_substate("WallMappingFSM", "REPORT", {"slot_count": len(slots), "last_valid_count": last_valid_count})
        return slots

    async def _run_phase_perception_fsm(self, goal_handle, phase: int) -> None:
        from fsm_core.constants import SlotStatus
        from fsm_core.error_code import ErrorCode

        phase_cols = self._phase_cols(phase)
        occupied_slots = [
            slot
            for slot in self._grid_slots
            if int(slot.col_index) in phase_cols and int(slot.status) == int(SlotStatus.OCCUPIED)
        ]
        timeout_sec = float(self.config.get("business.local_window.timeout_sec", 1.5))
        frames_target = int(self.config.get("business.local_window.frames", 5))
        deadline = time.monotonic() + timeout_sec
        window_start = time.monotonic()
        seen_sequences = set()
        valid_frames = []
        received_frames = 0
        invalid_frames = 0

        self._set_active_substate(
            "PhasePerceptionFSM",
            "START_LOCAL_WINDOW",
            {
                "phase": int(phase),
                "phase_cols": phase_cols,
                "occupied_slots": len(occupied_slots),
                "nav_feedback_states": list(self._last_nav_feedback_states),
            },
        )
        await self._sleep(0.01)
        self._set_active_substate("PhasePerceptionFSM", "COLLECT_LOCAL_FRAMES", {"target_frames": frames_target})

        while time.monotonic() < deadline and len(valid_frames) < frames_target:
            self._check_cancel_or_estop(goal_handle)
            blocking_error = self._blocking_perception_error_code()
            if blocking_error:
                self._set_active_substate("PhasePerceptionFSM", "PERCEPTION_ERROR", {"error_code": blocking_error})
                raise _StrategyFailure(blocking_error, f"perception health error {blocking_error}")

            msg = self._last_detections_msg
            if msg is not None and self._last_detections_monotonic >= window_start:
                sequence = int(getattr(msg, "frame_seq", 0))
                if sequence not in seen_sequences:
                    seen_sequences.add(sequence)
                    received_frames += 1
                    valid = self._valid_detections(msg)
                    if valid:
                        valid_frames.append(valid)
                    elif len(msg.detections) > 0:
                        invalid_frames += 1
                    self._set_active_substate(
                        "PhasePerceptionFSM",
                        "COLLECT_LOCAL_FRAMES",
                        {
                            "received_frames": received_frames,
                            "valid_frames": len(valid_frames),
                            "last_valid_count": len(valid),
                            "invalid_frames": invalid_frames,
                        },
                    )
            await self._sleep(0.05)

        if not occupied_slots:
            self._set_active_substate("PhasePerceptionFSM", "REPORT", {"matched_slots": 0, "reason": "no occupied slot in workpose columns"})
            return
        if received_frames == 0:
            self._set_active_substate("PhasePerceptionFSM", "PERCEPTION_ERROR", {"error_code": int(ErrorCode.E_PERC_LOCAL_SCAN_TIMEOUT)})
            raise _StrategyFailure(int(ErrorCode.E_PERC_LOCAL_SCAN_TIMEOUT), "phase perception stream timeout")
        if not valid_frames:
            self._set_active_substate(
                "PhasePerceptionFSM",
                "PERCEPTION_ERROR",
                {"error_code": int(ErrorCode.E_PERC_NO_LOCAL_DETECTION), "received_frames": received_frames, "invalid_frames": invalid_frames},
            )
            raise _StrategyFailure(int(ErrorCode.E_PERC_NO_LOCAL_DETECTION), "no local detections while phase still has occupied slots")

        self._set_active_substate("PhasePerceptionFSM", "FILTER_LOCAL", {"valid_frames": len(valid_frames)})
        best_detections = max(valid_frames, key=len)
        try:
            detections_in_robot, transform_summary = self._transform_detections_to_robot(best_detections)
        except Exception as exc:
            self._set_active_substate("PhasePerceptionFSM", "PERCEPTION_ERROR", {"error_code": int(ErrorCode.E_COMM_TF_LOOKUP_FAIL), "reason": str(exc)})
            raise _StrategyFailure(int(ErrorCode.E_COMM_TF_LOOKUP_FAIL), f"tf transform failed: {exc}") from exc
        self._set_active_substate(
            "PhasePerceptionFSM",
            "TRANSFORM_TO_ROBOT",
            {"frame_id": "base_link", "detections": len(detections_in_robot), "summary": transform_summary},
        )
        matches = self._match_detections_to_phase_slots(detections_in_robot, phase)
        self._set_active_substate("PhasePerceptionFSM", "MATCH_TO_SLOTS", {"matched_slots": len(matches), "occupied_slots": len(occupied_slots)})
        if not matches:
            self._set_active_substate("PhasePerceptionFSM", "PERCEPTION_ERROR", {"error_code": int(ErrorCode.E_PERC_ASSOCIATION_FAIL)})
            raise _StrategyFailure(int(ErrorCode.E_PERC_ASSOCIATION_FAIL), "local detections do not match current phase slots")

        now = self.get_clock().now().to_msg()
        self._set_active_substate("PhasePerceptionFSM", "UPDATE_SLOT_POSES", {"matched_slots": len(matches)})
        for slot in self._grid_slots:
            det = matches.get(slot.slot_id)
            if det is None:
                continue
            slot.latest_pose_robot = self._copy_pose_stamped(det.pose)
            slot.visible = True
            slot.confidence = float(det.confidence)
            slot.last_seen_time = now
            self._slot_sizes_by_id[slot.slot_id] = self._copy_vector3(det.size)

        unseen = 0
        self._set_active_substate("PhasePerceptionFSM", "MARK_UNSEEN", {"matched_slots": len(matches)})
        for slot in occupied_slots:
            if slot.slot_id not in matches:
                slot.visible = False
                unseen += 1
        self._set_active_substate("PhasePerceptionFSM", "REPORT", {"matched_slots": len(matches), "unseen_slots": unseen})
        self._publish_grid_snapshot()

    def _run_pair_selection_fsm(self, current_phase: int, fixed_place_pose_robot):
        from fsm_core.constants import SlotStatus
        from fsm_core.error_code import ErrorCode

        max_retry = int(self.config.get("business.max_retry_per_slot", 3))
        column_heights = self._column_heights()
        top_slot_ids = self._top_slot_ids_by_column()
        available = [
            slot
            for slot in self._grid_slots
            if slot.slot_id in top_slot_ids.values()
            and int(slot.status) == int(SlotStatus.OCCUPIED)
            and bool(slot.visible)
            and int(slot.retry_count) < max_retry
            and self._slot_has_valid_pose(slot)
        ]
        self._set_active_substate(
            "PairSelectionFSM",
            "FILTER_AVAILABLE",
            {
                "available_slots": len(available),
                "current_phase": int(current_phase),
                "column_heights": column_heights,
            },
        )
        if not available:
            self._set_active_substate("PairSelectionFSM", "REPORT", {"pair_id": "", "reason": "no available slot"})
            return None

        available.sort(key=lambda slot: (int(slot.row_index), int(slot.col_index)))
        self._set_active_substate(
            "PairSelectionFSM",
            "SORT_BY_PRIORITY",
            {"available_slots": len(available), "priority": "phase-local top then left"},
        )

        candidates_by_phase = {}
        single_blocked_by_phase = {}
        for phase in (0, 1):
            candidates, single_blocked = self._build_pair_candidates(phase, available)
            candidates_by_phase[int(phase)] = candidates
            single_blocked_by_phase[int(phase)] = single_blocked
        candidate_count = sum(len(candidates) for candidates in candidates_by_phase.values())
        self._set_active_substate(
            "PairSelectionFSM",
            "BUILD_CANDIDATES",
            {
                "allow_single_arm": bool(self._allow_single_arm),
                "candidate_count": candidate_count,
                "left_candidate_count": len(candidates_by_phase[0]),
                "right_candidate_count": len(candidates_by_phase[1]),
                "single_not_allowed": any(single_blocked_by_phase.values()),
            },
        )
        if not candidate_count:
            if any(single_blocked_by_phase.values()):
                self._set_active_substate(
                    "PairSelectionFSM",
                    "PAIR_SELECTION_ERROR",
                    {"error_code": int(ErrorCode.E_PAIR_SINGLE_NOT_ALLOWED), "reason": "single arm grasp disabled"},
                )
                raise _StrategyFailure(int(ErrorCode.E_PAIR_SINGLE_NOT_ALLOWED), "single arm grasp disabled")
            self._set_active_substate("PairSelectionFSM", "PAIR_SELECTION_ERROR", {"reason": "no candidate"})
            return None

        global_plan = self._plan_global_grasp_sequence(column_heights, current_phase)
        self._set_active_substate(
            "PairSelectionFSM",
            "PLAN_GLOBAL_SEQUENCE",
            {
                "total_cost": int(global_plan.total_cost),
                "grasp_count": int(global_plan.grasp_count),
                "phase_moves": int(global_plan.phase_moves),
                "expanded_states": int(global_plan.expanded_states),
                "searched_edges": int(global_plan.searched_edges),
                "next_action": self._global_action_summary(global_plan.actions[0]) if global_plan.actions else {},
            },
        )

        safe_candidates = []
        safety_rejections = []
        for phase, candidates in candidates_by_phase.items():
            for candidate in candidates:
                safe, reason = self._candidate_height_safe(candidate, column_heights)
                if safe:
                    safe_candidates.append(candidate)
                else:
                    safety_rejections.append(reason)
        self._set_active_substate(
            "PairSelectionFSM",
            "CHECK_HEIGHT_SAFETY",
            {
                "max_adjacent_delta": int(getattr(self, "_max_adjacent_height_delta", self.config.get("business.max_adjacent_column_height_delta", 1))),
                "left_safe": any(int(candidate["phase"]) == 0 for candidate in safe_candidates),
                "right_safe": any(int(candidate["phase"]) == 1 for candidate in safe_candidates),
                "rejection_reasons": safety_rejections[:6],
            },
        )

        if not safe_candidates:
            self._set_active_substate(
                "PairSelectionFSM",
                "PAIR_SELECTION_ERROR",
                {
                    "error_code": int(ErrorCode.E_PAIR_NO_CANDIDATE),
                    "reason": "no height-safe candidate",
                    "column_heights": column_heights,
                    "rejection_reasons": safety_rejections[:6],
                },
            )
            return None

        rejection_reasons = []
        scored_candidates = []
        for candidate in safe_candidates:
            reachable, reason = self._pair_candidate_reachable(candidate)
            if not reachable:
                rejection_reasons.append(reason)
                continue
            score = self._candidate_global_score(candidate, current_phase, column_heights)
            if score is None:
                safety_rejections.append(f"{self._candidate_label(candidate)}:no_global_plan")
                continue
            scored_candidates.append((score, candidate))
        scored_candidates.sort(key=lambda item: item[0])
        selected = scored_candidates[0][1] if scored_candidates else None
        selected_phase = int(selected["phase"]) if selected is not None else int(current_phase)
        self._set_active_substate(
            "PairSelectionFSM",
            "VALIDATE_REACHABILITY",
            {
                "selected_phase": int(selected_phase),
                "candidate_count": candidate_count,
                "reachable": selected is not None,
                "global_score": list(scored_candidates[0][0][:4]) if scored_candidates else [],
                "rejection_reasons": rejection_reasons[:4],
            },
        )
        if selected is None:
            self._set_active_substate(
                "PairSelectionFSM",
                "PAIR_SELECTION_ERROR",
                {"error_code": int(ErrorCode.E_PAIR_NO_REACHABLE), "rejection_reasons": rejection_reasons[:6]},
            )
            raise _StrategyFailure(int(ErrorCode.E_PAIR_NO_REACHABLE), "no reachable pair candidate")

        pair_msg = self._make_pair_from_candidate(selected, int(selected["phase"]), fixed_place_pose_robot)
        self._set_active_substate("PairSelectionFSM", "CHECK_DUAL_ARM_CONFLICT", {"grasp_mode": int(pair_msg.grasp_mode)})
        self._set_active_substate(
            "PairSelectionFSM",
            "ASSIGN_ARMS_BY_Y",
            {"left_slot_id": pair_msg.left_slot_id, "right_slot_id": pair_msg.right_slot_id},
        )
        self._set_active_substate("PairSelectionFSM", "BUILD_GRASP_PAIR", {"pair_id": pair_msg.pair_id})
        self._set_active_substate("PairSelectionFSM", "REPORT", {"pair_id": pair_msg.pair_id, "grasp_mode": int(pair_msg.grasp_mode)})
        return pair_msg

    async def _run_wall_recovery_fsm(
        self,
        error_code: int,
        reason: str,
        retry_count: int = 0,
        max_attempts: int | None = None,
    ) -> dict:
        from fsm_core.error_code import ErrorLevel, RecoveryAction, get_error_meta

        self._set_wall_state("WALL_ERROR_HANDLE")
        meta = get_error_meta(int(error_code))
        action_name = RecoveryAction(int(meta.default_recovery)).name
        level_name = ErrorLevel(int(meta.level)).name
        counter_key = f"{action_name}:{int(error_code)}"
        if int(retry_count) <= 0:
            retry_count = int(self._recovery_counters.get(counter_key, 0)) + 1
        else:
            retry_count = max(int(retry_count), int(self._recovery_counters.get(counter_key, 0)))
        self._recovery_counters[counter_key] = int(retry_count)
        payload = {
            "error_code": int(error_code),
            "error_name": meta.name,
            "level": level_name,
            "source": meta.source.name,
            "reason": str(reason),
            "recovery_action": action_name,
            "retry_counter_key": counter_key,
        }
        self._set_active_substate("WallRecoveryFSM", "RECEIVE_ERROR", payload)
        await self._sleep(0.01)
        self._set_active_substate("WallRecoveryFSM", "CLASSIFY", payload)
        await self._sleep(0.01)
        retry_key = f"fsm.recovery_max_attempts.{action_name}"
        resolved_max_attempts = int(max_attempts if max_attempts is not None else self.config.get(retry_key, 1))
        payload["retry_count"] = int(retry_count)
        payload["max_attempts"] = resolved_max_attempts
        self._set_active_substate("WallRecoveryFSM", "CHECK_RETRY_LIMIT", payload)
        await self._sleep(0.01)
        self._set_active_substate("WallRecoveryFSM", "SELECT_RECOVERY_ACTION", payload)
        await self._sleep(0.01)
        terminal_actions = {
            RecoveryAction.WAIT_MANUAL_RECOVERY.name,
            RecoveryAction.ABORT_TASK.name,
            RecoveryAction.E_STOP.name,
        }
        self._set_active_substate("WallRecoveryFSM", "EXECUTE_RECOVERY", payload)
        await self._sleep(0.01)
        retry_exhausted = resolved_max_attempts > 0 and int(retry_count) >= resolved_max_attempts
        final_state = (
            "WAIT_MANUAL_RECOVERY"
            if action_name in terminal_actions or level_name in ("FATAL", "ESTOP") or retry_exhausted
            else "REPORT"
        )
        self._last_recovery_action = action_name
        payload["manual_required"] = final_state == "WAIT_MANUAL_RECOVERY"
        payload["retry_exhausted"] = bool(retry_exhausted)
        self._set_active_substate("WallRecoveryFSM", final_state, payload)
        return payload

    async def _wait_for_grid_detections(self, goal_handle):
        from fsm_core.error_code import ErrorCode

        timeout_sec = float(self.config.get("business.mapping_window.timeout_sec", 3.0))
        deadline = time.monotonic() + timeout_sec
        expected = self._rows * self._cols
        last_valid_count = 0
        while time.monotonic() < deadline:
            self._check_cancel_or_estop(goal_handle)
            msg = self._last_detections_msg
            if msg is not None:
                valid = self._valid_detections(msg)
                last_valid_count = len(valid)
                if last_valid_count >= expected:
                    return valid
            await self._sleep(0.05)
        if last_valid_count == 0:
            raise _StrategyFailure(int(ErrorCode.E_MAP_NO_DETECTION), "no valid box detections")
        raise _StrategyFailure(
            int(ErrorCode.E_MAP_INSUFFICIENT_DETECTION),
            f"need {expected} detections to build grid, got {last_valid_count}",
        )

    def _valid_detections(self, msg):
        confidence_min = float(self.config.get("business.detection_filter.confidence_min", 0.5))
        expected_label = str(self.config.get("business.detection_filter.class_label", "box"))
        valid = []
        invalid_frame_count = 0
        for det in msg.detections:
            frame_id = det.pose.header.frame_id or det.header.frame_id or msg.header.frame_id
            if not self._can_transform_frame_to_robot(frame_id):
                invalid_frame_count += 1
                continue
            if not det.pose_valid:
                continue
            if float(det.confidence) < confidence_min:
                continue
            if expected_label and det.class_label and det.class_label != expected_label:
                continue
            valid.append(det)
        if invalid_frame_count:
            sequence = int(getattr(msg, "frame_seq", -1))
            if sequence not in self._invalid_detection_frame_warnings:
                self._invalid_detection_frame_warnings.add(sequence)
                self.get_logger().warning(
                    f"reject detection frame_seq={sequence}: {invalid_frame_count} detections are not in base_link"
                )
        return valid

    def _estimate_wall_frame(self, detections):
        points = [self._point_from_detection(det) for det in detections]
        plane = fit_wall_plane_ransac(
            points,
            distance_threshold=float(self.config.get("business.wall_plane.ransac_distance_threshold", 0.03)),
            min_inliers=int(self.config.get("business.mapping_window.min_valid_detections", 8)),
        )
        min_confidence = float(self.config.get("business.wall_plane.min_confidence", 0.3))
        if float(plane.confidence) < min_confidence:
            from fsm_core.error_code import ErrorCode

            raise _StrategyFailure(
                int(ErrorCode.E_MAP_WALL_FRAME_LOW_CONFIDENCE),
                f"wall frame confidence too low: {plane.confidence:.3f}",
            )
        pose = self._identity_pose("base_link")
        pose.pose.position.x = float(plane.centroid[0])
        pose.pose.position.y = float(plane.centroid[1])
        pose.pose.position.z = float(plane.centroid[2])
        self._wall_plane_estimate = plane
        self._wall_frame_pose_robot = pose
        return plane

    def _transform_detections_to_robot(self, detections) -> tuple[list[object], dict]:
        transformed = []
        summary = {"identity": 0, "static_fallback": 0}
        for det in detections:
            frame_id = self._detection_frame_id(det)
            if frame_id in ("", "base_link"):
                det.header.frame_id = "base_link"
                det.pose.header.frame_id = "base_link"
                summary["identity"] += 1
                transformed.append(det)
                continue
            if not self._can_transform_frame_to_robot(frame_id):
                raise ValueError(f"no transform from {frame_id} to base_link")
            det.header.frame_id = "base_link"
            det.pose.header.frame_id = "base_link"
            summary["static_fallback"] += 1
            transformed.append(det)
        return transformed, summary

    def _detection_frame_id(self, det) -> str:
        return det.pose.header.frame_id or det.header.frame_id or "base_link"

    def _can_transform_frame_to_robot(self, frame_id: str) -> bool:
        if frame_id in ("", "base_link"):
            return True
        allowed_frames = self.config.get(
            "business.tf_static_fallback.allowed_source_frames",
            ["camera_link", "depth_camera_link", "camera_color_optical_frame"],
        )
        return bool(self.config.get("business.tf_static_fallback.enabled", True)) and str(frame_id) in {str(item) for item in allowed_frames}

    def _point_from_detection(self, det) -> tuple[float, float, float]:
        position = det.pose.pose.position
        return (float(position.x), float(position.y), float(position.z))

    def _blocking_perception_error_code(self) -> int:
        from fsm_core.error_code import ErrorCode

        if self._last_perception_error == 0:
            return 0
        if self._last_perception_health_monotonic <= 0.0:
            return 0
        if time.monotonic() - self._last_perception_health_monotonic > 1.5:
            return 0
        blocking_codes = {
            int(ErrorCode.E_EXT_PERC_CAMERA_FAIL),
            int(ErrorCode.E_EXT_PERC_LIDAR_FAIL),
            int(ErrorCode.E_EXT_PERC_YOLO_FAIL),
            int(ErrorCode.E_COMM_TF_LOOKUP_FAIL),
        }
        return int(self._last_perception_error) if int(self._last_perception_error) in blocking_codes else 0

    def _match_detections_to_phase_slots(self, detections, phase: int) -> dict[str, object]:
        from fsm_core.constants import SlotStatus

        phase_cols = self._phase_cols(phase)
        y_tol = float(self.config.get("business.match_tolerance.y", 0.15))
        z_tol = float(self.config.get("business.match_tolerance.z", 0.15))
        candidate_slots = [
            slot
            for slot in self._grid_slots
            if int(slot.col_index) in phase_cols and int(slot.status) == int(SlotStatus.OCCUPIED)
        ]
        matches = {}
        for det in detections:
            best_slot = None
            best_score = float("inf")
            det_y = float(det.pose.pose.position.y)
            det_z = float(det.pose.pose.position.z)
            for slot in candidate_slots:
                if slot.slot_id in matches:
                    continue
                exp = slot.expected_pose_robot.pose.position
                dy = abs(det_y - float(exp.y))
                dz = abs(det_z - float(exp.z))
                if dy <= y_tol and dz <= z_tol:
                    score = dy + dz
                    if score < best_score:
                        best_score = score
                        best_slot = slot
            if best_slot is not None:
                matches[best_slot.slot_id] = det
        return matches

    def _build_grid_slots(self, detections, wall_index: int):
        from fsm_core.constants import SlotStatus
        from fsm_msgs.msg import GridSlotState

        expected = self._rows * self._cols
        assignments = assign_grid_indices_by_yz(
            [self._point_from_detection(det) for det in detections],
            rows=self._rows,
            cols=self._cols,
        )
        now = self.get_clock().now().to_msg()
        slots = []
        self._slot_sizes_by_id = {}
        for assignment in assignments[:expected]:
            det = detections[assignment.source_index]
            slot = GridSlotState()
            slot.slot_id = f"wall_{wall_index}_row_{assignment.row}_col_{assignment.col}"
            slot.wall_index = int(wall_index)
            slot.row_index = int(assignment.row)
            slot.col_index = int(assignment.col)
            slot.status = int(SlotStatus.OCCUPIED)
            slot.expected_pose_robot = self._copy_pose_stamped(det.pose)
            slot.latest_pose_robot = self._copy_pose_stamped(det.pose)
            slot.visible = True
            slot.confidence = float(det.confidence)
            slot.last_seen_time = now
            slots.append(slot)
            self._slot_sizes_by_id[slot.slot_id] = self._copy_vector3(det.size)
        return slots

    def _publish_grid_snapshot(self) -> None:
        from fsm_msgs.msg import WallGridSnapshot

        msg = WallGridSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.task_id = self._current_task_id
        msg.wall_index = int(self._current_wall_index)
        msg.rows = int(self._rows)
        msg.cols = int(self._cols)
        msg.wall_frame_pose = self._copy_pose_stamped(self._wall_frame_pose_robot) if self._wall_frame_pose_robot is not None else self._identity_pose("base_link")
        msg.slots = self._grid_slots
        msg.status = 0
        self._grid_snapshot_pub.publish(msg)

    def _build_pair_candidates(self, phase: int, available_slots) -> tuple[list[dict], bool]:
        from fsm_core.constants import GraspMode, Phase

        candidates = []
        single_blocked = False
        phase_slots = sorted(
            [slot for slot in available_slots if int(slot.col_index) in self._phase_cols(phase)],
            key=lambda slot: int(slot.col_index),
        )
        for slot_a, slot_b in combinations(phase_slots, 2):
            left_slot, right_slot = self._assign_candidate_slots_by_y((slot_a, slot_b))
            candidates.append(
                {
                    "phase": int(phase),
                    "mode": GraspMode.DUAL,
                    "left_slot": left_slot,
                    "right_slot": right_slot,
                }
            )

        for slot in phase_slots:
            if not self._allow_single_arm:
                single_blocked = True
                continue
            mode = GraspMode.LEFT_ONLY if phase == int(Phase.LEFT) else GraspMode.RIGHT_ONLY
            candidates.append(
                {
                    "phase": int(phase),
                    "mode": mode,
                    "left_slot": slot if mode == GraspMode.LEFT_ONLY else None,
                    "right_slot": slot if mode == GraspMode.RIGHT_ONLY else None,
                }
            )
        candidates.sort(key=lambda candidate: self._candidate_priority_key(candidate))
        return candidates, single_blocked

    def _assign_candidate_slots_by_y(self, slots):
        left_slot, right_slot = sorted(
            slots,
            key=lambda slot: float(slot.latest_pose_robot.pose.position.y),
            reverse=True,
        )
        return left_slot, right_slot

    def _candidate_priority_key(self, candidate: dict) -> tuple:
        slots = self._candidate_slots(candidate)
        top_row = min(int(slot.row_index) for slot in slots)
        columns = self._candidate_columns(candidate)
        return (top_row, columns, 0 if self._candidate_is_dual(candidate) else 1, int(candidate["phase"]))

    def _candidate_slots(self, candidate: dict) -> list[object]:
        return [slot for slot in (candidate.get("left_slot"), candidate.get("right_slot")) if slot is not None]

    def _candidate_is_dual(self, candidate: dict) -> bool:
        from fsm_core.constants import GraspMode

        return int(candidate.get("mode", -1)) == int(GraspMode.DUAL)

    def _candidate_columns(self, candidate: dict) -> tuple[int, ...]:
        return tuple(sorted({int(slot.col_index) for slot in self._candidate_slots(candidate)}))

    def _candidate_height_safe(self, candidate: dict, column_heights: list[int]) -> tuple[bool, str]:
        after = self._simulate_candidate_heights(column_heights, candidate)
        if after is None:
            return False, f"{self._candidate_label(candidate)}:invalid_height"
        if not self._heights_safe(after):
            return False, f"{self._candidate_label(candidate)}:{column_heights}->{after}"
        return True, "height_safe"

    def _simulate_candidate_heights(self, column_heights: list[int], candidate: dict) -> list[int] | None:
        return self._simulate_columns(column_heights, self._candidate_columns(candidate))

    def _simulate_columns(self, column_heights: list[int], columns: tuple[int, ...]) -> list[int] | None:
        heights = list(column_heights)
        for col in columns:
            if col < 0 or col >= len(heights) or heights[col] <= 0:
                return None
            heights[col] -= 1
        return heights

    def _heights_safe(self, heights: list[int]) -> bool:
        max_delta = int(getattr(self, "_max_adjacent_height_delta", self.config.get("business.max_adjacent_column_height_delta", 1)))
        return all(abs(int(heights[col]) - int(heights[col + 1])) <= max_delta for col in range(len(heights) - 1))

    def _candidate_label(self, candidate: dict) -> str:
        slot_ids = ",".join(slot.slot_id for slot in self._candidate_slots(candidate))
        return f"p{int(candidate.get('phase', -1))}:{slot_ids}"

    def _plan_global_grasp_sequence(self, column_heights: list[int], current_phase: int):
        return plan_global_grasp_sequence(
            column_heights,
            current_phase,
            rows=self._rows,
            left_phase_cols=self._left_phase_cols,
            right_phase_cols=self._right_phase_cols,
            allow_single_arm=self._allow_single_arm,
            max_adjacent_height_delta=self._max_adjacent_height_delta,
        )

    def _candidate_global_score(self, candidate: dict, current_phase: int, column_heights: list[int]) -> tuple | None:
        after = self._simulate_candidate_heights(column_heights, candidate)
        if after is None:
            return None
        remaining_plan = self._plan_global_grasp_sequence(after, int(candidate["phase"]))
        if remaining_plan.total_cost < 0:
            return None
        immediate_cost = self._candidate_action_cost(candidate, current_phase, column_heights)
        total_cost = int(immediate_cost) + int(remaining_plan.total_cost)
        phase_moves = (1 if int(candidate["phase"]) != int(current_phase) else 0) + int(remaining_plan.phase_moves)
        grasp_count = 1 + int(remaining_plan.grasp_count)
        return (
            total_cost,
            phase_moves,
            grasp_count,
            0 if int(candidate["phase"]) == int(current_phase) else 1,
            self._candidate_priority_key(candidate),
        )

    def _candidate_action_cost(self, candidate: dict, current_phase: int, column_heights: list[int]) -> int:
        if int(candidate["phase"]) != int(current_phase):
            return 10
        columns = self._candidate_columns(candidate)
        if len(columns) == 1:
            return 5
        row_a = self._rows - int(column_heights[columns[0]])
        row_b = self._rows - int(column_heights[columns[1]])
        if row_a == row_b and abs(int(columns[0]) - int(columns[1])) == 1:
            return 3
        return 1

    def _global_action_summary(self, action) -> dict:
        return {
            "phase": int(action.phase),
            "columns": list(action.columns),
            "cost": int(action.cost),
            "phase_move": bool(action.phase_move),
            "same_layer_adjacent": bool(action.same_layer_adjacent),
        }

    def _column_heights(self) -> list[int]:
        from fsm_core.constants import SlotStatus

        removed = int(SlotStatus.REMOVED)
        heights = [0 for _ in range(self._cols)]
        for col in range(self._cols):
            occupied_like_rows = [
                int(slot.row_index)
                for slot in self._grid_slots
                if int(slot.col_index) == col and int(slot.status) != removed
            ]
            if occupied_like_rows:
                heights[col] = self._rows - min(occupied_like_rows)
        return heights

    def _top_slot_ids_by_column(self) -> dict[int, str]:
        from fsm_core.constants import SlotStatus

        removed = int(SlotStatus.REMOVED)
        result = {}
        for col in range(self._cols):
            candidates = [
                slot
                for slot in self._grid_slots
                if int(slot.col_index) == col and int(slot.status) != removed
            ]
            if not candidates:
                continue
            top = min(candidates, key=lambda slot: int(slot.row_index))
            result[col] = top.slot_id
        return result

    def _make_pair_from_candidate(self, candidate: dict, phase: int, fixed_place_pose_robot):
        from fsm_msgs.msg import GraspPair

        return self._make_grasp_pair_msg(
            GraspPair,
            phase,
            fixed_place_pose_robot,
            candidate["mode"],
            left_slot=candidate.get("left_slot"),
            right_slot=candidate.get("right_slot"),
        )

    def _pair_candidate_reachable(self, candidate: dict) -> tuple[bool, str]:
        left_slot = candidate.get("left_slot")
        right_slot = candidate.get("right_slot")
        margin = self._reachability_margin()
        if left_slot is not None and not point_in_aabb(self._slot_point(left_slot), self._workspace_aabb("left_arm_workspace"), margin):
            return False, f"{left_slot.slot_id}:left_workspace"
        if right_slot is not None and not point_in_aabb(self._slot_point(right_slot), self._workspace_aabb("right_arm_workspace"), margin):
            return False, f"{right_slot.slot_id}:right_workspace"
        return True, "reachable"

    def _workspace_aabb(self, key: str) -> AABB:
        prefix = f"business.{key}"
        return AABB(
            x_min=float(self.config.get(f"{prefix}.x_min", 0.0)),
            x_max=float(self.config.get(f"{prefix}.x_max", 0.0)),
            y_min=float(self.config.get(f"{prefix}.y_min", 0.0)),
            y_max=float(self.config.get(f"{prefix}.y_max", 0.0)),
            z_min=float(self.config.get(f"{prefix}.z_min", 0.0)),
            z_max=float(self.config.get(f"{prefix}.z_max", 0.0)),
        )

    def _reachability_margin(self) -> float:
        default_margin = max(
            float(self.config.get("business.box_size.length", 0.4)),
            float(self.config.get("business.box_size.width", 0.4)),
            float(self.config.get("business.box_size.height", 0.4)),
        )
        return float(self.config.get("business.reachability_aabb_margin", default_margin))

    def _slot_point(self, slot) -> tuple[float, float, float]:
        position = slot.latest_pose_robot.pose.position
        return (float(position.x), float(position.y), float(position.z))

    def _slot_has_valid_pose(self, slot) -> bool:
        pose = slot.latest_pose_robot
        return bool(pose.header.frame_id) or any(
            abs(value) > 1e-9
            for value in (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
                float(pose.pose.orientation.w),
            )
        )

    def _make_grasp_pair_msg(self, msg_type, phase: int, fixed_place_pose_robot, grasp_mode: int, left_slot=None, right_slot=None):
        pair = msg_type()
        pair.pair_id = f"{self._current_task_id}_w{self._current_wall_index}_p{phase}_{self._pair_sequence:04d}"
        self._pair_sequence += 1
        pair.task_id = self._current_task_id
        pair.wall_index = int(self._current_wall_index)
        pair.phase = int(phase)
        pair.fixed_place_pose_robot = self._normalized_fixed_place_pose(fixed_place_pose_robot)
        pair.grasp_mode = int(grasp_mode)
        if left_slot is not None:
            pair.left_slot_id = left_slot.slot_id
            pair.left_box_pose_robot = self._copy_pose_stamped(left_slot.latest_pose_robot)
            pair.left_box_size = self._copy_vector3(self._slot_sizes_by_id.get(left_slot.slot_id))
        if right_slot is not None:
            pair.right_slot_id = right_slot.slot_id
            pair.right_box_pose_robot = self._copy_pose_stamped(right_slot.latest_pose_robot)
            pair.right_box_size = self._copy_vector3(self._slot_sizes_by_id.get(right_slot.slot_id))
        return pair

    def _mark_pair_removed(self, pair_msg, pair_result=None) -> int:
        from fsm_core.constants import SlotStatus

        active_slot_ids = self._pair_success_slot_ids(pair_msg, pair_result)
        removed = 0
        for slot in self._grid_slots:
            if slot.slot_id in active_slot_ids and int(slot.status) == int(SlotStatus.OCCUPIED):
                slot.status = int(SlotStatus.REMOVED)
                slot.visible = False
                removed += 1
        return removed

    def _pair_active_slot_ids(self, pair_msg) -> list[str]:
        from fsm_core.constants import GraspMode

        active_slot_ids = []
        if pair_msg.grasp_mode in (int(GraspMode.DUAL), int(GraspMode.LEFT_ONLY)) and pair_msg.left_slot_id:
            active_slot_ids.append(pair_msg.left_slot_id)
        if pair_msg.grasp_mode in (int(GraspMode.DUAL), int(GraspMode.RIGHT_ONLY)) and pair_msg.right_slot_id:
            active_slot_ids.append(pair_msg.right_slot_id)
        return active_slot_ids

    def _pair_success_slot_ids(self, pair_msg, pair_result=None) -> list[str]:
        from fsm_core.constants import ResultCode

        if pair_result is None:
            return self._pair_active_slot_ids(pair_msg)
        result_code = int(getattr(pair_result, "result_code", ResultCode.FAILED_BOTH))
        if result_code == int(ResultCode.SUCCESS_BOTH):
            return self._pair_active_slot_ids(pair_msg)
        if result_code == int(ResultCode.SUCCESS_LEFT_ONLY):
            return [pair_msg.left_slot_id] if pair_msg.left_slot_id else []
        if result_code == int(ResultCode.SUCCESS_RIGHT_ONLY):
            return [pair_msg.right_slot_id] if pair_msg.right_slot_id else []
        return []

    def _record_pair_failure(self, pair_msg, error_code: int, pair_result=None) -> int:
        active_slot_ids = set(self._pair_active_slot_ids(pair_msg)) - set(self._pair_success_slot_ids(pair_msg, pair_result))
        if not active_slot_ids:
            return int(self.config.get("business.max_retry_per_slot", 3))
        max_retry_count = 0
        for slot in self._grid_slots:
            if slot.slot_id not in active_slot_ids:
                continue
            slot.retry_count = min(int(slot.retry_count) + 1, 255)
            slot.last_error_code = int(error_code)
            max_retry_count = max(max_retry_count, int(slot.retry_count))
        return max_retry_count

    def _should_retry_pair_grasp(self, recovery: dict, retry_count: int, max_retry: int) -> bool:
        if self._estop:
            return False
        action = str(recovery.get("recovery_action", ""))
        if action not in {"RETRY_CURRENT_STATE", "SWITCH_TARGET", "REPLAN"}:
            return False
        return int(retry_count) < int(max_retry)

    def _phase_has_occupied_slots(self, phase: int) -> bool:
        from fsm_core.constants import SlotStatus

        phase_cols = self._phase_cols(phase)
        return any(int(slot.status) == int(SlotStatus.OCCUPIED) and int(slot.col_index) in phase_cols for slot in self._grid_slots)

    def _wall_has_occupied_slots(self) -> bool:
        from fsm_core.constants import SlotStatus

        return any(int(slot.status) == int(SlotStatus.OCCUPIED) for slot in self._grid_slots)

    def _phase_progress_percent(self, phase: int) -> int:
        from fsm_core.constants import SlotStatus

        phase_cols = self._phase_cols(phase)
        phase_slots = [slot for slot in self._grid_slots if int(slot.col_index) in phase_cols]
        if not phase_slots:
            return 0
        removed = sum(1 for slot in phase_slots if int(slot.status) == int(SlotStatus.REMOVED))
        return int(round(100.0 * removed / len(phase_slots)))

    def _wall_progress_percent(self) -> int:
        from fsm_core.constants import SlotStatus

        if not self._grid_slots:
            return 0
        removed = sum(1 for slot in self._grid_slots if int(slot.status) == int(SlotStatus.REMOVED))
        return int(round(100.0 * removed / len(self._grid_slots)))

    def _phase_state_name(self, phase: int) -> str:
        return "LEFT_PHASE" if int(phase) == 0 else "RIGHT_PHASE"

    def _phase_cols(self, phase: int) -> list[int]:
        return self._left_phase_cols if int(phase) == 0 else self._right_phase_cols

    def _pose_from_business_prefix(self, prefix: str, frame_id: str):
        pose = self._identity_pose(frame_id)
        pose.pose.position.x = float(self.config.get(f"{prefix}.x", 0.0))
        pose.pose.position.y = float(self.config.get(f"{prefix}.y", 0.0))
        pose.pose.position.z = float(self.config.get(f"{prefix}.z", 0.0))
        yaw = float(self.config.get(f"{prefix}.yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _identity_pose(self, frame_id: str):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.orientation.w = 1.0
        return pose

    def _normalized_fixed_place_pose(self, pose):
        out = self._copy_pose_stamped(pose)
        if not out.header.frame_id:
            out.header.frame_id = "base_link"
        if out.pose.orientation.w == 0.0 and out.pose.orientation.x == 0.0 and out.pose.orientation.y == 0.0 and out.pose.orientation.z == 0.0:
            out.pose.orientation.w = 1.0
        return out

    def _copy_pose_stamped(self, pose):
        from geometry_msgs.msg import PoseStamped

        out = PoseStamped()
        out.header.stamp.sec = int(pose.header.stamp.sec)
        out.header.stamp.nanosec = int(pose.header.stamp.nanosec)
        out.header.frame_id = pose.header.frame_id or "base_link"
        out.pose.position.x = float(pose.pose.position.x)
        out.pose.position.y = float(pose.pose.position.y)
        out.pose.position.z = float(pose.pose.position.z)
        out.pose.orientation.x = float(pose.pose.orientation.x)
        out.pose.orientation.y = float(pose.pose.orientation.y)
        out.pose.orientation.z = float(pose.pose.orientation.z)
        out.pose.orientation.w = float(pose.pose.orientation.w)
        return out

    def _copy_vector3(self, value=None):
        from geometry_msgs.msg import Vector3

        out = Vector3()
        if value is None:
            out.x = float(self.config.get("business.box_size.length", 0.4))
            out.y = float(self.config.get("business.box_size.width", 0.4))
            out.z = float(self.config.get("business.box_size.height", 0.4))
            return out
        out.x = float(value.x)
        out.y = float(value.y)
        out.z = float(value.z)
        return out

    async def _sleep(self, duration_sec: float) -> None:
        from rclpy.task import Future

        future = Future()

        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(float(duration_sec), wake)
        await future

    def _cancel_goal_handle(self, goal_handle) -> None:
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - ROS2 防御兜底
            self.get_logger().warning(f"failed to cancel child goal: {exc}")

    def _cancel_active_children(self) -> None:
        if self._active_grasp_goal_handle is not None:
            self._cancel_goal_handle(self._active_grasp_goal_handle)
        if self._active_nav_goal_handle is not None:
            self._cancel_goal_handle(self._active_nav_goal_handle)


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("wall_destacking_strategy_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_topic_name
    from fsm_msgs.action import RunWallDestacking
    from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth, SafetyStatus

    class WallDestackingStrategyNode(WallDestackingStrategyNodeMixin, SkeletonNodeMixin, Node):
        def __init__(self):
            super().__init__("wall_destacking_strategy_node")
            self._action_group = ReentrantCallbackGroup()
            self._subscription_group = ReentrantCallbackGroup()
            self.init_fsm_node_base(ready_state="IDLE", heartbeat_fsm_name="WallDestackingFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_wall_destacking_state", "/fsm/wall_destacking_state", "WallDestackingFSM")
            self._init_strategy_runtime()
            self._detections_sub = self.create_subscription(
                BoxDetectionArray,
                get_topic_name(self, "perception_detections", "/perception/box_detections"),
                self.on_detections,
                10,
                callback_group=self._subscription_group,
            )
            self._perception_health_sub = self.create_subscription(
                PerceptionHealth,
                get_topic_name(self, "perception_health", "/perception/health"),
                self.on_perception_health,
                10,
                callback_group=self._subscription_group,
            )
            self._safety_sub = self.create_subscription(
                SafetyStatus,
                get_topic_name(self, "safety_status", "/safety/status"),
                self.on_safety_status,
                10,
                callback_group=self._subscription_group,
            )
            self._action_server = ActionServer(
                self,
                RunWallDestacking,
                get_action_name(self, "run_wall_destacking", "/run_wall_destacking"),
                self.execute_wall_destacking,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
                callback_group=self._action_group,
            )
            self.get_logger().info("wall_destacking_strategy_node skeleton ready; minimal happy path enabled")

    rclpy.init(args=args)
    node = WallDestackingStrategyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
