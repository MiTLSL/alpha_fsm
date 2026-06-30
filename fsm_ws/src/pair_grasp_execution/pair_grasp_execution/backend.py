from __future__ import annotations

import time

from .errors import GraspCancelled, GraspFailure
from .geometry import make_pose_stamped
from .moveit_goal import MoveItGoalBuilderMixin
from .planning_scene import PlanningSceneMixin
from .stages import FAKE_FAILURES, STAGES
from .target_builder import GraspTargetBuilderMixin


class PairGraspExecutionNodeMixin(GraspTargetBuilderMixin, MoveItGoalBuilderMixin, PlanningSceneMixin):
    STAGES = STAGES
    FAKE_FAILURES = FAKE_FAILURES

    def init_grasp_backend(self):
        from fsm_core.ros2_helpers import get_action_name, get_service_name
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._io_callback_group = ReentrantCallbackGroup()
        self._backend_mode = str(self.config.get("business.pair_grasp_execution.backend_mode", "dry_run"))
        self._fake_failure_mode = str(self.config.get("business.pair_grasp_execution.fake_failure_mode", "NONE"))
        self._stage_delay_sec = float(self.config.get("business.pair_grasp_execution.stage_delay_sec", 0.03))
        self._moveit_wait_sec = float(self.config.get("business.pair_grasp_execution.moveit_action_wait_sec", 2.0))
        self._moveit_group = str(self.config.get("business.pair_grasp_execution.moveit_planning_group", "dual_v5_arm_with_base"))
        self._left_tip = str(self.config.get("business.pair_grasp_execution.left_tip_link", "left_v5_tool0"))
        self._right_tip = str(self.config.get("business.pair_grasp_execution.right_tip_link", "right_v5_tool0"))
        self._moveit_allowed_planning_time_sec = float(
            self.config.get("business.pair_grasp_execution.moveit_allowed_planning_time_sec", 5.0)
        )
        self._moveit_num_planning_attempts = int(self.config.get("business.pair_grasp_execution.moveit_num_planning_attempts", 10))
        self._moveit_velocity_scaling = float(self.config.get("business.pair_grasp_execution.moveit_velocity_scaling", 0.3))
        self._moveit_acceleration_scaling = float(self.config.get("business.pair_grasp_execution.moveit_acceleration_scaling", 0.2))
        self._position_tolerance = float(self.config.get("business.pair_grasp_execution.moveit_position_tolerance", 0.015))
        self._orientation_tolerance = float(self.config.get("business.pair_grasp_execution.moveit_orientation_tolerance", 0.08))
        self._input_pose_represents_box_center = bool(
            self.config.get("business.pair_grasp_execution.input_pose_represents_box_center", True)
        )
        self._contact_standoff_x = float(self.config.get("business.pair_grasp_execution.contact_standoff_x", 0.0))
        self._pregrasp_offset_x = float(self.config.get("business.pair_grasp_execution.pregrasp_offset_x", 0.10))
        self._extract_offset_x = float(self.config.get("business.pair_grasp_execution.extract_offset_x", 0.10))
        self._retreat_offset_x = float(self.config.get("business.pair_grasp_execution.retreat_offset_x", 0.10))
        self._place_y_separation = float(self.config.get("business.pair_grasp_execution.place_y_separation", 0.40))
        self._attach_box_to_planning_scene = bool(
            self.config.get("business.pair_grasp_execution.attach_box_to_planning_scene", True)
        )
        self._manage_world_collision_objects = bool(
            self.config.get("business.pair_grasp_execution.collision_scene.manage_world_objects", True)
        )
        self._remove_target_box_world_objects = bool(
            self.config.get("business.pair_grasp_execution.collision_scene.remove_target_box_objects", False)
        )
        self._self_collision_check_current_state = bool(
            self.config.get("business.pair_grasp_execution.self_collision.check_current_state", True)
        )
        self._self_collision_require_dual_arm_group = bool(
            self.config.get("business.pair_grasp_execution.self_collision.require_dual_arm_group", True)
        )
        self._self_collision_required_group_name = str(
            self.config.get("business.pair_grasp_execution.self_collision.required_group_name", "dual_v5_arm_with_base")
        )
        self._joint_state_max_age_sec = float(
            self.config.get("business.pair_grasp_execution.self_collision.joint_state_max_age_sec", 1.0)
        )
        self._state_validity_timeout_sec = float(
            self.config.get("business.pair_grasp_execution.self_collision.state_validity_timeout_sec", 1.0)
        )
        self._attached_box_weight_kg = float(self.config.get("business.pair_grasp_execution.attached_box_weight_kg", 2.0))
        self._attached_box_touch_links = list(self.config.get("business.pair_grasp_execution.attached_box_touch_links", []))
        self._planning_frame = str(self.config.get("interfaces.frames.base_link", "base_link"))
        self._skip_vacuum_check = bool(self.config.get("business.pair_grasp_execution.skip_vacuum_check", True))
        self._hold_on_estop = bool(self.config.get("business.vacuum.hold_on_estop", True))
        self._release_on_cancel = bool(self.config.get("business.vacuum.release_on_cancel", False))
        self._attach_threshold_kpa = float(self.config.get("business.vacuum.attach_threshold_kpa", -50.0))
        self._moveit_action_name = get_action_name(self, "moveit_move_group", "/move_action")
        self._apply_planning_scene_service = get_service_name(self, "moveit_apply_planning_scene", "/apply_planning_scene")
        self._check_state_validity_service = get_service_name(self, "moveit_check_state_validity", "/check_state_validity")
        self._moveit_client = ActionClient(self, MoveGroup, self._moveit_action_name, callback_group=self._io_callback_group)
        self._planning_scene_client = self.create_client(
            ApplyPlanningScene,
            self._apply_planning_scene_service,
            callback_group=self._io_callback_group,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            self._check_state_validity_service,
            callback_group=self._io_callback_group,
        )
        self._active_moveit_goal_handle = None
        self._attached_object_ids = set()
        self._world_collision_object_ids = set()
        self._last_joint_state = None
        self._last_joint_state_received_monotonic = None

    def handle_goal(self, goal_request):
        from rclpy.action import GoalResponse

        if self._fake_failure_mode == "GOAL_REJECTED":
            return GoalResponse.REJECT
        if self._backend_mode not in ("dry_run", "fake_real", "real"):
            self.get_logger().warning(f"unsupported grasp backend_mode={self._backend_mode}")
            return GoalResponse.REJECT
        try:
            self._validate_grasp_pair(goal_request.grasp_pair)
        except GraspFailure as exc:
            self.get_logger().warning(f"accepting invalid pair so action can report structured error: {exc}")
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

        self._ready_state = "CANCEL_REQUESTED"
        self.publish_state_heartbeat()
        self._cancel_active_moveit_goal()
        return CancelResponse.ACCEPT

    def on_estop(self, msg):
        self._estop = bool(msg.data)
        if self._estop:
            self._ready_state = "ESTOP"
            self.publish_state_heartbeat()
            self._cancel_active_moveit_goal()

    def on_pressure_raw(self, msg):
        if len(msg.data) >= 2:
            self._last_pressure_raw = [float(msg.data[0]), float(msg.data[1])]

    def on_joint_states(self, msg):
        self._last_joint_state = msg
        self._last_joint_state_received_monotonic = time.monotonic()

    def publish_pressure_forward(self):
        from std_msgs.msg import Float32MultiArray

        msg = Float32MultiArray()
        msg.data = list(self._last_pressure_raw)
        self._vacuum_pressure_pub.publish(msg)

    async def execute_pair_grasp_goal(self, goal_handle):
        from fsm_core.error_code import ErrorCode

        start = time.monotonic()
        request = goal_handle.request
        pair = request.grasp_pair

        try:
            self._validate_grasp_pair(pair)
        except GraspFailure as exc:
            goal_handle.abort()
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="FAILED_BOTH",
                error_code=exc.error_code,
                failed_stage=exc.failed_stage,
                start_time=start,
            )

        vacuum_enabled = bool(not request.dry_run and not self._skip_vacuum_check)
        if vacuum_enabled:
            self._publish_vacuum_command(*self._active_arms(pair))

        try:
            for index, (state, stage) in enumerate(self.STAGES):
                self._publish_feedback(goal_handle, state, stage, index)
                await self._sleep(self._stage_delay_sec)

                if goal_handle.is_cancel_requested:
                    if self._release_on_cancel:
                        self._publish_vacuum_command(False, False)
                    await self._clear_attached_boxes_best_effort()
                    await self._cleanup_pair_planning_scene(pair)
                    goal_handle.canceled()
                    return self._make_result(
                        goal_handle,
                        success=False,
                        result_code_name="CANCELLED",
                        error_code=0,
                        failed_stage="CANCEL",
                        start_time=start,
                    )
                if self._estop:
                    if not self._hold_on_estop:
                        self._publish_vacuum_command(False, False)
                        await self._clear_attached_boxes_best_effort()
                    await self._cleanup_pair_planning_scene(pair)
                    goal_handle.abort()
                    return self._make_result(
                        goal_handle,
                        success=False,
                        result_code_name="ESTOP",
                        error_code=int(ErrorCode.E_SAFETY_ESTOP_HW),
                        failed_stage="ESTOP",
                        start_time=start,
                    )

                await self._backend_step(goal_handle, state, stage)

        except GraspCancelled:
            if self._release_on_cancel:
                self._publish_vacuum_command(False, False)
            await self._clear_attached_boxes_best_effort()
            await self._cleanup_pair_planning_scene(pair)
            goal_handle.canceled()
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="CANCELLED",
                error_code=0,
                failed_stage="CANCEL",
                start_time=start,
            )
        except GraspFailure as exc:
            if vacuum_enabled:
                self._publish_vacuum_command(False, False)
            if not self._hold_on_estop or exc.failed_stage != "ESTOP":
                await self._clear_attached_boxes_best_effort()
            await self._cleanup_pair_planning_scene(pair)
            goal_handle.abort()
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="FAILED_BOTH",
                error_code=exc.error_code,
                failed_stage=exc.failed_stage,
                start_time=start,
            )

        if vacuum_enabled:
            self._publish_vacuum_command(False, False)
        await self._cleanup_pair_planning_scene(pair)
        goal_handle.succeed()
        self._ready_state = "REPORT"
        self.publish_state_heartbeat()
        return self._make_result(
            goal_handle,
            success=True,
            result_code_name=self._success_code_name(pair),
            error_code=0,
            failed_stage="",
            start_time=start,
        )

    async def _backend_step(self, goal_handle, state: str, stage: str) -> None:
        from fsm_core.error_code import ErrorCode

        if self._backend_mode == "dry_run":
            return

        if self._backend_mode == "fake_real":
            failure = self.FAKE_FAILURES.get(self._fake_failure_mode)
            if failure and failure[0] == state:
                raise GraspFailure(failure[2], failure[1], self._fake_failure_mode)
            return

        if self._backend_mode == "real":
            if stage == "PLAN":
                ok, error_code, reason = await self._call_moveit_plan(goal_handle, state, plan_only=True)
                if not ok:
                    raise GraspFailure(error_code, stage, reason)
            elif stage == "MOVE" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._call_moveit_plan(goal_handle, state, plan_only=False)
                if not ok:
                    raise GraspFailure(error_code, stage, reason)
            elif state == "ATTACH_BOX_MODEL" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._apply_attached_boxes(goal_handle.request.grasp_pair, attach=True)
                if not ok:
                    raise GraspFailure(error_code, stage, reason)
            elif state == "RELEASE_BOX" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._apply_attached_boxes(goal_handle.request.grasp_pair, attach=False)
                if not ok:
                    raise GraspFailure(error_code, stage, reason)

    async def _call_moveit_plan(self, goal_handle, state: str, plan_only: bool) -> tuple[bool, int, str]:
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.action import MoveGroup

        if not self._moveit_client.wait_for_server(timeout_sec=max(self._moveit_wait_sec, 0.1)):
            self.get_logger().warning("MoveIt move_action server unavailable")
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt move_action server unavailable"

        targets = self._stage_targets(goal_handle.request.grasp_pair, state)
        if not targets:
            return False, int(ErrorCode.E_PLAN_PREGRASP_INVALID), f"no MoveIt target for stage {state}"

        ok, error_code, reason = await self._prepare_planning_scene_for_stage(goal_handle.request.grasp_pair, state)
        if not ok:
            return False, int(error_code), reason
        ok, error_code, reason = await self._check_robot_state_valid_for_planning()
        if not ok:
            return False, int(error_code), reason

        move_goal = MoveGroup.Goal()
        self._fill_motion_plan_request(move_goal.request, targets)
        move_goal.request.group_name = self._moveit_group
        move_goal.planning_options.plan_only = bool(plan_only or goal_handle.request.dry_run)
        move_goal.planning_options.planning_scene_diff.is_diff = True
        move_goal.planning_options.replan = bool(not plan_only)
        move_goal.planning_options.replan_attempts = 1
        move_goal.planning_options.replan_delay = 0.2
        send_future = self._moveit_client.send_goal_async(move_goal)
        sent, move_goal_handle = await self._wait_future(send_future, self._moveit_wait_sec, "MoveIt send_goal")
        if not sent or move_goal_handle is None or not move_goal_handle.accepted:
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt rejected or did not accept goal"
        self._active_moveit_goal_handle = move_goal_handle
        result_future = move_goal_handle.get_result_async()
        try:
            done, move_result = await self._wait_moveit_result(goal_handle, result_future, max(goal_handle.request.timeout_sec or 5.0, 1.0))
        finally:
            self._active_moveit_goal_handle = None
        if not done or move_result is None:
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt result timeout"
        if int(move_result.status) == int(GoalStatus.STATUS_CANCELED):
            raise GraspCancelled()
        if int(move_result.status) != int(GoalStatus.STATUS_SUCCEEDED):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), f"MoveIt action status {int(move_result.status)}"
        moveit_error = int(getattr(getattr(move_result.result, "error_code", None), "val", 99999))
        if moveit_error != 1:
            return False, self._map_moveit_error(moveit_error, executing=not plan_only), f"MoveIt error_code={moveit_error}"
        return True, 0, ""

    def _map_moveit_error(self, moveit_error: int, executing: bool) -> int:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.msg import MoveItErrorCodes

        if executing and int(moveit_error) in self._moveit_error_values(MoveItErrorCodes, "CONTROL_FAILED"):
            return int(ErrorCode.E_MOT_EXEC_FAIL)
        if int(moveit_error) in self._moveit_error_values(MoveItErrorCodes, "NO_IK_SOLUTION"):
            return int(ErrorCode.E_PLAN_IK_FAIL)
        if int(moveit_error) in self._moveit_error_values(
            MoveItErrorCodes,
            "START_STATE_IN_COLLISION",
            "GOAL_IN_COLLISION",
            "COLLISION_CHECKING_UNAVAILABLE",
            "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        ):
            return int(ErrorCode.E_PLAN_COLLISION_DETECTED)
        if executing:
            return int(ErrorCode.E_MOT_EXEC_FAIL)
        return int(ErrorCode.E_PLAN_TRAJ_FAIL)

    @staticmethod
    def _moveit_error_values(error_codes, *names: str) -> set[int]:
        values = set()
        for name in names:
            if hasattr(error_codes, name):
                values.add(int(getattr(error_codes, name)))
        return values

    async def _wait_moveit_result(self, goal_handle, future, timeout_sec: float):
        from fsm_core.error_code import ErrorCode

        deadline = time.monotonic() + max(float(timeout_sec), 0.01)
        while not future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_moveit_goal()
                raise GraspCancelled()
            if self._estop:
                self._cancel_active_moveit_goal()
                raise GraspFailure(int(ErrorCode.E_SAFETY_ESTOP_HW), "ESTOP", "estop during MoveIt motion")
            if time.monotonic() >= deadline:
                self._cancel_active_moveit_goal()
                return False, None
            await self._sleep(0.02)
        return True, future.result()

    def _cancel_active_moveit_goal(self) -> None:
        moveit_goal = getattr(self, "_active_moveit_goal_handle", None)
        if moveit_goal is None:
            return
        try:
            moveit_goal.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - defensive DDS boundary
            self.get_logger().warning(f"failed to cancel MoveIt goal: {exc}")

    def _validate_grasp_pair(self, pair) -> None:
        from fsm_core.error_code import ErrorCode

        if not str(pair.pair_id):
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "pair_id is required")
        left_active, right_active = self._active_arms(pair)
        if not left_active and not right_active:
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "no active arm in grasp_mode")
        if left_active and not str(pair.left_slot_id):
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "left_slot_id is required")
        if right_active and not str(pair.right_slot_id):
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "right_slot_id is required")
        if left_active and not self._pose_has_frame(pair.left_box_pose_robot):
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "left_box_pose_robot frame_id is required")
        if right_active and not self._pose_has_frame(pair.right_box_pose_robot):
            raise GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "right_box_pose_robot frame_id is required")

    @staticmethod
    def _pose_has_frame(pose_stamped) -> bool:
        return bool(str(getattr(getattr(pose_stamped, "header", None), "frame_id", "")))

    @staticmethod
    def _active_arms(pair) -> tuple[bool, bool]:
        left_active = pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_LEFT_ONLY)
        right_active = pair.grasp_mode in (pair.MODE_DUAL, pair.MODE_RIGHT_ONLY)
        return bool(left_active), bool(right_active)

    def _success_code_name(self, pair) -> str:
        left_active, right_active = self._active_arms(pair)
        if left_active and right_active:
            return "SUCCESS_BOTH"
        if left_active:
            return "SUCCESS_LEFT_ONLY"
        return "SUCCESS_RIGHT_ONLY"

    def _publish_feedback(self, goal_handle, state: str, stage: str, index: int) -> None:
        from fsm_msgs.action import ExecutePairGrasp

        self._ready_state = state
        self.publish_state_heartbeat()
        feedback = ExecutePairGrasp.Feedback()
        feedback.current_state = state
        feedback.current_stage = stage
        feedback.progress_percent = float(index * 100 / max(len(self.STAGES) - 1, 1))
        feedback.vacuum_left_kpa = float(self._last_pressure_raw[0])
        feedback.vacuum_right_kpa = float(self._last_pressure_raw[1])
        goal_handle.publish_feedback(feedback)

    def _publish_vacuum_command(self, left_on: bool, right_on: bool) -> None:
        from fsm_msgs.msg import VacuumCommand

        msg = VacuumCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left_on = bool(left_on)
        msg.right_on = bool(right_on)
        msg.command_source = msg.SOURCE_PAIR_GRASP
        self._vacuum_cmd_pub.publish(msg)

    def _make_result(self, goal_handle, success: bool, result_code_name: str, error_code: int, failed_stage: str, start_time: float):
        from fsm_msgs.action import ExecutePairGrasp

        result = ExecutePairGrasp.Result()
        result.success = bool(success)
        result.result.pair_id = goal_handle.request.grasp_pair.pair_id
        result.result.result_code = getattr(result.result, result_code_name)
        left_active, right_active = self._active_arms(goal_handle.request.grasp_pair)
        result.result.left_result = 0 if success and left_active else (1 if left_active else 2)
        result.result.right_result = 0 if success and right_active else (1 if right_active else 2)
        result.result.failed_stage = str(failed_stage)
        result.result.error_code = int(error_code)
        result.result.vacuum_left_kpa = float(self._last_pressure_raw[0])
        result.result.vacuum_right_kpa = float(self._last_pressure_raw[1])
        result.result.execution_time_sec = float(time.monotonic() - start_time)
        result.result.final_robot_pose = make_pose_stamped("base_link")
        return result

    async def _wait_future(self, future, timeout_sec: float, label: str):
        deadline = time.monotonic() + max(float(timeout_sec), 0.01)
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"{label} timeout")
                return False, None
            await self._sleep(0.02)
        return True, future.result()

    async def _sleep(self, duration_sec: float) -> None:
        from rclpy.task import Future

        future = Future()

        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(float(duration_sec), wake)
        await future
