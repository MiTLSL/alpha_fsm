from __future__ import annotations

import json
import math
import time


class _StrategyCancelled(Exception):
    pass


class _StrategyFailure(Exception):
    def __init__(self, error_code: int, reason: str):
        super().__init__(reason)
        self.error_code = int(error_code)
        self.reason = reason


class WallDestackingStrategyNodeMixin:
    def _init_strategy_runtime(self) -> None:
        from fsm_core.ros2_helpers import get_action_name, get_topic_name, make_qos_profile
        from fsm_msgs.action import ExecutePairGrasp, NavigateToPose
        from fsm_msgs.msg import GraspPair, WallGridSnapshot
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
        self._last_detections_msg = None
        self._last_detections_monotonic = 0.0
        self._last_detection_count = 0
        self._last_perception_error = 0
        self._estop = False
        self._active_nav_goal_handle = None
        self._active_grasp_goal_handle = None
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

    def on_config_reloaded(self) -> None:
        self._refresh_strategy_config()

    def _refresh_strategy_config(self) -> None:
        self._rows = int(self.config.get("business.grid_shape.rows", 5))
        self._cols = int(self.config.get("business.grid_shape.cols", 5))
        self._left_phase_cols = [int(col) for col in self.config.get("business.left_phase_cols", [0, 1, 2])]
        self._right_phase_cols = [int(col) for col in self.config.get("business.right_phase_cols", [3, 4])]
        self._allow_single_arm = bool(self.config.get("business.allow_single_arm_grasp", True))

    def on_detections(self, msg):
        self._last_detections_msg = msg
        self._last_detections_monotonic = time.monotonic()
        self._last_detection_count = len(msg.detections)

    def on_perception_health(self, msg):
        self._last_perception_error = int(msg.error_code)

    def on_safety_status(self, msg):
        self._estop = bool(msg.estop)

    def handle_goal(self, goal_request):
        del goal_request
        from rclpy.action import GoalResponse

        if self._strategy_goal_active or self._estop:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

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
            detections = await self._wait_for_grid_detections(goal_handle)
            self._grid_slots = self._build_grid_slots(detections, self._current_wall_index)
            self._publish_grid_snapshot()

            for phase, phase_state in ((0, "LEFT_PHASE"), (1, "RIGHT_PHASE")):
                self._current_phase = phase
                self.ctx.current_phase = phase_state
                self._set_wall_state(phase_state)
                self._publish_feedback(goal_handle, phase_state, self._phase_progress_percent(phase))
                while self._phase_has_occupied_slots(phase):
                    self._check_cancel_or_estop(goal_handle)
                    pair_msg = self._select_next_pair(phase, goal_handle.request.fixed_place_pose_robot)
                    if pair_msg is None:
                        raise _StrategyFailure(int(ErrorCode.E_PAIR_NO_CANDIDATE), f"no pair candidate in {phase_state}")
                    self.ctx.current_grasp_pair = pair_msg
                    self._set_wall_state("DECIDE_NEXT_PAIR")
                    self._grasp_pair_pub.publish(pair_msg)
                    self._publish_feedback(goal_handle, "DECIDE_NEXT_PAIR", self._phase_progress_percent(phase))

                    self._set_wall_state("WAIT_PAIR_GRASP_RESULT")
                    self._publish_feedback(goal_handle, "WAIT_PAIR_GRASP_RESULT", self._phase_progress_percent(phase))
                    grasp_result = await self._execute_pair_grasp(goal_handle, pair_msg, dry_run)
                    if not grasp_result.success:
                        code = int(grasp_result.result.error_code or ErrorCode.E_GRASP_UNKNOWN)
                        reason = grasp_result.result.failed_stage or "pair grasp failed"
                        raise _StrategyFailure(code, reason)
                    self._total_boxes_picked += self._mark_pair_removed(pair_msg)
                    self._set_wall_state("UPDATE_GRID_AFTER_GRASP")
                    self._publish_grid_snapshot()
                    self._publish_feedback(goal_handle, "UPDATE_GRID_AFTER_GRASP", self._phase_progress_percent(phase))

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
            self._set_wall_state("FAILED")
            goal_handle.abort()
            result = RunWallDestacking.Result()
            result.success = False
            result.walls_completed = 0
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = int(exc.error_code)
            result.failure_reason = exc.reason
            return result
        except Exception as exc:  # pragma: no cover - ROS2 运行期兜底
            self._last_error_code = int(ErrorCode.E_WALL_UNKNOWN)
            self._set_wall_state("FAILED")
            goal_handle.abort()
            result = RunWallDestacking.Result()
            result.success = False
            result.walls_completed = 0
            result.total_boxes_picked = int(self._total_boxes_picked)
            result.error_code = int(ErrorCode.E_WALL_UNKNOWN)
            result.failure_reason = str(exc)
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
        from action_msgs.msg import GoalStatus
        from fsm_core.constants import GoalType, Phase
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import NavigateToPose

        await self._wait_for_action_server(
            self._nav_client,
            goal_handle,
            float(self.config.get("fsm.action_send_goal_timeout_sec", 2.0)),
            "navigate_to_pose",
        )
        goal = NavigateToPose.Goal()
        goal.goal_type = GoalType.OBSERVATION
        goal.target_pose = self._pose_from_business_prefix("business.observation_pose", "map")
        goal.wall_frame_pose = self._identity_pose("map")
        goal.phase = int(Phase.LEFT)
        goal.desired_distance_to_wall = float(self.config.get("business.desired_distance_to_wall", 0.6))
        goal.desired_yaw_to_wall = float(self.config.get("business.desired_yaw_to_wall", 0.0))
        goal.desired_lateral_offset = 0.0
        goal.require_fine_alignment = False
        goal.timeout_sec = float(self.config.get("fsm.state_timeouts.WallDestackingFSM_NAVIGATE_TO_OBSERVATION_POSE", 60.0))
        send_future = self._nav_client.send_goal_async(goal)
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
            float(self.config.get("fsm.state_timeouts.WallDestackingFSM_NAVIGATE_TO_OBSERVATION_POSE", 60.0)) + 2.0,
            child_goal_handle=nav_goal_handle,
        )
        self._active_nav_goal_handle = None
        if result_wrapper.status == GoalStatus.STATUS_CANCELED:
            raise _StrategyCancelled()
        result = result_wrapper.result
        if not result.success:
            raise _StrategyFailure(int(result.error_code or ErrorCode.E_NAV_UNKNOWN), result.failure_reason or "navigation failed")
        return result

    async def _execute_pair_grasp(self, goal_handle, pair_msg, dry_run: bool):
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import ExecutePairGrasp

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
        send_future = self._grasp_client.send_goal_async(goal)
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
        for det in msg.detections:
            frame_id = det.pose.header.frame_id or det.header.frame_id or msg.header.frame_id
            if frame_id != "base_link":
                continue
            if not det.pose_valid:
                continue
            if float(det.confidence) < confidence_min:
                continue
            if expected_label and det.class_label and det.class_label != expected_label:
                continue
            valid.append(det)
        return valid

    def _build_grid_slots(self, detections, wall_index: int):
        from fsm_core.constants import SlotStatus
        from fsm_msgs.msg import GridSlotState

        expected = self._rows * self._cols
        ordered_by_z = sorted(detections, key=lambda det: float(det.pose.pose.position.z), reverse=True)[:expected]
        now = self.get_clock().now().to_msg()
        slots = []
        self._slot_sizes_by_id = {}
        for row in range(self._rows):
            row_detections = ordered_by_z[row * self._cols : (row + 1) * self._cols]
            row_detections = sorted(row_detections, key=lambda det: float(det.pose.pose.position.y), reverse=True)
            for col, det in enumerate(row_detections):
                slot = GridSlotState()
                slot.slot_id = f"wall_{wall_index}_row_{row}_col_{col}"
                slot.wall_index = int(wall_index)
                slot.row_index = int(row)
                slot.col_index = int(col)
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
        msg.wall_frame_pose = self._identity_pose("base_link")
        msg.slots = self._grid_slots
        msg.status = 0
        self._grid_snapshot_pub.publish(msg)

    def _select_next_pair(self, phase: int, fixed_place_pose_robot):
        from fsm_core.constants import GraspMode, Phase, SlotStatus
        from fsm_msgs.msg import GraspPair

        phase_cols = self._phase_cols(phase)
        for row in range(self._rows):
            row_slots = [
                slot
                for slot in self._grid_slots
                if int(slot.row_index) == row
                and int(slot.col_index) in phase_cols
                and int(slot.status) == int(SlotStatus.OCCUPIED)
            ]
            row_slots.sort(key=lambda slot: int(slot.col_index))
            if len(row_slots) >= 2:
                left_slot, right_slot = sorted(
                    row_slots[:2],
                    key=lambda slot: float(slot.latest_pose_robot.pose.position.y),
                    reverse=True,
                )
                return self._make_grasp_pair_msg(
                    GraspPair,
                    phase,
                    fixed_place_pose_robot,
                    GraspMode.DUAL,
                    left_slot=left_slot,
                    right_slot=right_slot,
                )
            if len(row_slots) == 1 and self._allow_single_arm:
                mode = GraspMode.LEFT_ONLY if phase == int(Phase.LEFT) else GraspMode.RIGHT_ONLY
                slot = row_slots[0]
                return self._make_grasp_pair_msg(
                    GraspPair,
                    phase,
                    fixed_place_pose_robot,
                    mode,
                    left_slot=slot if mode == GraspMode.LEFT_ONLY else None,
                    right_slot=slot if mode == GraspMode.RIGHT_ONLY else None,
                )
        return None

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

    def _mark_pair_removed(self, pair_msg) -> int:
        from fsm_core.constants import GraspMode, SlotStatus

        active_slot_ids = []
        if pair_msg.grasp_mode in (int(GraspMode.DUAL), int(GraspMode.LEFT_ONLY)) and pair_msg.left_slot_id:
            active_slot_ids.append(pair_msg.left_slot_id)
        if pair_msg.grasp_mode in (int(GraspMode.DUAL), int(GraspMode.RIGHT_ONLY)) and pair_msg.right_slot_id:
            active_slot_ids.append(pair_msg.right_slot_id)
        removed = 0
        for slot in self._grid_slots:
            if slot.slot_id in active_slot_ids and int(slot.status) == int(SlotStatus.OCCUPIED):
                slot.status = int(SlotStatus.REMOVED)
                slot.visible = False
                removed += 1
        return removed

    def _phase_has_occupied_slots(self, phase: int) -> bool:
        from fsm_core.constants import SlotStatus

        phase_cols = self._phase_cols(phase)
        return any(int(slot.status) == int(SlotStatus.OCCUPIED) and int(slot.col_index) in phase_cols for slot in self._grid_slots)

    def _phase_progress_percent(self, phase: int) -> int:
        from fsm_core.constants import SlotStatus

        phase_cols = self._phase_cols(phase)
        phase_slots = [slot for slot in self._grid_slots if int(slot.col_index) in phase_cols]
        if not phase_slots:
            return 0
        removed = sum(1 for slot in phase_slots if int(slot.status) == int(SlotStatus.REMOVED))
        return int(round(100.0 * removed / len(phase_slots)))

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
