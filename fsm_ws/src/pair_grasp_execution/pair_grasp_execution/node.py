from __future__ import annotations

import copy
import math
import time


def _make_pose_stamped(frame_id: str = "base_link"):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


class _GraspFailure(Exception):
    def __init__(self, error_code: int, failed_stage: str, message: str):
        super().__init__(message)
        self.error_code = int(error_code)
        self.failed_stage = str(failed_stage)


class _GraspCancelled(Exception):
    pass


class PairGraspExecutionNodeMixin:
    STAGES = [
        ("RECEIVE_PAIR", "CHECK"),
        ("CHECK_PAIR_VALID", "CHECK"),
        ("PLAN_PREGRASP", "PLAN"),
        ("MOVE_TO_PREGRASP", "MOVE"),
        ("APPROACH_AND_CONTACT", "MOVE"),
        ("CHECK_VACUUM", "VACUUM"),
        ("ATTACH_BOX_MODEL", "VACUUM"),
        ("PLAN_EXTRACT", "PLAN"),
        ("EXECUTE_EXTRACT", "MOVE"),
        ("PLAN_CARRY", "PLAN"),
        ("EXECUTE_CARRY", "MOVE"),
        ("RELEASE_BOX", "RELEASE"),
        ("RETREAT_SAFE", "MOVE"),
        ("REPORT", "REPORT"),
    ]

    FAKE_FAILURES = {
        "IK_FAIL": ("PLAN_PREGRASP", "PLAN", 5200),
        "TRAJ_FAIL": ("PLAN_EXTRACT", "PLAN", 5201),
        "COLLISION": ("PLAN_CARRY", "PLAN", 5210),
        "MOVE_FAIL": ("EXECUTE_EXTRACT", "MOVE", 5300),
        "VACUUM_NOT_REACHED": ("CHECK_VACUUM", "VACUUM", 5105),
    }

    def init_grasp_backend(self):
        from fsm_core.ros2_helpers import get_action_name, get_service_name
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.srv import ApplyPlanningScene
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
        self._attached_box_weight_kg = float(self.config.get("business.pair_grasp_execution.attached_box_weight_kg", 2.0))
        self._attached_box_touch_links = list(self.config.get("business.pair_grasp_execution.attached_box_touch_links", []))
        self._planning_frame = str(self.config.get("interfaces.frames.base_link", "base_link"))
        self._skip_vacuum_check = bool(self.config.get("business.pair_grasp_execution.skip_vacuum_check", True))
        self._hold_on_estop = bool(self.config.get("business.vacuum.hold_on_estop", True))
        self._release_on_cancel = bool(self.config.get("business.vacuum.release_on_cancel", False))
        self._attach_threshold_kpa = float(self.config.get("business.vacuum.attach_threshold_kpa", -50.0))
        self._moveit_action_name = get_action_name(self, "moveit_move_group", "/move_action")
        self._apply_planning_scene_service = get_service_name(self, "moveit_apply_planning_scene", "/apply_planning_scene")
        self._moveit_client = ActionClient(self, MoveGroup, self._moveit_action_name, callback_group=self._io_callback_group)
        self._planning_scene_client = self.create_client(
            ApplyPlanningScene,
            self._apply_planning_scene_service,
            callback_group=self._io_callback_group,
        )
        self._active_moveit_goal_handle = None
        self._attached_object_ids = set()

    def handle_goal(self, goal_request):
        from rclpy.action import GoalResponse

        if self._fake_failure_mode == "GOAL_REJECTED":
            return GoalResponse.REJECT
        if self._backend_mode not in ("dry_run", "fake_real", "real"):
            self.get_logger().warning(f"unsupported grasp backend_mode={self._backend_mode}")
            return GoalResponse.REJECT
        try:
            self._validate_grasp_pair(goal_request.grasp_pair)
        except _GraspFailure as exc:
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
        except _GraspFailure as exc:
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

        except _GraspCancelled:
            if self._release_on_cancel:
                self._publish_vacuum_command(False, False)
            await self._clear_attached_boxes_best_effort()
            goal_handle.canceled()
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="CANCELLED",
                error_code=0,
                failed_stage="CANCEL",
                start_time=start,
            )
        except _GraspFailure as exc:
            if vacuum_enabled:
                self._publish_vacuum_command(False, False)
            if not self._hold_on_estop or exc.failed_stage != "ESTOP":
                await self._clear_attached_boxes_best_effort()
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
                raise _GraspFailure(failure[2], failure[1], self._fake_failure_mode)
            return

        if self._backend_mode == "real":
            if stage == "PLAN":
                ok, error_code, reason = await self._call_moveit_plan(goal_handle, state, plan_only=True)
                if not ok:
                    raise _GraspFailure(error_code, stage, reason)
            elif stage == "MOVE" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._call_moveit_plan(goal_handle, state, plan_only=False)
                if not ok:
                    raise _GraspFailure(error_code, stage, reason)
            elif state == "ATTACH_BOX_MODEL" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._apply_attached_boxes(goal_handle.request.grasp_pair, attach=True)
                if not ok:
                    raise _GraspFailure(error_code, stage, reason)
            elif state == "RELEASE_BOX" and not goal_handle.request.dry_run:
                ok, error_code, reason = await self._apply_attached_boxes(goal_handle.request.grasp_pair, attach=False)
                if not ok:
                    raise _GraspFailure(error_code, stage, reason)

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
            raise _GraspCancelled()
        if int(move_result.status) != int(GoalStatus.STATUS_SUCCEEDED):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), f"MoveIt action status {int(move_result.status)}"
        moveit_error = int(getattr(getattr(move_result.result, "error_code", None), "val", 99999))
        if moveit_error != 1:
            return False, self._map_moveit_error(moveit_error, executing=not plan_only), f"MoveIt error_code={moveit_error}"
        return True, 0, ""

    def _fill_motion_plan_request(self, request, targets: list[tuple[str, object]]) -> None:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        request.workspace_parameters.header.frame_id = self._planning_frame
        request.workspace_parameters.min_corner.x = float(self.config.get("business.pair_grasp_execution.planning_workspace.x_min", -1.0))
        request.workspace_parameters.min_corner.y = float(self.config.get("business.pair_grasp_execution.planning_workspace.y_min", -1.0))
        request.workspace_parameters.min_corner.z = float(self.config.get("business.pair_grasp_execution.planning_workspace.z_min", 0.0))
        request.workspace_parameters.max_corner.x = float(self.config.get("business.pair_grasp_execution.planning_workspace.x_max", 1.5))
        request.workspace_parameters.max_corner.y = float(self.config.get("business.pair_grasp_execution.planning_workspace.y_max", 1.0))
        request.workspace_parameters.max_corner.z = float(self.config.get("business.pair_grasp_execution.planning_workspace.z_max", 2.0))
        request.start_state.is_diff = True
        request.num_planning_attempts = max(int(self._moveit_num_planning_attempts), 1)
        request.allowed_planning_time = max(float(self._moveit_allowed_planning_time_sec), 0.1)
        request.max_velocity_scaling_factor = self._bounded_scaling(self._moveit_velocity_scaling)
        request.max_acceleration_scaling_factor = self._bounded_scaling(self._moveit_acceleration_scaling)

        constraints = Constraints()
        constraints.name = "pair_grasp_tcp_targets"
        stamp = self.get_clock().now().to_msg()
        position_box_size = max(float(self._position_tolerance) * 2.0, 0.001)
        for link_name, pose_stamped in targets:
            target = self._normalized_pose(pose_stamped)

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [position_box_size, position_box_size, position_box_size]
            region_pose = Pose()
            region_pose.position = target.pose.position
            region_pose.orientation.w = 1.0

            position = PositionConstraint()
            position.header.stamp = stamp
            position.header.frame_id = target.header.frame_id or self._planning_frame
            position.link_name = str(link_name)
            position.constraint_region.primitives.append(primitive)
            position.constraint_region.primitive_poses.append(region_pose)
            position.weight = 1.0
            constraints.position_constraints.append(position)

            orientation = OrientationConstraint()
            orientation.header.stamp = stamp
            orientation.header.frame_id = target.header.frame_id or self._planning_frame
            orientation.link_name = str(link_name)
            orientation.orientation = target.pose.orientation
            orientation.absolute_x_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.absolute_y_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.absolute_z_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
            orientation.weight = 1.0
            constraints.orientation_constraints.append(orientation)

        request.goal_constraints.append(constraints)

    def _stage_targets(self, pair, state: str) -> list[tuple[str, object]]:
        left_active, right_active = self._active_arms(pair)
        targets = []
        if left_active:
            targets.append((self._left_tip, self._target_pose_for_arm(pair, "left", state)))
        if right_active:
            targets.append((self._right_tip, self._target_pose_for_arm(pair, "right", state)))
        return [(link, pose) for link, pose in targets if pose is not None]

    def _target_pose_for_arm(self, pair, arm: str, state: str):
        if state in ("PLAN_PREGRASP", "MOVE_TO_PREGRASP"):
            return self._offset_pose_along_local_x(self._contact_pose(pair, arm), self._pregrasp_offset_x)
        if state == "APPROACH_AND_CONTACT":
            return self._contact_pose(pair, arm)
        if state in ("PLAN_EXTRACT", "EXECUTE_EXTRACT"):
            return self._offset_pose_along_local_x(self._contact_pose(pair, arm), self._extract_offset_x)
        if state in ("PLAN_CARRY", "EXECUTE_CARRY"):
            return self._place_pose(pair, arm, retreat=False)
        if state == "RETREAT_SAFE":
            return self._place_pose(pair, arm, retreat=True)
        return None

    def _contact_pose(self, pair, arm: str):
        if arm == "left":
            box_pose = self._normalized_pose(pair.left_box_pose_robot)
            box_size = pair.left_box_size
        else:
            box_pose = self._normalized_pose(pair.right_box_pose_robot)
            box_size = pair.right_box_size
        if not self._input_pose_represents_box_center:
            return self._offset_pose_along_local_x(box_pose, self._contact_standoff_x)
        half_depth = self._box_depth(box_size) * 0.5
        return self._offset_pose_along_local_x(box_pose, half_depth + self._contact_standoff_x)

    def _place_pose(self, pair, arm: str, retreat: bool):
        pose = self._normalized_pose(pair.fixed_place_pose_robot)
        if arm == "left":
            pose.pose.position.y += abs(self._place_y_separation) * 0.5
        else:
            pose.pose.position.y -= abs(self._place_y_separation) * 0.5
        if retreat:
            pose = self._offset_pose_along_local_x(pose, self._retreat_offset_x)
        return pose

    def _normalized_pose(self, pose_stamped):
        pose = copy.deepcopy(pose_stamped)
        if not str(pose.header.frame_id):
            pose.header.frame_id = self._planning_frame
        q = pose.pose.orientation
        length = math.sqrt(float(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))
        if length <= 1e-9:
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
        else:
            q.x = float(q.x / length)
            q.y = float(q.y / length)
            q.z = float(q.z / length)
            q.w = float(q.w / length)
        return pose

    def _offset_pose_along_local_x(self, pose_stamped, offset_x: float):
        pose = self._normalized_pose(pose_stamped)
        dx, dy, dz = self._rotate_vector_by_quaternion(
            (float(offset_x), 0.0, 0.0),
            (
                float(pose.pose.orientation.x),
                float(pose.pose.orientation.y),
                float(pose.pose.orientation.z),
                float(pose.pose.orientation.w),
            ),
        )
        pose.pose.position.x += dx
        pose.pose.position.y += dy
        pose.pose.position.z += dz
        return pose

    def _box_depth(self, size) -> float:
        value = float(getattr(size, "x", 0.0))
        if value > 1e-6:
            return value
        return float(self.config.get("business.box_size.length", 0.4))

    async def _apply_attached_boxes(self, pair, attach: bool) -> tuple[bool, int, str]:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.srv import ApplyPlanningScene

        if not self._attach_box_to_planning_scene:
            return True, 0, ""
        if not self._planning_scene_client.wait_for_service(timeout_sec=max(self._moveit_wait_sec, 0.1)):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt apply_planning_scene service unavailable"

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for arm, active, link_name in (
            ("left", self._active_arms(pair)[0], self._left_tip),
            ("right", self._active_arms(pair)[1], self._right_tip),
        ):
            if not active:
                continue
            request.scene.robot_state.attached_collision_objects.append(self._make_attached_object(pair, arm, link_name, attach))

        done, response = await self._wait_future(
            self._planning_scene_client.call_async(request),
            max(self._moveit_wait_sec, 0.1),
            "MoveIt apply_planning_scene",
        )
        if not done or response is None or not bool(response.success):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt apply_planning_scene failed"
        if attach:
            for attached in request.scene.robot_state.attached_collision_objects:
                self._attached_object_ids.add(str(attached.object.id))
        else:
            for attached in request.scene.robot_state.attached_collision_objects:
                self._attached_object_ids.discard(str(attached.object.id))
        return True, 0, ""

    def _make_attached_object(self, pair, arm: str, link_name: str, attach: bool):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import AttachedCollisionObject
        from shape_msgs.msg import SolidPrimitive

        attached = AttachedCollisionObject()
        attached.link_name = str(link_name)
        attached.object.id = self._attached_object_id(pair, arm)
        attached.object.header.frame_id = str(link_name)
        if attach:
            size = pair.left_box_size if arm == "left" else pair.right_box_size
            depth = self._box_depth(size)
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                depth,
                self._dimension_or_default(size, "y", "business.box_size.width"),
                self._dimension_or_default(size, "z", "business.box_size.height"),
            ]
            primitive_pose = Pose()
            primitive_pose.position.x = -depth * 0.5
            primitive_pose.orientation.w = 1.0
            attached.object.primitives.append(primitive)
            attached.object.primitive_poses.append(primitive_pose)
            attached.object.operation = attached.object.ADD
            attached.touch_links = sorted({str(link_name), *[str(item) for item in self._attached_box_touch_links]})
            attached.weight = float(self._attached_box_weight_kg)
        else:
            attached.object.operation = attached.object.REMOVE
        return attached

    async def _clear_attached_boxes_best_effort(self) -> None:
        if not self._attached_object_ids or not self._attach_box_to_planning_scene:
            return
        from moveit_msgs.srv import ApplyPlanningScene
        from moveit_msgs.msg import AttachedCollisionObject

        if not self._planning_scene_client.wait_for_service(timeout_sec=0.2):
            return
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for object_id in sorted(self._attached_object_ids):
            attached = AttachedCollisionObject()
            attached.object.id = str(object_id)
            attached.object.operation = attached.object.REMOVE
            request.scene.robot_state.attached_collision_objects.append(attached)
        done, response = await self._wait_future(self._planning_scene_client.call_async(request), 0.5, "clear attached boxes")
        if done and response is not None and bool(response.success):
            self._attached_object_ids.clear()

    def _attached_object_id(self, pair, arm: str) -> str:
        slot_id = pair.left_slot_id if arm == "left" else pair.right_slot_id
        return f"{pair.pair_id}_{slot_id or arm}_box"

    def _dimension_or_default(self, size, attr: str, fallback_key: str) -> float:
        value = float(getattr(size, attr, 0.0))
        if value > 1e-6:
            return value
        return float(self.config.get(fallback_key, 0.4))

    def _map_moveit_error(self, moveit_error: int, executing: bool) -> int:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.msg import MoveItErrorCodes

        if executing and int(moveit_error) == int(MoveItErrorCodes.CONTROL_FAILED):
            return int(ErrorCode.E_MOT_EXEC_FAIL)
        if int(moveit_error) == int(MoveItErrorCodes.NO_IK_SOLUTION):
            return int(ErrorCode.E_PLAN_IK_FAIL)
        collision_errors = {
            MoveItErrorCodes.START_STATE_IN_COLLISION,
            MoveItErrorCodes.GOAL_IN_COLLISION,
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE,
        }
        if int(moveit_error) in {int(value) for value in collision_errors}:
            return int(ErrorCode.E_PLAN_COLLISION_DETECTED)
        if executing:
            return int(ErrorCode.E_MOT_EXEC_FAIL)
        return int(ErrorCode.E_PLAN_TRAJ_FAIL)

    async def _wait_moveit_result(self, goal_handle, future, timeout_sec: float):
        from fsm_core.error_code import ErrorCode

        deadline = time.monotonic() + max(float(timeout_sec), 0.01)
        while not future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_moveit_goal()
                raise _GraspCancelled()
            if self._estop:
                self._cancel_active_moveit_goal()
                raise _GraspFailure(int(ErrorCode.E_SAFETY_ESTOP_HW), "ESTOP", "estop during MoveIt motion")
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

    @staticmethod
    def _bounded_scaling(value: float) -> float:
        return max(0.01, min(float(value), 1.0))

    @staticmethod
    def _rotate_vector_by_quaternion(
        vector: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = quaternion
        vx, vy, vz = vector
        # q * v * q^-1
        ix = qw * vx + qy * vz - qz * vy
        iy = qw * vy + qz * vx - qx * vz
        iz = qw * vz + qx * vy - qy * vx
        iw = -qx * vx - qy * vy - qz * vz
        return (
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
        )

    def _validate_grasp_pair(self, pair) -> None:
        from fsm_core.error_code import ErrorCode

        if not str(pair.pair_id):
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "pair_id is required")
        left_active, right_active = self._active_arms(pair)
        if not left_active and not right_active:
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "no active arm in grasp_mode")
        if left_active and not str(pair.left_slot_id):
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "left_slot_id is required")
        if right_active and not str(pair.right_slot_id):
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "right_slot_id is required")
        if left_active and not self._pose_has_frame(pair.left_box_pose_robot):
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "left_box_pose_robot frame_id is required")
        if right_active and not self._pose_has_frame(pair.right_box_pose_robot):
            raise _GraspFailure(int(ErrorCode.E_GRASP_INVALID_PAIR), "CHECK", "right_box_pose_robot frame_id is required")

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
        result.result.final_robot_pose = _make_pose_stamped("base_link")
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


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("pair_grasp_execution_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_topic_name
    from fsm_msgs.action import ExecutePairGrasp
    from fsm_msgs.msg import VacuumCommand
    from std_msgs.msg import Bool, Float32MultiArray

    class PairGraspExecutionNode(SkeletonNodeMixin, PairGraspExecutionNodeMixin, Node):
        def __init__(self):
            super().__init__("pair_grasp_execution_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="PairGraspExecutionFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "PairGraspExecutionFSM")
            self._last_pressure_raw = [0.0, 0.0]
            self._estop = False
            self.init_grasp_backend()
            self._action_server = ActionServer(
                self,
                ExecutePairGrasp,
                get_action_name(self, "execute_pair_grasp", "/execute_pair_grasp"),
                self.execute_pair_grasp_goal,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
                callback_group=self._io_callback_group,
            )
            self._vacuum_cmd_pub = self.create_publisher(VacuumCommand, get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"), 10)
            self._vacuum_pressure_pub = self.create_publisher(Float32MultiArray, get_topic_name(self, "vacuum_pressure", "/vacuum/pressure"), 10)
            self._vacuum_pressure_sub = self.create_subscription(
                Float32MultiArray,
                get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"),
                self.on_pressure_raw,
                10,
                callback_group=self._io_callback_group,
            )
            self._estop_sub = self.create_subscription(
                Bool,
                get_topic_name(self, "safety_estop", "/safety/estop"),
                self.on_estop,
                10,
                callback_group=self._io_callback_group,
            )
            self._pressure_forward_timer = self.create_timer(0.05, self.publish_pressure_forward)
            self.get_logger().info(
                f"pair_grasp_execution_node ready backend_mode={self._backend_mode} moveit_action={self._moveit_action_name}"
            )

    rclpy.init(args=args)
    node = PairGraspExecutionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
