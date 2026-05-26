from __future__ import annotations

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
        from fsm_core.ros2_helpers import get_action_name
        from moveit_msgs.action import MoveGroup
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
        self._skip_vacuum_check = bool(self.config.get("business.pair_grasp_execution.skip_vacuum_check", True))
        self._hold_on_estop = bool(self.config.get("business.vacuum.hold_on_estop", True))
        self._release_on_cancel = bool(self.config.get("business.vacuum.release_on_cancel", False))
        self._attach_threshold_kpa = float(self.config.get("business.vacuum.attach_threshold_kpa", -50.0))
        self._moveit_action_name = get_action_name(self, "moveit_move_group", "/move_action")
        self._moveit_client = ActionClient(self, MoveGroup, self._moveit_action_name, callback_group=self._io_callback_group)

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
        return CancelResponse.ACCEPT

    def on_estop(self, msg):
        self._estop = bool(msg.data)
        if self._estop:
            self._ready_state = "ESTOP"
            self.publish_state_heartbeat()

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

        except _GraspFailure as exc:
            if vacuum_enabled:
                self._publish_vacuum_command(False, False)
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
                ok = await self._call_moveit_plan(goal_handle, plan_only=True)
                if not ok:
                    raise _GraspFailure(int(ErrorCode.E_PLAN_TRAJ_FAIL), stage, "MoveIt plan failed")
            elif stage == "MOVE" and not goal_handle.request.dry_run:
                ok = await self._call_moveit_plan(goal_handle, plan_only=False)
                if not ok:
                    raise _GraspFailure(int(ErrorCode.E_MOT_EXEC_FAIL), stage, "MoveIt execute failed")

    async def _call_moveit_plan(self, goal_handle, plan_only: bool) -> bool:
        from action_msgs.msg import GoalStatus
        from moveit_msgs.action import MoveGroup

        if not self._moveit_client.wait_for_server(timeout_sec=max(self._moveit_wait_sec, 0.1)):
            self.get_logger().warning("MoveIt move_action server unavailable")
            return False

        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = self._moveit_group
        move_goal.planning_options.plan_only = bool(plan_only or goal_handle.request.dry_run)
        move_goal.planning_options.planning_scene_diff.is_diff = True
        send_future = self._moveit_client.send_goal_async(move_goal)
        sent, move_goal_handle = await self._wait_future(send_future, self._moveit_wait_sec, "MoveIt send_goal")
        if not sent or move_goal_handle is None or not move_goal_handle.accepted:
            return False
        result_future = move_goal_handle.get_result_async()
        done, move_result = await self._wait_future(result_future, max(goal_handle.request.timeout_sec or 5.0, 1.0), "MoveIt result")
        return bool(done and move_result is not None and int(move_result.status) == int(GoalStatus.STATUS_SUCCEEDED))

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
