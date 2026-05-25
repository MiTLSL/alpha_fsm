from __future__ import annotations

from .common import FailureInjectionMixin, make_pose_stamped


class MockPairGraspExecutionMixin(FailureInjectionMixin):
    def _set_mock_state(self, state: str, extra: str = "{}") -> None:
        if state == "ESTOP" and getattr(self, "_ready_state", "") in ("CANCEL_REQUESTED", "CANCELLED", "ESTOP_ABORT"):
            return
        self._ready_state = state
        self._mock_state_extra_json = extra
        self.publish_state_heartbeat()

    def handle_goal(self, goal_request):
        del goal_request
        from rclpy.action import GoalResponse

        if self._current_failure == "GOAL_REJECTED":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

        self._set_mock_state("CANCEL_REQUESTED")
        return CancelResponse.ACCEPT

    def on_estop(self, msg):
        self._estop = bool(msg.data)
        if self._estop:
            self._set_mock_state("ESTOP")

    def on_pressure_raw(self, msg):
        if len(msg.data) >= 2:
            self._vacuum_left_kpa = float(msg.data[0])
            self._vacuum_right_kpa = float(msg.data[1])

    def publish_vacuum_command(self, left_on: bool, right_on: bool) -> None:
        from fsm_msgs.msg import VacuumCommand

        msg = VacuumCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left_on = bool(left_on)
        msg.right_on = bool(right_on)
        msg.command_source = msg.SOURCE_PAIR_GRASP
        self._vacuum_cmd_pub.publish(msg)

    def publish_pressure_forward(self) -> None:
        from std_msgs.msg import Float32MultiArray

        msg = Float32MultiArray()
        msg.data = [float(self._vacuum_left_kpa), float(self._vacuum_right_kpa)]
        self._vacuum_pressure_pub.publish(msg)

    def _active_arms(self, grasp_pair):
        left_active = grasp_pair.grasp_mode in (
            grasp_pair.MODE_DUAL,
            grasp_pair.MODE_LEFT_ONLY,
        )
        right_active = grasp_pair.grasp_mode in (
            grasp_pair.MODE_DUAL,
            grasp_pair.MODE_RIGHT_ONLY,
        )
        return bool(left_active), bool(right_active)

    def _vacuum_attached(self, grasp_pair) -> bool:
        left_active, right_active = self._active_arms(grasp_pair)
        left_ok = (not left_active) or float(self._vacuum_left_kpa) <= float(self._attach_threshold_kpa)
        right_ok = (not right_active) or float(self._vacuum_right_kpa) <= float(self._attach_threshold_kpa)
        return bool(left_ok and right_ok)

    async def _sleep(self, duration_sec: float) -> None:
        from rclpy.task import Future

        future = Future()

        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(float(duration_sec), wake)
        await future

    async def _wait_for_vacuum_threshold(self, goal_handle) -> bool:
        from fsm_msgs.action import ExecutePairGrasp

        deadline = self.get_clock().now().nanoseconds / 1e9 + float(self._buildup_timeout_sec)
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            if goal_handle.is_cancel_requested or self._estop:
                return False
            feedback = ExecutePairGrasp.Feedback()
            feedback.current_state = "CHECK_VACUUM"
            feedback.current_stage = "VACUUM"
            feedback.progress_percent = 40.0
            feedback.vacuum_left_kpa = float(self._vacuum_left_kpa)
            feedback.vacuum_right_kpa = float(self._vacuum_right_kpa)
            goal_handle.publish_feedback(feedback)
            if self._vacuum_attached(goal_handle.request.grasp_pair):
                await self._sleep(min(float(self._hold_sec), 0.05))
                return True
            await self._sleep(0.05)
        return False

    async def execute_pair_grasp_goal(self, goal_handle):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import ExecutePairGrasp

        feedback = ExecutePairGrasp.Feedback()
        stages = [
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
        failure_stage = {
            "IK_FAIL": "PLAN_PREGRASP",
            "TRAJ_FAIL": "PLAN_EXTRACT",
            "COLLISION": "PLAN_CARRY",
            "MOVE_FAIL": "EXECUTE_EXTRACT",
            "VACUUM_NOT_REACHED": "CHECK_VACUUM",
            "VACUUM_UNILATERAL": "CHECK_VACUUM",
            "VACUUM_LOST_DURING_CARRY": "EXECUTE_CARRY",
            "DROP_BOX": "EXECUTE_CARRY",
            "PLACE_FAIL": "RELEASE_BOX",
            "TIMEOUT": "CHECK_VACUUM",
        }

        if not goal_handle.request.dry_run:
            self.publish_vacuum_command(True, True)

        failed_stage = ""
        vacuum_failure_code = 0
        for index, (state, stage) in enumerate(stages):
            self._set_mock_state(state)
            feedback.current_state = state
            feedback.current_stage = stage
            feedback.progress_percent = float(index * 100 / max(len(stages) - 1, 1))
            feedback.vacuum_left_kpa = 0.0 if goal_handle.request.dry_run else self._vacuum_left_kpa
            feedback.vacuum_right_kpa = 0.0 if goal_handle.request.dry_run else self._vacuum_right_kpa
            goal_handle.publish_feedback(feedback)
            await self._sleep(0.05)
            if goal_handle.is_cancel_requested:
                self._set_mock_state("CANCELLED")
                if self._release_on_cancel:
                    self.publish_vacuum_command(False, False)
                goal_handle.canceled()
                return self._make_result(goal_handle, success=False, result_code_name="CANCELLED", error_code=0, failed_stage="CANCEL")
            if self._estop:
                self._set_mock_state("ESTOP_ABORT")
                if not self._hold_on_estop:
                    self.publish_vacuum_command(False, False)
                goal_handle.abort()
                return self._make_result(goal_handle, success=False, result_code_name="ESTOP", error_code=int(ErrorCode.E_SAFETY_ESTOP_HW), failed_stage="ESTOP")
            if failure_stage.get(self._current_failure) == state:
                failed_stage = stage
                break
            if state == "CHECK_VACUUM" and not goal_handle.request.dry_run:
                if not await self._wait_for_vacuum_threshold(goal_handle):
                    failed_stage = stage
                    vacuum_failure_code = int(ErrorCode.E_VAC_NOT_REACHED)
                    break

        if not goal_handle.request.dry_run and (not failed_stage or vacuum_failure_code):
            self.publish_vacuum_command(False, False)

        if vacuum_failure_code:
            goal_handle.abort()
            self._set_mock_state("FAILED", f'{{"error_code":{int(vacuum_failure_code)}}}')
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="FAILED_BOTH",
                error_code=int(vacuum_failure_code),
                failed_stage=failed_stage or "VACUUM",
            )

        failure_map = {
            "GOAL_REJECTED": ErrorCode.E_GRASP_GOAL_REJECTED,
            "IK_FAIL": ErrorCode.E_PLAN_IK_FAIL,
            "TRAJ_FAIL": ErrorCode.E_PLAN_TRAJ_FAIL,
            "COLLISION": ErrorCode.E_PLAN_COLLISION_DETECTED,
            "MOVE_FAIL": ErrorCode.E_MOT_EXEC_FAIL,
            "VACUUM_NOT_REACHED": ErrorCode.E_VAC_NOT_REACHED,
            "VACUUM_UNILATERAL": ErrorCode.E_VAC_UNILATERAL_FAIL,
            "VACUUM_LOST_DURING_CARRY": ErrorCode.E_VAC_LOST_DURING_CARRY,
            "DROP_BOX": ErrorCode.E_MOT_DROP_BOX,
            "PLACE_FAIL": ErrorCode.E_MOT_PLACE_FAIL,
            "TIMEOUT": ErrorCode.E_GRASP_GOAL_TIMEOUT,
        }
        if self._current_failure in failure_map:
            if self._current_failure == "TIMEOUT":
                await self._sleep(min(float(goal_handle.request.timeout_sec or 0.5), 2.0))
            goal_handle.abort()
            self._set_mock_state("FAILED", f'{{"failure":"{self._current_failure}","error_code":{int(failure_map[self._current_failure])}}}')
            return self._make_result(
                goal_handle,
                success=False,
                result_code_name="FAILED_BOTH",
                error_code=int(failure_map[self._current_failure]),
                failed_stage=failed_stage or "PLAN",
            )

        goal_handle.succeed()
        self._set_mock_state("REPORT")
        return self._make_result(goal_handle, success=True, result_code_name="SUCCESS_BOTH", error_code=0, failed_stage="")

    def _make_result(self, goal_handle, success: bool, result_code_name: str, error_code: int, failed_stage: str):
        from fsm_msgs.action import ExecutePairGrasp

        result = ExecutePairGrasp.Result()
        result.success = bool(success)
        result.result.pair_id = goal_handle.request.grasp_pair.pair_id
        result.result.final_robot_pose = make_pose_stamped("base_link", 0.0, 0.0, 0.0)
        result.result.execution_time_sec = 0.5
        result.result.vacuum_left_kpa = 0.0 if goal_handle.request.dry_run else self._vacuum_left_kpa
        result.result.vacuum_right_kpa = 0.0 if goal_handle.request.dry_run else self._vacuum_right_kpa
        result.result.result_code = getattr(result.result, result_code_name)
        left_active = goal_handle.request.grasp_pair.grasp_mode in (
            goal_handle.request.grasp_pair.MODE_DUAL,
            goal_handle.request.grasp_pair.MODE_LEFT_ONLY,
        )
        right_active = goal_handle.request.grasp_pair.grasp_mode in (
            goal_handle.request.grasp_pair.MODE_DUAL,
            goal_handle.request.grasp_pair.MODE_RIGHT_ONLY,
        )
        result.result.left_result = 0 if success and left_active else (1 if left_active else 2)
        result.result.right_result = 0 if success and right_active else (1 if right_active else 2)
        result.result.failed_stage = failed_stage
        result.result.error_code = int(error_code)
        return result


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_pair_grasp_execution_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name
    from fsm_msgs.action import ExecutePairGrasp

    class MockPairGraspExecutionNode(SkeletonNodeMixin, MockPairGraspExecutionMixin, Node):
        def __init__(self):
            super().__init__("mock_pair_grasp_execution_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="MockPairGraspExecution")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "MockPairGraspExecution")
            self.init_failure_injection()
            self._estop = False
            self._vacuum_left_kpa = 0.0
            self._vacuum_right_kpa = 0.0
            self._mock_state_extra_json = "{}"
            self._hold_on_estop = bool(self.config.get("business.vacuum.hold_on_estop", True))
            self._release_on_cancel = bool(self.config.get("business.vacuum.release_on_cancel", False))
            self._attach_threshold_kpa = float(self.config.get("business.vacuum.attach_threshold_kpa", -50.0))
            self._buildup_timeout_sec = float(self.config.get("business.vacuum.buildup_timeout_ms", 800.0)) / 1000.0
            self._hold_sec = float(self.config.get("business.vacuum.hold_ms", 150.0)) / 1000.0
            self._action_server = ActionServer(
                self,
                ExecutePairGrasp,
                get_action_name(self, "execute_pair_grasp", "/execute_pair_grasp"),
                self.execute_pair_grasp_goal,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
            )
            from fsm_core.ros2_helpers import get_topic_name
            from fsm_msgs.msg import VacuumCommand
            from std_msgs.msg import Bool, Float32MultiArray

            self._vacuum_cmd_pub = self.create_publisher(VacuumCommand, get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"), 10)
            self._vacuum_pressure_pub = self.create_publisher(Float32MultiArray, get_topic_name(self, "vacuum_pressure", "/vacuum/pressure"), 10)
            self._vacuum_pressure_sub = self.create_subscription(Float32MultiArray, get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"), self.on_pressure_raw, 10)
            self._estop_sub = self.create_subscription(Bool, get_topic_name(self, "safety_estop", "/safety/estop"), self.on_estop, 10)
            self._pressure_forward_timer = self.create_timer(0.05, self.publish_pressure_forward)
            self._inject_srv = self.create_inject_failure_service()
            self.get_logger().info("mock_pair_grasp_execution_node ready")

    rclpy.init(args=args)
    node = MockPairGraspExecutionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
