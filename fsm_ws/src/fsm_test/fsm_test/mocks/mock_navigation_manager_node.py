from __future__ import annotations

import asyncio

from .common import FailureInjectionMixin, make_pose_stamped


class MockNavigationManagerMixin(FailureInjectionMixin):
    def handle_goal(self, goal_request):
        del goal_request
        from rclpy.action import GoalResponse

        if self._current_failure == "GOAL_REJECTED":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

        return CancelResponse.ACCEPT

    def on_estop(self, msg):
        self._estop = bool(msg.data)

    def publish_nav_health(self):
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = self._current_failure not in ("LOCALIZATION_LOST", "LIFECYCLE_INACTIVE")
        self._nav_health_pub.publish(msg)

    async def execute_navigation_goal(self, goal_handle):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import NavigateToPose

        feedback = NavigateToPose.Feedback()
        stages = [
            "RECEIVE_GOAL",
            "CHECK_PRECONDITION",
            "CHECK_LOCALIZATION",
            "PLAN_PATH",
            "EXECUTE",
            "MONITOR",
        ]
        if goal_handle.request.require_fine_alignment:
            stages.append("FINE_ALIGN")
        stages.extend(["VERIFY", "REPORT"])

        failure_stage = {
            "LOCALIZATION_LOST": "CHECK_LOCALIZATION",
            "PATH_PLAN_FAIL": "PLAN_PATH",
            "STUCK": "EXECUTE",
            "GOAL_TIMEOUT": "EXECUTE",
            "FINE_ALIGN_FAIL": "FINE_ALIGN",
            "FINE_ALIGN_NO_FEEDBACK": "FINE_ALIGN",
            "LIFECYCLE_INACTIVE": "CHECK_PRECONDITION",
        }
        delay_sec = 0.05

        result = NavigateToPose.Result()
        result.actual_base_pose = goal_handle.request.target_pose or make_pose_stamped("map", 0.0, 0.0, 0.0)
        result.position_error = 0.01
        result.yaw_error = 0.005
        result.alignment_error = 0.005 if goal_handle.request.require_fine_alignment else float("nan")
        result.workpose_valid = True
        result.error_code = 0
        result.failure_reason = ""

        for state in stages:
            feedback.current_state = state
            feedback.distance_remaining = 0.0 if state in ("VERIFY", "REPORT") else 0.2
            feedback.estimated_time_remaining = 0.1
            feedback.alignment_error_current = 0.01 if state == "FINE_ALIGN" else float("nan")
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(delay_sec)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.workpose_valid = False
                result.error_code = int(ErrorCode.E_NAV_GOAL_CANCELLED)
                result.failure_reason = "cancelled"
                return result
            if self._estop:
                goal_handle.abort()
                result.success = False
                result.workpose_valid = False
                result.error_code = int(ErrorCode.E_SAFETY_ESTOP_HW)
                result.failure_reason = "estop"
                return result
            if failure_stage.get(self._current_failure) == state:
                break

        failure_map = {
            "LOCALIZATION_LOST": ErrorCode.E_NAV_LOCALIZATION_LOST,
            "FINE_ALIGN_FAIL": ErrorCode.E_NAV_FINE_ALIGN_FAIL,
            "FINE_ALIGN_NO_FEEDBACK": ErrorCode.E_NAV_FINE_ALIGN_NO_FEEDBACK,
            "STUCK": ErrorCode.E_NAV_STUCK,
            "GOAL_TIMEOUT": ErrorCode.E_NAV_GOAL_TIMEOUT,
            "PATH_PLAN_FAIL": ErrorCode.E_NAV_PATH_PLAN_FAIL,
            "LIFECYCLE_INACTIVE": ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE,
        }
        if self._current_failure in failure_map:
            if self._current_failure == "GOAL_TIMEOUT":
                await asyncio.sleep(min(float(goal_handle.request.timeout_sec or 0.5), 2.0))
            goal_handle.abort()
            result.success = False
            result.workpose_valid = False
            result.error_code = int(failure_map[self._current_failure])
            result.failure_reason = self._current_failure
        else:
            goal_handle.succeed()
            result.success = True
        return result

    def handle_base_recovery(self, request, response):
        from fsm_core.constants import ClearErrorStage
        from fsm_core.error_code import ErrorCode

        stage_by_command = {
            request.RELEASE_ESTOP: ClearErrorStage.ESTOP_RELEASED,
            request.RESET_FAULT: ClearErrorStage.FAULT_RESET,
            request.ENABLE_CHASSIS: ClearErrorStage.CHASSIS_ENABLED,
        }
        error_by_failure = {
            "ESTOP_LOCK_STUCK": ErrorCode.E_SAFETY_ESTOP_LOCK_STUCK,
            "CHASSIS_FAULT_RESET_FAIL": ErrorCode.E_CHASSIS_FAULT_RESET_FAIL,
            "CHASSIS_ENABLE_FAIL": ErrorCode.E_CHASSIS_ENABLE_FAIL,
        }
        response.stage_reached = int(stage_by_command.get(request.command, ClearErrorStage.NONE))
        if self._current_failure in error_by_failure:
            response.success = False
            response.error_code = int(error_by_failure[self._current_failure])
            response.message = self._current_failure
        else:
            response.success = True
            response.error_code = 0
            response.message = "mock base recovery ok"
        return response


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_navigation_manager_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_service_name, get_topic_name
    from fsm_msgs.action import NavigateToPose
    from fsm_msgs.srv import BaseRecoveryCommand

    class MockNavigationManagerNode(SkeletonNodeMixin, MockNavigationManagerMixin, Node):
        def __init__(self):
            super().__init__("mock_navigation_manager_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="MockNavigationManager")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "MockNavigationManager")
            self.init_failure_injection()
            self._estop = False
            self._action_server = ActionServer(
                self,
                NavigateToPose,
                get_action_name(self, "navigate_to_pose", "/navigate_to_pose"),
                self.execute_navigation_goal,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
            )
            self._recovery_srv = self.create_service(
                BaseRecoveryCommand,
                get_service_name(self, "nav_base_recovery", "/nav/base_recovery"),
                self.handle_base_recovery,
            )
            self._inject_srv = self.create_inject_failure_service()
            from std_msgs.msg import Bool

            self._nav_health_pub = self.create_publisher(Bool, get_topic_name(self, "nav_health", "/fsm/nav_health"), 1)
            self._estop_sub = self.create_subscription(Bool, get_topic_name(self, "safety_estop", "/safety/estop"), self.on_estop, 10)
            self._nav_health_timer = self.create_timer(1.0, self.publish_nav_health)
            self.get_logger().info("mock_navigation_manager_node ready")

    rclpy.init(args=args)
    node = MockNavigationManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
