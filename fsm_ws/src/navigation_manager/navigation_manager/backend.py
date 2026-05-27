from __future__ import annotations

import math
import threading
import time

from .fine_alignment import FineAlignmentMixin
from .geometry import angle_delta, duration_to_sec, make_pose_stamped, yaw_from_pose


class NavigationManagerNodeMixin(FineAlignmentMixin):
    def init_navigation_backend(self):
        from fsm_core.ros2_helpers import get_action_name, get_service_name, get_topic_name
        from fsm_msgs.msg import BoxDetectionArray
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger

        self._io_callback_group = ReentrantCallbackGroup()
        self._backend_mode = str(self.config.get("business.navigation_manager.backend_mode", "fake_real"))
        self._nav2_action_name = get_action_name(self, "nav2_navigate_to_pose", "/nav2/navigate_to_pose")
        self._service_timeout_sec = float(self.config.get("business.navigation_manager.service_timeout_sec", 2.0))
        self._nav2_wait_sec = float(self.config.get("business.navigation_manager.nav2_client_wait_sec", 2.0))
        self._require_lifecycle_active = bool(self.config.get("business.navigation_manager.require_lifecycle_active", True))
        self._require_amcl_pose = bool(self.config.get("business.navigation_manager.require_amcl_pose", True))
        self._amcl_timeout_sec = float(self.config.get("business.navigation_manager.amcl_timeout_sec", 1.5))
        self._amcl_max_covariance = float(self.config.get("business.navigation_manager.amcl_max_covariance", 0.25))
        self._fine_align_feedback_timeout_sec = float(self.config.get("business.fine_alignment.feedback_timeout_ms", 1000.0)) / 1000.0
        self._fine_align_timeout_sec = float(self.config.get("business.fine_alignment.timeout_sec", 15.0))
        self._fine_align_pass_frames = int(self.config.get("business.fine_alignment.pass_frames", 5))
        self._fine_align_dist_tolerance = float(self.config.get("business.fine_alignment.dist_tolerance", 0.02))
        self._fine_align_yaw_tolerance = float(self.config.get("business.fine_alignment.yaw_tolerance", 0.05))
        self._fine_align_max_linear_x = float(self.config.get("business.fine_alignment.max_linear_x", 0.05))
        self._fine_align_max_angular_z = float(self.config.get("business.fine_alignment.max_angular_z", 0.20))
        self._fine_align_linear_gain = float(self.config.get("business.fine_alignment.linear_gain", 0.8))
        self._fine_align_angular_gain = float(self.config.get("business.fine_alignment.angular_gain", 1.5))
        self._fine_align_min_detection_confidence = float(self.config.get("business.fine_alignment.min_detection_confidence", 0.5))
        self._map_frame = str(self.config.get("interfaces.frames.map", "map"))

        self._last_amcl_pose = None
        self._last_amcl_monotonic = 0.0
        self._last_amcl_covariance = float("inf")
        self._last_nav2_feedback_pose = None
        self._last_nav2_distance_remaining = 0.0
        self._last_nav2_eta_sec = 0.0
        self._last_box_detections = []
        self._last_detection_monotonic = 0.0
        self._last_lifecycle_ok = False
        self._active_nav2_goal_handle = None

        self._nav2_action_client = ActionClient(self, Nav2NavigateToPose, self._nav2_action_name, callback_group=self._io_callback_group)
        self._lifecycle_navigation_client = self.create_client(
            Trigger,
            get_service_name(self, "lifecycle_manager_navigation_is_active", "/lifecycle_manager_navigation/is_active"),
            callback_group=self._io_callback_group,
        )
        self._lifecycle_localization_client = self.create_client(
            Trigger,
            get_service_name(self, "lifecycle_manager_localization_is_active", "/lifecycle_manager_localization/is_active"),
            callback_group=self._io_callback_group,
        )
        self._local_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            get_service_name(self, "local_costmap_clear", "/local_costmap/clear_entirely_local_costmap"),
            callback_group=self._io_callback_group,
        )
        self._global_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            get_service_name(self, "global_costmap_clear", "/global_costmap/clear_entirely_global_costmap"),
            callback_group=self._io_callback_group,
        )
        self._chassis_reset_client = self.create_client(
            Trigger,
            get_service_name(self, "chassis_reset_fault", "/chassis_node/reset_fault"),
            callback_group=self._io_callback_group,
        )
        self._chassis_enable_client = self.create_client(
            Trigger,
            get_service_name(self, "chassis_enable", "/chassis_node/enable"),
            callback_group=self._io_callback_group,
        )
        self._estop_pub = self.create_publisher(Bool, get_topic_name(self, "estop", "/estop"), 10)
        self._cmd_vel_align_pub = self.create_publisher(Twist, get_topic_name(self, "cmd_vel_align", "/cmd_vel_align"), 10)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            get_topic_name(self, "amcl_pose", "/amcl_pose"),
            self.on_amcl_pose,
            10,
            callback_group=self._io_callback_group,
        )
        self._perception_sub = self.create_subscription(
            BoxDetectionArray,
            get_topic_name(self, "perception_detections", "/perception/box_detections"),
            self.on_box_detections,
            5,
            callback_group=self._io_callback_group,
        )

    def handle_goal(self, goal_request):
        del goal_request
        from rclpy.action import GoalResponse

        if self._backend_mode not in ("fake_real", "real"):
            self.get_logger().warning(f"unsupported navigation backend_mode={self._backend_mode}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        del goal_handle
        from rclpy.action import CancelResponse

        self._ready_state = "CANCEL_REQUESTED"
        self.publish_state_heartbeat()
        nav2_goal = getattr(self, "_active_nav2_goal_handle", None)
        if nav2_goal is not None:
            try:
                nav2_goal.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - defensive DDS boundary
                self.get_logger().warning(f"failed to forward nav2 cancel: {exc}")
        return CancelResponse.ACCEPT

    def on_amcl_pose(self, msg):
        self._last_amcl_pose = msg
        self._last_amcl_monotonic = time.monotonic()
        covariance = list(getattr(msg.pose, "covariance", []))
        diagonal_indices = (0, 7, 35)
        values = [abs(float(covariance[index])) for index in diagonal_indices if index < len(covariance)]
        self._last_amcl_covariance = max(values) if values else float("inf")

    def publish_nav_health(self):
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = bool(self._last_lifecycle_ok and self._localization_ok())
        self._nav_health_pub.publish(msg)

    async def execute_navigation_goal(self, goal_handle):
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import NavigateToPose
        from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose

        request = goal_handle.request
        target_pose = request.target_pose
        if not target_pose.header.frame_id:
            target_pose.header.frame_id = self._map_frame

        result = NavigateToPose.Result()
        result.actual_base_pose = make_pose_stamped(target_pose.header.frame_id)
        result.alignment_error = float("nan")

        self._ready_state = "RECEIVE_GOAL"
        self.publish_state_heartbeat()
        self._publish_feedback(goal_handle, "RECEIVE_GOAL", distance=0.0, eta=0.0)

        lifecycle_ok = await self._check_lifecycle_active()
        self._last_lifecycle_ok = lifecycle_ok
        if not lifecycle_ok:
            goal_handle.abort()
            return self._make_failure_result(
                result,
                target_pose,
                int(ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE),
                "Nav2 lifecycle manager is not active",
            )

        self._publish_feedback(goal_handle, "CHECK_LOCALIZATION", distance=0.0, eta=0.0)
        if not self._localization_ok():
            goal_handle.abort()
            return self._make_failure_result(
                result,
                target_pose,
                int(ErrorCode.E_NAV_LOCALIZATION_LOST),
                "AMCL pose missing, stale, or covariance too high",
            )

        if not self._nav2_action_client.wait_for_server(timeout_sec=max(self._nav2_wait_sec, 0.1)):
            goal_handle.abort()
            return self._make_failure_result(result, target_pose, int(ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE), "Nav2 action server unavailable")

        self._publish_feedback(goal_handle, "PLAN_PATH", distance=0.0, eta=0.0)
        nav2_goal = Nav2NavigateToPose.Goal()
        nav2_goal.pose = target_pose

        send_goal_future = self._nav2_action_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda feedback_msg: self._on_nav2_feedback(goal_handle, feedback_msg),
        )
        sent, nav2_goal_handle = await self._wait_future(send_goal_future, self._nav2_wait_sec, "nav2 send_goal")
        if not sent or nav2_goal_handle is None:
            goal_handle.abort()
            return self._make_failure_result(result, target_pose, int(ErrorCode.E_NAV_GOAL_TIMEOUT), "Nav2 send_goal timeout")
        if not nav2_goal_handle.accepted:
            goal_handle.abort()
            return self._make_failure_result(result, target_pose, int(ErrorCode.E_NAV_GOAL_REJECTED), "Nav2 rejected goal")

        self._active_nav2_goal_handle = nav2_goal_handle
        self._publish_feedback(goal_handle, "EXECUTE", distance=self._last_nav2_distance_remaining, eta=self._last_nav2_eta_sec)
        result_future = nav2_goal_handle.get_result_async()
        deadline = time.monotonic() + self._goal_timeout_sec(request.timeout_sec)

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await self._cancel_active_nav2_goal()
                self._active_nav2_goal_handle = None
                goal_handle.canceled()
                return self._make_failure_result(result, target_pose, int(ErrorCode.E_NAV_GOAL_CANCELLED), "cancelled")
            if time.monotonic() >= deadline:
                await self._cancel_active_nav2_goal()
                self._active_nav2_goal_handle = None
                goal_handle.abort()
                return self._make_failure_result(result, target_pose, int(ErrorCode.E_NAV_GOAL_TIMEOUT), "Nav2 goal timeout")
            await self._sleep(0.05)

        self._active_nav2_goal_handle = None
        nav2_result = result_future.result()
        if int(nav2_result.status) != int(GoalStatus.STATUS_SUCCEEDED):
            goal_handle.abort()
            error = ErrorCode.E_NAV_PATH_PLAN_FAIL
            if int(nav2_result.status) == int(GoalStatus.STATUS_CANCELED):
                error = ErrorCode.E_NAV_GOAL_CANCELLED
            return self._make_failure_result(result, target_pose, int(error), f"Nav2 returned status {int(nav2_result.status)}")

        actual_pose = self._last_nav2_feedback_pose or target_pose
        final_alignment_error = float("nan")
        if request.require_fine_alignment:
            ok, error_code, reason, final_alignment_error = await self._run_fine_alignment(goal_handle, request)
            if not ok:
                if error_code == int(ErrorCode.E_NAV_GOAL_CANCELLED):
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._make_failure_result(result, actual_pose, int(error_code), reason)

        self._publish_feedback(goal_handle, "VERIFY", distance=0.0, eta=0.0)
        position_error, yaw_error = self._pose_error(target_pose, actual_pose)
        result.success = True
        result.actual_base_pose = actual_pose
        result.position_error = float(position_error)
        result.yaw_error = float(yaw_error)
        result.alignment_error = float(final_alignment_error)
        result.workpose_valid = True
        result.error_code = 0
        result.failure_reason = ""
        self._ready_state = "REPORT"
        self.publish_state_heartbeat()
        goal_handle.succeed()
        return result

    def _on_nav2_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._last_nav2_feedback_pose = feedback.current_pose
        self._last_nav2_distance_remaining = float(getattr(feedback, "distance_remaining", 0.0))
        self._last_nav2_eta_sec = duration_to_sec(getattr(feedback, "estimated_time_remaining", None))
        self._publish_feedback(
            goal_handle,
            "EXECUTE",
            distance=self._last_nav2_distance_remaining,
            eta=self._last_nav2_eta_sec,
        )

    def _publish_feedback(self, goal_handle, state: str, distance: float, eta: float, alignment_error: float = float("nan")) -> None:
        from fsm_msgs.action import NavigateToPose

        self._ready_state = state
        self.publish_state_heartbeat()
        feedback = NavigateToPose.Feedback()
        feedback.current_state = state
        feedback.distance_remaining = float(distance)
        feedback.estimated_time_remaining = float(eta)
        feedback.alignment_error_current = float(alignment_error)
        goal_handle.publish_feedback(feedback)

    def _make_failure_result(self, result, actual_pose, error_code: int, reason: str):
        result.success = False
        result.actual_base_pose = actual_pose
        result.position_error = 0.0
        result.yaw_error = 0.0
        result.alignment_error = float("nan")
        result.workpose_valid = False
        result.error_code = int(error_code)
        result.failure_reason = str(reason)
        self._ready_state = "FAILED"
        self.publish_state_heartbeat()
        return result

    async def _check_lifecycle_active(self) -> bool:
        if not self._require_lifecycle_active:
            return True
        navigation_ok = await self._call_trigger_async(self._lifecycle_navigation_client, "lifecycle navigation")
        localization_ok = await self._call_trigger_async(self._lifecycle_localization_client, "lifecycle localization")
        return bool(navigation_ok and localization_ok)

    async def _call_trigger_async(self, client, label: str) -> bool:
        from std_srvs.srv import Trigger

        if not client.wait_for_service(timeout_sec=max(self._service_timeout_sec, 0.1)):
            self.get_logger().warning(f"{label} service unavailable")
            return False
        done, response = await self._wait_future(client.call_async(Trigger.Request()), self._service_timeout_sec, label)
        return bool(done and response is not None and response.success)

    def _localization_ok(self) -> bool:
        if not self._require_amcl_pose:
            return True
        if self._last_amcl_pose is None:
            return False
        if time.monotonic() - self._last_amcl_monotonic > self._amcl_timeout_sec:
            return False
        return bool(self._last_amcl_covariance <= self._amcl_max_covariance)

    def _goal_timeout_sec(self, requested_timeout: float) -> float:
        if requested_timeout and float(requested_timeout) > 0.0:
            return float(requested_timeout)
        return 30.0

    def _pose_error(self, target_pose, actual_pose) -> tuple[float, float]:
        dx = float(target_pose.pose.position.x) - float(actual_pose.pose.position.x)
        dy = float(target_pose.pose.position.y) - float(actual_pose.pose.position.y)
        dz = float(target_pose.pose.position.z) - float(actual_pose.pose.position.z)
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_error = abs(angle_delta(yaw_from_pose(target_pose), yaw_from_pose(actual_pose)))
        return position_error, yaw_error

    async def _cancel_active_nav2_goal(self) -> None:
        nav2_goal = getattr(self, "_active_nav2_goal_handle", None)
        if nav2_goal is None:
            return
        try:
            await self._wait_future(nav2_goal.cancel_goal_async(), 1.0, "nav2 cancel")
        except Exception as exc:  # pragma: no cover - defensive DDS boundary
            self.get_logger().warning(f"failed to cancel Nav2 goal: {exc}")

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

    def handle_base_recovery(self, request, response):
        from fsm_core.constants import ClearErrorStage
        from fsm_core.error_code import ErrorCode
        from std_msgs.msg import Bool

        stage_by_command = {
            request.RELEASE_ESTOP: ClearErrorStage.ESTOP_RELEASED,
            request.RESET_FAULT: ClearErrorStage.FAULT_RESET,
            request.ENABLE_CHASSIS: ClearErrorStage.CHASSIS_ENABLED,
        }
        response.stage_reached = int(stage_by_command.get(request.command, ClearErrorStage.NONE))

        if request.command == request.RELEASE_ESTOP:
            msg = Bool()
            msg.data = False
            self._estop_pub.publish(msg)
            response.success = True
            response.error_code = 0
            response.message = "estop release command published"
            return response

        if request.command == request.RESET_FAULT:
            self._clear_costmaps_best_effort()
            ok, message = self._call_trigger_sync(self._chassis_reset_client, "chassis reset_fault")
            response.success = bool(ok)
            response.error_code = 0 if ok else int(ErrorCode.E_CHASSIS_FAULT_RESET_FAIL)
            response.message = message
            return response

        if request.command == request.ENABLE_CHASSIS:
            ok, message = self._call_trigger_sync(self._chassis_enable_client, "chassis enable")
            response.success = bool(ok)
            response.error_code = 0 if ok else int(ErrorCode.E_CHASSIS_ENABLE_FAIL)
            response.message = message
            return response

        response.success = False
        response.error_code = int(ErrorCode.E_SYS_CONFIG_INVALID)
        response.message = "unknown base recovery command"
        return response

    def _clear_costmaps_best_effort(self) -> None:
        from nav2_msgs.srv import ClearEntireCostmap

        request = ClearEntireCostmap.Request()
        for label, client in (
            ("local costmap clear", self._local_costmap_clear_client),
            ("global costmap clear", self._global_costmap_clear_client),
        ):
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warning(f"{label} unavailable")
                continue
            self._wait_sync_future(client.call_async(request), 0.5, label)

    def _call_trigger_sync(self, client, label: str) -> tuple[bool, str]:
        from std_srvs.srv import Trigger

        if not client.wait_for_service(timeout_sec=max(self._service_timeout_sec, 0.1)):
            return False, f"{label} service unavailable"
        ok, response = self._wait_sync_future(client.call_async(Trigger.Request()), self._service_timeout_sec, label)
        if not ok or response is None:
            return False, f"{label} timeout"
        return bool(response.success), str(response.message)

    @staticmethod
    def _wait_sync_future(future, timeout_sec: float, label: str):
        del label
        event = threading.Event()
        holder = {"response": None, "error": None}

        def done_callback(done_future):
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive DDS boundary
                holder["error"] = exc
            finally:
                event.set()

        future.add_done_callback(done_callback)
        if not event.wait(timeout=float(timeout_sec)):
            return False, None
        if holder["error"] is not None:
            return False, None
        return True, holder["response"]
