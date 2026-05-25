from __future__ import annotations

import json
import time

from fsm_core.constants import ClearErrorStage, TaskCommand
from fsm_core.error_code import ErrorCode, get_error_meta
from fsm_core.state_context import ErrorReportData
from .context import TaskContext


SYSTEM_BOOTING = "BOOTING"
SYSTEM_SELF_CHECK = "SELF_CHECK"
SYSTEM_STANDBY = "STANDBY"
SYSTEM_AUTO_MODE = "AUTO_MODE"
SYSTEM_FAULT = "FAULT"
SYSTEM_E_STOP = "E_STOP"

TASK_WAIT = "WAIT_TASK"
TASK_ACCEPT = "ACCEPT_TASK"
TASK_VALIDATE = "VALIDATE_TASK"
TASK_PREPARE = "PREPARE_TASK"
TASK_RUN = "RUN_TASK"
TASK_VERIFY = "VERIFY_TASK_RESULT"
TASK_COMPLETE = "COMPLETE_TASK"
TASK_FAIL = "FAIL_TASK"
TASK_CANCEL = "CANCEL_TASK"


class TaskManagerNodeMixin:
    def _init_task_manager_runtime(self) -> None:
        from fsm_core.ros2_helpers import get_action_name, get_service_name
        from fsm_msgs.action import RunWallDestacking
        from fsm_msgs.srv import BaseRecoveryCommand
        from rclpy.action import ActionClient

        self.ctx = TaskContext(config=self.config)
        self._system_state = SYSTEM_BOOTING
        self._task_state = TASK_WAIT
        self._ready_state = self._system_state
        self._state_enter_monotonic = time.monotonic()
        self._task_state_enter_monotonic = self._state_enter_monotonic
        self._last_error_code = 0
        self._run_wall_destacking_client = ActionClient(
            self,
            RunWallDestacking,
            get_action_name(self, "run_wall_destacking", "/run_wall_destacking"),
            callback_group=self._action_group,
        )
        self._base_recovery_client = self.create_client(
            BaseRecoveryCommand,
            get_service_name(self, "nav_base_recovery", "/nav/base_recovery"),
            callback_group=self._service_client_group,
        )
        self.ctx.wall_destacking_action_client = self._run_wall_destacking_client

    def _tick(self) -> None:
        self._sync_estop_state()
        self._tick_system_fsm()
        self._tick_task_fsm()

    def handle_task_control(self, request, response):
        command = (request.command or "").strip().lower()
        if command not in (TaskCommand.START, TaskCommand.PAUSE, TaskCommand.RESUME, TaskCommand.CANCEL):
            response.accepted = False
            response.message = f"unsupported command: {request.command}"
            response.current_task_state = self._task_state
            return response

        handler = {
            TaskCommand.START: self._request_start,
            TaskCommand.PAUSE: self._request_pause,
            TaskCommand.RESUME: self._request_resume,
            TaskCommand.CANCEL: self._request_cancel,
        }[command]
        accepted, message = handler(request)
        response.accepted = accepted
        response.message = message
        response.current_task_state = self._task_state
        return response

    def handle_clear_error(self, request, response):
        del request
        if self._system_state not in (SYSTEM_FAULT, SYSTEM_E_STOP):
            response.cleared = False
            response.stage_reached = int(ClearErrorStage.NONE)
            response.message = f"clear_error is only valid in FAULT/E_STOP, current={self._system_state}"
            return response

        if self._ctx_estop_active():
            response.cleared = False
            response.stage_reached = int(ClearErrorStage.NONE)
            response.message = "estop source is still active"
            return response

        ok, stage, message = self._run_clear_error_protocol()
        response.cleared = ok
        response.stage_reached = int(stage)
        response.message = message
        return response

    def on_safety_status(self, msg):
        self.ctx.safety_status = msg

    def on_perception_health(self, msg):
        self.ctx.perception_health = msg

    def publish_task_state(self):
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = "TaskFSM"
        msg.current_state = self._task_state
        msg.parent_fsm = "RobotSystemFSM"
        msg.parent_state = self._system_state
        msg.task_id = self.ctx.task_id
        msg.state_elapsed_sec = float(time.monotonic() - self._task_state_enter_monotonic)
        msg.last_error_code = int(self._last_error_code)
        msg.extra_json = json.dumps(
            {
                "pause_requested": self.ctx.pause_requested,
                "cancel_requested": self.ctx.cancel_requested,
                "clear_error_stage_reached": self.ctx.clear_error_stage_reached,
            },
            sort_keys=True,
        )
        self._task_state_pub.publish(msg)

    def publish_state_heartbeat(self) -> None:
        if self._fsm_state_pub is None:
            return
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = "RobotSystemFSM"
        msg.current_state = self._system_state
        msg.task_id = self.ctx.task_id
        msg.state_elapsed_sec = float(time.monotonic() - self._state_enter_monotonic)
        msg.last_error_code = int(self._last_error_code)
        msg.extra_json = json.dumps({"task_state": self._task_state}, sort_keys=True)
        self._fsm_state_pub.publish(msg)

    def _request_start(self, request) -> tuple[bool, str]:
        if self._system_state != SYSTEM_STANDBY or self._task_state != TASK_WAIT:
            return False, f"task can only start from STANDBY/{TASK_WAIT}, current={self._system_state}/{self._task_state}"
        task_id = request.task_id.strip() or f"task_{int(time.time())}"
        self.ctx.pending_start = {
            "task_id": task_id,
            "params_json": request.params_json or "{}",
        }
        self._set_system_state(SYSTEM_AUTO_MODE)
        self._set_task_state(TASK_ACCEPT)
        return True, "task accepted"

    def _request_pause(self, request) -> tuple[bool, str]:
        del request
        if self._system_state != SYSTEM_AUTO_MODE or self._task_state not in (TASK_PREPARE, TASK_RUN):
            return False, f"pause rejected from {self._system_state}/{self._task_state}"
        self.ctx.pause_requested = True
        self._cancel_wall_destacking_goal()
        return True, "pause requested"

    def _request_resume(self, request) -> tuple[bool, str]:
        del request
        if self._system_state != SYSTEM_AUTO_MODE or not self.ctx.pause_requested:
            return False, f"resume rejected from {self._system_state}/{self._task_state}"
        self.ctx.pause_requested = False
        self.ctx.resume_requested = True
        if self._task_state == TASK_WAIT and self.ctx.task_id:
            self.ctx.pending_start = {"task_id": self.ctx.task_id, "params_json": self.ctx.task_params_json}
            self._set_task_state(TASK_ACCEPT)
        return True, "resume requested"

    def _request_cancel(self, request) -> tuple[bool, str]:
        del request
        if self._task_state == TASK_WAIT:
            return False, "no active task to cancel"
        self.ctx.cancel_requested = True
        self._set_task_state(TASK_CANCEL)
        self._cancel_wall_destacking_goal()
        return True, "cancel requested"

    def _tick_system_fsm(self) -> None:
        if self._system_state == SYSTEM_BOOTING:
            self._set_task_state(TASK_WAIT)
            self._set_system_state(SYSTEM_SELF_CHECK)
            return

        if self._system_state == SYSTEM_SELF_CHECK:
            if self._self_check_ok():
                self._set_system_state(SYSTEM_STANDBY)
            return

        if self._system_state == SYSTEM_STANDBY:
            return

        if self._system_state == SYSTEM_AUTO_MODE:
            if self._task_state == TASK_WAIT and not self.ctx.pending_start:
                self._set_system_state(SYSTEM_STANDBY)
            elif self._task_state == TASK_FAIL and self.ctx.last_error:
                self._set_system_state(SYSTEM_FAULT)
            return

    def _tick_task_fsm(self) -> None:
        if self._system_state not in (SYSTEM_AUTO_MODE, SYSTEM_FAULT, SYSTEM_E_STOP):
            return
        if self._system_state in (SYSTEM_FAULT, SYSTEM_E_STOP) and self._task_state not in (TASK_FAIL, TASK_CANCEL):
            return

        if self.ctx.cancel_requested and self._task_state not in (TASK_WAIT, TASK_CANCEL):
            self._set_task_state(TASK_CANCEL)

        if self._task_state == TASK_ACCEPT:
            self._accept_task()
        elif self._task_state == TASK_VALIDATE:
            self._validate_task()
        elif self._task_state == TASK_PREPARE:
            self._prepare_task()
        elif self._task_state == TASK_RUN:
            self._run_task()
        elif self._task_state == TASK_VERIFY:
            self._verify_task_result()
        elif self._task_state == TASK_COMPLETE:
            self._complete_task()
        elif self._task_state == TASK_FAIL:
            self._fail_task()
        elif self._task_state == TASK_CANCEL:
            self._cancel_task()

    def _accept_task(self) -> None:
        pending = self.ctx.pending_start
        if not pending:
            self._set_error(ErrorCode.E_TASK_VALIDATE_FAIL, "missing pending task")
            self._set_task_state(TASK_FAIL)
            return
        try:
            json.loads(pending.get("params_json") or "{}")
        except json.JSONDecodeError as exc:
            self._set_error(ErrorCode.E_TASK_VALIDATE_FAIL, f"invalid params_json: {exc}")
            self._set_task_state(TASK_FAIL)
            return
        self.ctx.task_id = pending["task_id"]
        self.ctx.task_params_json = pending.get("params_json") or "{}"
        self.ctx.task_start_time = time.monotonic()
        self.ctx.pending_start = None
        self.ctx.cancel_requested = False
        self.ctx.wall_destacking_result = None
        self._set_task_state(TASK_VALIDATE)

    def _validate_task(self) -> None:
        if self._ctx_estop_active():
            self._set_error(ErrorCode.E_SAFETY_ESTOP_HW, "estop during task validation")
            self._set_task_state(TASK_FAIL)
            return
        if self._system_state != SYSTEM_AUTO_MODE:
            self._set_error(ErrorCode.E_TASK_PRECONDITION_FAIL, f"system mode is {self._system_state}")
            self._set_task_state(TASK_FAIL)
            return
        self._set_task_state(TASK_PREPARE)

    def _prepare_task(self) -> None:
        self._set_task_state(TASK_RUN)

    def _run_task(self) -> None:
        from action_msgs.msg import GoalStatus
        from fsm_core.error_code import ErrorCode

        if self.ctx.pause_requested:
            return
        if self.ctx.cancel_requested:
            self._set_task_state(TASK_CANCEL)
            return
        if self._ctx_estop_active():
            self._cancel_wall_destacking_goal()
            self._set_error(ErrorCode.E_SAFETY_ESTOP_HW, "estop during task run")
            self._set_task_state(TASK_FAIL)
            return
        if self.ctx.wall_destacking_goal_future is None:
            self._send_wall_destacking_goal()
            return
        if self.ctx.wall_destacking_goal_handle is None:
            if not self._adopt_wall_destacking_goal_if_ready():
                return
            return
        if self.ctx.wall_destacking_result_future is None or not self.ctx.wall_destacking_result_future.done():
            return
        result_wrapper = self.ctx.wall_destacking_result_future.result()
        self.ctx.wall_destacking_result = result_wrapper.result
        if result_wrapper.status == GoalStatus.STATUS_CANCELED:
            self._set_task_state(TASK_CANCEL)
        else:
            self._set_task_state(TASK_VERIFY)

    def _verify_task_result(self) -> None:
        result = self.ctx.wall_destacking_result
        if result is None:
            self._set_error(ErrorCode.E_TASK_CHILD_FAILED, "missing wall destacking result")
            self._set_task_state(TASK_FAIL)
            return
        if result.success:
            self._set_task_state(TASK_COMPLETE)
            return
        code = int(result.error_code or ErrorCode.E_TASK_CHILD_FAILED)
        self._set_error(code, result.failure_reason or "wall destacking failed")
        self._set_task_state(TASK_FAIL)

    def _complete_task(self) -> None:
        self._reset_task_runtime(clear_identity=True)
        self._set_task_state(TASK_WAIT)

    def _fail_task(self) -> None:
        if self._system_state == SYSTEM_AUTO_MODE:
            self._set_system_state(SYSTEM_FAULT)

    def _cancel_task(self) -> None:
        from action_msgs.msg import GoalStatus

        self._cancel_wall_destacking_goal()
        if self._wall_destacking_cancel_in_progress():
            return
        result_future = self.ctx.wall_destacking_result_future
        if result_future is not None and not result_future.done():
            return
        if result_future is not None and result_future.done():
            result_wrapper = result_future.result()
            self.ctx.wall_destacking_result = result_wrapper.result
            if result_wrapper.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                self.get_logger().warning(f"task cancel finished with strategy status={result_wrapper.status}")
        self.ctx.cancel_requested = False
        self.ctx.pause_requested = False
        self._set_error(ErrorCode.E_MAN_CANCELLED, "task cancelled")
        self._reset_task_runtime(clear_identity=True)
        self._set_task_state(TASK_WAIT)

    def _send_wall_destacking_goal(self) -> None:
        from fsm_msgs.action import RunWallDestacking

        if not self._run_wall_destacking_client.server_is_ready():
            if not self._run_wall_destacking_client.wait_for_server(timeout_sec=0.0):
                return
        goal = RunWallDestacking.Goal()
        goal.task_id = self.ctx.task_id
        goal.start_wall_index = 0
        goal.max_walls = int(self.config.get("business.max_walls", 0))
        goal.fixed_place_pose_robot = self._make_fixed_place_pose()
        goal.config_overrides_json = self.ctx.task_params_json
        self.ctx.wall_destacking_goal_future = self._run_wall_destacking_client.send_goal_async(
            goal,
            feedback_callback=self._on_wall_destacking_feedback,
        )

    def _on_wall_destacking_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.ctx.wall_index = int(feedback.current_wall_index)
        self.ctx.phase = int(feedback.current_phase)

    def _cancel_wall_destacking_goal(self) -> None:
        self._adopt_wall_destacking_goal_if_ready()
        handle = self.ctx.wall_destacking_goal_handle
        if handle is None:
            return
        result_future = self.ctx.wall_destacking_result_future
        if result_future is not None and result_future.done():
            return
        cancel_future = self.ctx.wall_destacking_cancel_future
        if cancel_future is not None and not cancel_future.done():
            return
        try:
            self.ctx.wall_destacking_cancel_future = handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - ROS2 防御兜底
            self.get_logger().warning(f"failed to cancel wall destacking goal: {exc}")

    def _wall_destacking_cancel_in_progress(self) -> bool:
        future = self.ctx.wall_destacking_cancel_future
        return bool(future is not None and not future.done())

    def _run_clear_error_protocol(self) -> tuple[bool, int, str]:
        from fsm_msgs.srv import BaseRecoveryCommand

        steps = (
            (BaseRecoveryCommand.Request.RELEASE_ESTOP, ClearErrorStage.ESTOP_RELEASED, ErrorCode.E_SAFETY_ESTOP_LOCK_STUCK),
            (None, ClearErrorStage.ACTIONS_CANCELED, ErrorCode.E_SYS_ACTION_CANCEL_TIMEOUT),
            (BaseRecoveryCommand.Request.RESET_FAULT, ClearErrorStage.FAULT_RESET, ErrorCode.E_CHASSIS_FAULT_RESET_FAIL),
            (BaseRecoveryCommand.Request.ENABLE_CHASSIS, ClearErrorStage.CHASSIS_ENABLED, ErrorCode.E_CHASSIS_ENABLE_FAIL),
        )

        for command, stage, fallback_error in steps:
            if command is None:
                if not self._clear_error_cancel_actions(timeout_sec=2.0):
                    self._set_error(fallback_error, "clear_error action cancel timeout")
                    return False, int(stage), "action cancel timeout"
                self.ctx.clear_error_stage_reached = int(stage)
                continue
            success, message, error_code = self._call_base_recovery(command, timeout_sec=2.0)
            if not success:
                self._set_error(error_code or fallback_error, message)
                return False, int(stage), message
            self.ctx.clear_error_stage_reached = int(stage)

        self._reset_task_runtime(clear_identity=True)
        self.ctx.cancel_requested = False
        self.ctx.pause_requested = False
        self._last_error_code = 0
        self.ctx.last_error = None
        self._set_task_state(TASK_WAIT)
        self._set_system_state(SYSTEM_SELF_CHECK)
        self.ctx.clear_error_stage_reached = int(ClearErrorStage.SELF_CHECK)
        return True, int(ClearErrorStage.SELF_CHECK), "clear_error completed, re-entered SELF_CHECK"

    def _call_base_recovery(self, command: int, timeout_sec: float) -> tuple[bool, str, int]:
        from fsm_msgs.srv import BaseRecoveryCommand

        if not self._base_recovery_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "base recovery service unavailable", int(ErrorCode.E_COMM_SERVICE_TIMEOUT)
        request = BaseRecoveryCommand.Request()
        request.command = command
        future = self._base_recovery_client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, "base recovery service timeout", int(ErrorCode.E_COMM_SERVICE_TIMEOUT)
        result = future.result()
        return bool(result.success), result.message, int(result.error_code)

    def _clear_error_cancel_actions(self, timeout_sec: float) -> bool:
        self._cancel_wall_destacking_goal()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            goal_future = self.ctx.wall_destacking_goal_future
            result_future = self.ctx.wall_destacking_result_future
            cancel_future = self.ctx.wall_destacking_cancel_future
            if goal_future is not None and not goal_future.done():
                time.sleep(0.01)
                self._adopt_wall_destacking_goal_if_ready()
                continue
            self._adopt_wall_destacking_goal_if_ready()
            if self.ctx.wall_destacking_goal_handle is None:
                return True
            if cancel_future is not None and not cancel_future.done():
                time.sleep(0.01)
                continue
            if result_future is not None and result_future.done():
                return True
            time.sleep(0.01)
        return False

    def _adopt_wall_destacking_goal_if_ready(self) -> bool:
        from fsm_core.error_code import ErrorCode

        if self.ctx.wall_destacking_goal_handle is not None:
            return True
        future = self.ctx.wall_destacking_goal_future
        if future is None or not future.done():
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._set_error(ErrorCode.E_TASK_CHILD_FAILED, "run_wall_destacking goal rejected")
            if self._task_state == TASK_RUN:
                self._set_task_state(TASK_FAIL)
            return True
        self.ctx.wall_destacking_goal_handle = goal_handle
        self.ctx.wall_destacking_result_future = goal_handle.get_result_async()
        return True

    def _self_check_ok(self) -> bool:
        return not self._ctx_estop_active()

    def _sync_estop_state(self) -> None:
        if not self._ctx_estop_active():
            return
        if self._system_state == SYSTEM_E_STOP:
            return
        self._cancel_wall_destacking_goal()
        code = ErrorCode.E_SAFETY_ESTOP_SW if getattr(self.ctx.safety_status, "estop_source", "") == "software" else ErrorCode.E_SAFETY_ESTOP_HW
        self._set_error(code, "estop active")
        self._set_system_state(SYSTEM_E_STOP)
        if self._task_state not in (TASK_WAIT, TASK_FAIL, TASK_CANCEL):
            self._set_task_state(TASK_FAIL)

    def _ctx_estop_active(self) -> bool:
        return bool(self.ctx.safety_status and self.ctx.safety_status.estop)

    def _set_system_state(self, state: str) -> None:
        if self._system_state == state:
            return
        previous = self._system_state
        self._system_state = state
        self._ready_state = state
        self.ctx.system_mode = state
        self._state_enter_monotonic = time.monotonic()
        self._publish_log_event("RobotSystemFSM", "transition", previous, state)
        self.publish_state_heartbeat()

    def _set_task_state(self, state: str) -> None:
        if self._task_state == state:
            return
        previous = self._task_state
        self._task_state = state
        self.ctx.task_state = state
        self._task_state_enter_monotonic = time.monotonic()
        self._publish_log_event("TaskFSM", "transition", previous, state)
        self.publish_task_state()

    def _set_error(self, code: int | ErrorCode, message: str) -> None:
        meta = get_error_meta(int(code))
        error = ErrorReportData(
            error_code=int(code),
            level=int(meta.level),
            source=int(meta.source),
            source_node=self.get_name(),
            source_fsm="TaskFSM" if self._task_state != TASK_WAIT else "RobotSystemFSM",
            source_state=self._task_state if self._task_state != TASK_WAIT else self._system_state,
            message=message,
            recommended_recovery=meta.default_recovery.name,
            extra_json="{}",
        )
        self.ctx.last_error = error
        self.ctx.error_history.append(error)
        self._last_error_code = int(code)
        self._publish_error(error)

    def _publish_error(self, error: ErrorReportData) -> None:
        from fsm_msgs.msg import ErrorReport

        msg = ErrorReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.error_code = int(error.error_code)
        msg.level = int(error.level)
        msg.source = int(error.source)
        msg.source_node = error.source_node
        msg.source_fsm = error.source_fsm
        msg.source_state = error.source_state
        msg.message = error.message
        msg.recommended_recovery = error.recommended_recovery
        msg.extra_json = error.extra_json
        self._error_pub.publish(msg)

    def _publish_log_event(self, fsm_name: str, event_type: str, from_state: str, to_state: str) -> None:
        from fsm_msgs.msg import FsmLogEvent

        msg = FsmLogEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = fsm_name
        msg.event_type = event_type
        msg.from_state = from_state
        msg.to_state = to_state
        msg.transition = f"{from_state}->{to_state}"
        msg.task_id = self.ctx.task_id
        msg.duration_ms = 0.0
        msg.error_code = int(self._last_error_code)
        msg.extra_json = "{}"
        self._log_pub.publish(msg)

    def _reset_task_runtime(self, clear_identity: bool) -> None:
        self.ctx.wall_destacking_goal_handle = None
        self.ctx.wall_destacking_goal_future = None
        self.ctx.wall_destacking_result_future = None
        self.ctx.wall_destacking_cancel_future = None
        self.ctx.wall_destacking_result = None
        self.ctx.pending_start = None
        if clear_identity:
            self.ctx.task_id = ""
            self.ctx.task_params_json = "{}"
            self.ctx.task_start_time = 0.0

    def _make_fixed_place_pose(self):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(self.config.get("business.fixed_place_pose.x", 0.5))
        pose.pose.position.y = float(self.config.get("business.fixed_place_pose.y", 0.0))
        pose.pose.position.z = float(self.config.get("business.fixed_place_pose.z", 0.8))
        pose.pose.orientation.x = float(self.config.get("business.fixed_place_pose.qx", 0.0))
        pose.pose.orientation.y = float(self.config.get("business.fixed_place_pose.qy", 0.0))
        pose.pose.orientation.z = float(self.config.get("business.fixed_place_pose.qz", 0.0))
        pose.pose.orientation.w = float(self.config.get("business.fixed_place_pose.qw", 1.0))
        return pose


def main(args=None):
    try:
        import rclpy
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("task_manager_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_service_name, get_topic_name, make_qos_profile
    from fsm_msgs.msg import ErrorReport, FsmLogEvent, FsmStateSnapshot, PerceptionHealth, SafetyStatus
    from fsm_msgs.srv import ClearError, TaskControl

    class TaskManagerNode(SkeletonNodeMixin, TaskManagerNodeMixin, Node):
        def __init__(self):
            super().__init__("task_manager_node")
            self._service_client_group = ReentrantCallbackGroup()
            self._action_group = ReentrantCallbackGroup()
            self._timer_group = MutuallyExclusiveCallbackGroup()
            self.init_fsm_node_base(ready_state=SYSTEM_BOOTING, heartbeat_fsm_name="RobotSystemFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_system_state", "/fsm/system_state", "RobotSystemFSM")
            self._init_task_manager_runtime()

            qos = make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1)
            self._task_state_pub = self.create_publisher(FsmStateSnapshot, get_topic_name(self, "fsm_task_state", "/fsm/task_state"), qos)
            self._error_pub = self.create_publisher(ErrorReport, get_topic_name(self, "fsm_error", "/fsm/error"), 10)
            self._log_pub = self.create_publisher(FsmLogEvent, get_topic_name(self, "fsm_log_event", "/fsm/log_event"), 10)
            self._safety_sub = self.create_subscription(SafetyStatus, get_topic_name(self, "safety_status", "/safety/status"), self.on_safety_status, 10)
            self._perception_health_sub = self.create_subscription(
                PerceptionHealth,
                get_topic_name(self, "perception_health", "/perception/health"),
                self.on_perception_health,
                10,
            )
            self._task_timer = self.create_timer(1.0, self.publish_task_state)
            tick_rate = float(self.config.get("fsm.tick_rate_hz", 20.0))
            self._tick_timer = self.create_timer(1.0 / max(tick_rate, 1.0), self._tick, callback_group=self._timer_group)

            self._task_start_srv = self.create_service(TaskControl, get_service_name(self, "task_start", "/task/start"), self.handle_task_control)
            self._task_pause_srv = self.create_service(TaskControl, get_service_name(self, "task_pause", "/task/pause"), self.handle_task_control)
            self._task_resume_srv = self.create_service(TaskControl, get_service_name(self, "task_resume", "/task/resume"), self.handle_task_control)
            self._task_cancel_srv = self.create_service(TaskControl, get_service_name(self, "task_cancel", "/task/cancel"), self.handle_task_control)
            self._clear_error_srv = self.create_service(ClearError, get_service_name(self, "clear_error", "/clear_error"), self.handle_clear_error)
            self.get_logger().info("task_manager_node ready")

    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
