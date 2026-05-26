#!/usr/bin/env python3
from __future__ import annotations

import os
import selectors
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "fsm_ws"


def _ensure_ros_python() -> None:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
        env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"/usr/bin/python3 {Path(__file__).resolve()} --ros-python"
        )
        raise SystemExit(subprocess.call(["/bin/bash", "-lc", command], cwd=ROOT, env=env))


class L2Harness:
    def __init__(self):
        import rclpy
        from action_msgs.msg import GoalStatus
        from fsm_msgs.action import RunWallDestacking
        from fsm_msgs.msg import FsmStateSnapshot, SafetyStatus
        from fsm_msgs.srv import ClearError, InjectFailure, TaskControl
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.task import Future
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.GoalStatus = GoalStatus
        self.RunWallDestacking = RunWallDestacking
        self.ClearError = ClearError
        self.InjectFailure = InjectFailure
        self.TaskControl = TaskControl
        self.Trigger = Trigger
        self.CancelResponse = CancelResponse
        self.GoalResponse = GoalResponse
        self.Future = Future

        self.node = Node("m1_task_manager_l2_harness")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.system_state = ""
        self.task_state = ""
        self.safety_estop = False
        self.reject_strategy_cancel = False
        self.finish_strategy_goal = False
        self.strategy_goal_active = False

        self.node.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._on_system_state, qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._on_task_state, qos)
        self.node.create_subscription(SafetyStatus, "/safety/status", self._on_safety_status, qos)
        self.clear_client = self.node.create_client(ClearError, "/clear_error")
        self.inject_client = self.node.create_client(InjectFailure, "/mock_navigation_manager_node/inject_failure")
        self.press_client = self.node.create_client(Trigger, "/mock_safety_button/press")
        self.release_client = self.node.create_client(Trigger, "/mock_safety_button/release")
        self.start_client = self.node.create_client(TaskControl, "/task/start")
        self.strategy_server = ActionServer(
            self.node,
            RunWallDestacking,
            "/run_wall_destacking",
            self._execute_strategy,
            goal_callback=self._handle_strategy_goal,
            cancel_callback=self._handle_strategy_cancel,
        )

    def _on_system_state(self, msg):
        self.system_state = msg.current_state

    def _on_task_state(self, msg):
        self.task_state = msg.current_state

    def _on_safety_status(self, msg):
        self.safety_estop = bool(msg.estop)

    def _handle_strategy_goal(self, goal_request):
        del goal_request
        return self.GoalResponse.ACCEPT

    def _handle_strategy_cancel(self, goal_handle):
        del goal_handle
        return self.CancelResponse.REJECT if self.reject_strategy_cancel else self.CancelResponse.ACCEPT

    async def _execute_strategy(self, goal_handle):
        self.strategy_goal_active = True
        feedback = self.RunWallDestacking.Feedback()
        feedback.current_state = "MOCK_RUNNING"
        goal_handle.publish_feedback(feedback)
        while not self.finish_strategy_goal:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = self.RunWallDestacking.Result()
                result.success = False
                result.error_code = 2003
                result.failure_reason = "cancelled"
                return result
            await self._sleep(0.05)
        goal_handle.succeed()
        result = self.RunWallDestacking.Result()
        result.success = True
        result.walls_completed = 1
        result.total_boxes_picked = 0
        return result

    async def _sleep(self, duration_sec: float):
        future = self.Future()

        def wake():
            timer.cancel()
            future.set_result(None)

        timer = self.node.create_timer(duration_sec, wake)
        await future

    def wait_for_services(self) -> None:
        for client in (self.clear_client, self.inject_client, self.press_client, self.release_client, self.start_client):
            if not client.wait_for_service(timeout_sec=8.0):
                raise RuntimeError(f"service unavailable: {client.srv_name}")

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def call(self, client, request, timeout_sec: float = 5.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise RuntimeError(f"service call timeout: {client.srv_name}")

    def set_nav_failure(self, failure: str) -> None:
        request = self.InjectFailure.Request()
        request.failure_name = failure
        response = self.call(self.inject_client, request)
        if not response.accepted:
            raise RuntimeError(f"inject failure rejected: {failure}")

    def press_estop(self) -> None:
        self.call(self.press_client, self.Trigger.Request())
        self.spin_until(lambda: self.safety_estop, 2.0, "SafetyStatus.estop=true")
        self.spin_until(lambda: self.system_state == "E_STOP", 2.0, "RobotSystemFSM E_STOP")

    def release_estop(self) -> None:
        self.call(self.release_client, self.Trigger.Request())
        self.spin_until(lambda: not self.safety_estop, 2.0, "SafetyStatus.estop=false")

    def clear_error(self):
        return self.call(self.clear_client, self.ClearError.Request(), timeout_sec=7.0)

    def start_task(self):
        request = self.TaskControl.Request()
        request.command = "start"
        request.task_id = "l2_cancel_timeout"
        request.params_json = "{}"
        response = self.call(self.start_client, request)
        if not response.accepted:
            raise RuntimeError(f"task start rejected: {response.message}")

    def destroy(self) -> None:
        self.strategy_server.destroy()
        self.node.destroy_node()


def _start_launch() -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch fsm_test task_manager_l2.launch.py"
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", command],
        cwd=WS,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def _run() -> int:
    _ensure_ros_python()
    import rclpy

    proc = _start_launch()
    output: list[str] = []
    selector = selectors.DefaultSelector()
    if proc.stdout:
        selector.register(proc.stdout, selectors.EVENT_READ)
    rclpy.init()
    harness = L2Harness()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            for key, _ in selector.select(timeout=0.01):
                line = key.fileobj.readline()
                if line:
                    output.append(line)
            rclpy.spin_once(harness.node, timeout_sec=0.05)
            try:
                harness.wait_for_services()
                break
            except RuntimeError:
                pass
        else:
            raise RuntimeError("task_manager L2 services unavailable")

        harness.spin_until(lambda: harness.system_state == "STANDBY", 3.0, "RobotSystemFSM STANDBY")

        harness.press_estop()
        harness.release_estop()
        clear_start = time.monotonic()
        response = harness.clear_error()
        assert response.cleared and response.stage_reached == 5, response
        harness.spin_until(lambda: harness.system_state in ("SELF_CHECK", "STANDBY"), 2.0, "clear_error self check")
        clear_elapsed = time.monotonic() - clear_start
        if clear_elapsed > 6.0:
            raise AssertionError(f"clear_error happy path exceeded 6s: {clear_elapsed:.3f}s")

        failures = [
            ("ESTOP_LOCK_STUCK", 1),
            ("CHASSIS_FAULT_RESET_FAIL", 3),
            ("CHASSIS_ENABLE_FAIL", 4),
        ]
        for failure, stage in failures:
            harness.press_estop()
            harness.release_estop()
            harness.set_nav_failure(failure)
            response = harness.clear_error()
            assert not response.cleared and response.stage_reached == stage, (failure, response)
            harness.set_nav_failure("NONE")
            response = harness.clear_error()
            assert response.cleared and response.stage_reached == 5, response
            harness.spin_until(lambda: harness.system_state == "STANDBY", 3.0, f"recover after {failure}")

        harness.reject_strategy_cancel = True
        harness.start_task()
        harness.spin_until(lambda: harness.task_state == "RUN_TASK", 3.0, "TaskFSM RUN_TASK")
        harness.spin_until(lambda: harness.strategy_goal_active, 3.0, "strategy action active")
        harness.press_estop()
        harness.release_estop()
        response = harness.clear_error()
        assert not response.cleared and response.stage_reached == 2, response

        print("M1 task_manager L2 smoke passed")
        return 0
    except Exception as exc:
        print(f"M1 task_manager L2 smoke failed: {exc}", file=sys.stderr)
        if output:
            print("".join(output[-80:]), file=sys.stderr)
        return 1
    finally:
        harness.destroy()
        rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)


if __name__ == "__main__":
    if "--ros-python" in sys.argv:
        sys.argv.remove("--ros-python")
    raise SystemExit(_run())
