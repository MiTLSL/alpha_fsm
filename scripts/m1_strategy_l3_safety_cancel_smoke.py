#!/usr/bin/env python3
from __future__ import annotations

import json
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


class L3SafetyCancelHarness:
    def __init__(self):
        import rclpy
        from fsm_msgs.msg import FsmStateSnapshot, SafetyStatus, VacuumCommand
        from fsm_msgs.srv import ClearError, TaskControl
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.ClearError = ClearError
        self.TaskControl = TaskControl
        self.Trigger = Trigger
        self.node = Node("m1_strategy_l3_safety_cancel_harness")
        qos = QoSProfile(depth=80, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.system_state = ""
        self.task_state = ""
        self.wall_state = ""
        self.safety_estop = False
        self.safety_state = ""
        self.saw_self_check_after_clear = False
        self.saw_task_cancel = False
        self.saw_wait_after_cancel = False
        self.vacuum_cmd_count = 0
        self.strategy_events: list[tuple[str, str, dict]] = []
        self.mock_events: list[tuple[str, str, str]] = []

        self.node.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._on_system_state, qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._on_task_state, qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/wall_destacking_state", self._on_wall_state, qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/active_substate", self._on_active_substate, qos)
        self.node.create_subscription(SafetyStatus, "/safety/status", self._on_safety_status, qos)
        self.node.create_subscription(VacuumCommand, "/vacuum/cmd", self._on_vacuum_cmd, 20)

        self.start_client = self.node.create_client(TaskControl, "/task/start")
        self.cancel_client = self.node.create_client(TaskControl, "/task/cancel")
        self.clear_client = self.node.create_client(ClearError, "/clear_error")
        self.press_client = self.node.create_client(Trigger, "/mock_safety_button/press")
        self.release_client = self.node.create_client(Trigger, "/mock_safety_button/release")

    def _on_system_state(self, msg):
        self.system_state = msg.current_state
        if msg.current_state == "SELF_CHECK":
            self.saw_self_check_after_clear = True

    def _on_task_state(self, msg):
        self.task_state = msg.current_state
        if msg.current_state == "CANCEL_TASK":
            self.saw_task_cancel = True
        if self.saw_task_cancel and msg.current_state == "WAIT_TASK":
            self.saw_wait_after_cancel = True

    def _on_wall_state(self, msg):
        self.wall_state = msg.current_state

    def _on_active_substate(self, msg):
        if msg.node_name == "wall_destacking_strategy_node":
            try:
                extra = json.loads(msg.extra_json) if msg.extra_json else {}
            except json.JSONDecodeError:
                extra = {}
            self.strategy_events.append((msg.fsm_name, msg.current_state, extra))
            if len(self.strategy_events) > 400:
                self.strategy_events = self.strategy_events[-400:]
            return
        if msg.node_name in ("mock_navigation_manager_node", "mock_pair_grasp_execution_node"):
            self.mock_events.append((msg.node_name, msg.fsm_name, msg.current_state))
            if len(self.mock_events) > 400:
                self.mock_events = self.mock_events[-400:]

    def _on_safety_status(self, msg):
        self.safety_estop = bool(msg.estop)
        try:
            details = json.loads(msg.details_json) if msg.details_json else {}
        except json.JSONDecodeError:
            details = {}
        self.safety_state = str(details.get("safety_state", ""))

    def _on_vacuum_cmd(self, msg):
        del msg
        self.vacuum_cmd_count += 1

    def wait_ready(self) -> None:
        clients = [self.start_client, self.cancel_client, self.clear_client, self.press_client, self.release_client]
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(client.wait_for_service(timeout_sec=0.0) for client in clients):
                return
        raise RuntimeError("services unavailable")

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def call(self, client, request, timeout_sec: float = 6.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise RuntimeError(f"service call timeout: {client.srv_name}")

    def start_task(self, task_id: str) -> None:
        request = self.TaskControl.Request()
        request.command = "start"
        request.task_id = task_id
        request.params_json = "{}"
        response = self.call(self.start_client, request)
        if not response.accepted:
            raise RuntimeError(f"task start rejected: {response.message}")

    def cancel_task(self) -> None:
        request = self.TaskControl.Request()
        request.command = "cancel"
        response = self.call(self.cancel_client, request)
        if not response.accepted:
            raise RuntimeError(f"task cancel rejected: {response.message}")

    def press_estop(self) -> None:
        self.call(self.press_client, self.Trigger.Request())
        self.spin_until(lambda: self.safety_estop, 2.0, "SafetyStatus.estop=true")

    def release_estop(self) -> None:
        self.call(self.release_client, self.Trigger.Request())
        self.spin_until(lambda: not self.safety_estop, 2.0, "SafetyStatus.estop=false")

    def clear_error(self):
        return self.call(self.clear_client, self.ClearError.Request(), timeout_sec=8.0)

    def reset_case_flags(self) -> None:
        self.saw_self_check_after_clear = False
        self.saw_task_cancel = False
        self.saw_wait_after_cancel = False
        self.strategy_events.clear()
        self.mock_events.clear()
        self.vacuum_cmd_count = 0

    def saw_strategy_state(self, state: str) -> bool:
        return any(current == state for _, current, _ in self.strategy_events)

    def saw_mock_node_state(self, node_name: str, state: str) -> bool:
        return any(node == node_name and current == state for node, _, current in self.mock_events)

    def destroy(self) -> None:
        self.node.destroy_node()


def _start_launch() -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch fsm_config bringup_with_mock.launch.py"
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


def _drain_output(selector: selectors.DefaultSelector, output: list[str]) -> None:
    for key, _ in selector.select(timeout=0.01):
        line = key.fileobj.readline()
        if line:
            output.append(line)


def _run() -> int:
    _ensure_ros_python()
    import rclpy

    proc = _start_launch()
    output: list[str] = []
    selector = selectors.DefaultSelector()
    if proc.stdout:
        selector.register(proc.stdout, selectors.EVENT_READ)
    rclpy.init()
    harness = L3SafetyCancelHarness()
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            _drain_output(selector, output)
            try:
                harness.wait_ready()
                break
            except RuntimeError:
                if proc.poll() is not None:
                    raise RuntimeError("bringup launch exited before services were ready")
        else:
            raise RuntimeError("bringup services unavailable")

        harness.spin_until(lambda: harness.system_state == "STANDBY", 5.0, "RobotSystemFSM STANDBY")

        harness.reset_case_flags()
        harness.start_task("l3_e2e_03_estop")
        harness.spin_until(lambda: harness.wall_state == "WAIT_PAIR_GRASP_RESULT", 20.0, "strategy grasp running before estop")
        estop_start = time.monotonic()
        harness.call(harness.press_client, harness.Trigger.Request())
        harness.spin_until(lambda: harness.system_state == "E_STOP", 3.0, "RobotSystemFSM E_STOP")
        harness.spin_until(lambda: harness.safety_state == "EMERGENCY", 2.0, "SafetyMonitorFSM EMERGENCY")
        harness.spin_until(lambda: harness.saw_strategy_state("ESTOP_CANCEL_CHILDREN"), 3.0, "strategy estop child cancel")
        estop_elapsed = time.monotonic() - estop_start
        if estop_elapsed > 0.2:
            raise AssertionError(f"estop response exceeded 200ms: {estop_elapsed:.3f}s")
        harness.spin_until(
            lambda: harness.saw_mock_node_state("mock_pair_grasp_execution_node", "CANCELLED")
            or harness.saw_mock_node_state("mock_pair_grasp_execution_node", "ESTOP_ABORT")
            or harness.saw_mock_node_state("mock_navigation_manager_node", "CANCELLED")
            or harness.saw_mock_node_state("mock_navigation_manager_node", "ESTOP_ABORT"),
            3.0,
            "child action estop cancel/abort",
        )
        harness.release_estop()
        response = harness.clear_error()
        if not response.cleared or int(response.stage_reached) != 5:
            raise AssertionError(f"L3-E2E-03 clear_error failed: {response}")
        harness.spin_until(lambda: harness.saw_self_check_after_clear, 2.0, "SELF_CHECK after clear_error")
        harness.spin_until(lambda: harness.system_state == "STANDBY", 5.0, "STANDBY after clear_error")
        harness.spin_until(lambda: harness.safety_state == "NORMAL", 2.0, "SafetyMonitorFSM NORMAL after clear_error")

        harness.reset_case_flags()
        harness.start_task("l3_e2e_04_cancel")
        harness.spin_until(lambda: harness.wall_state == "WAIT_PAIR_GRASP_RESULT", 20.0, "strategy grasp running before cancel")
        harness.cancel_task()
        harness.spin_until(lambda: harness.saw_strategy_state("CANCEL_REQUESTED"), 3.0, "strategy cancel requested")
        harness.spin_until(lambda: harness.saw_strategy_state("CANCELLED"), 5.0, "strategy cancelled")
        harness.spin_until(lambda: harness.saw_mock_node_state("mock_pair_grasp_execution_node", "CANCELLED"), 5.0, "grasp action cancelled")
        harness.spin_until(lambda: harness.saw_wait_after_cancel and harness.system_state == "STANDBY", 8.0, "task WAIT/STANDBY after cancel")

        print("M1 strategy L3 safety/cancel smoke passed")
        return 0
    except Exception as exc:
        print(f"M1 strategy L3 safety/cancel smoke failed: {exc}", file=sys.stderr)
        print(
            "state snapshot: "
            f"system={harness.system_state} task={harness.task_state} wall={harness.wall_state} "
            f"safety={harness.safety_state}/{harness.safety_estop} "
            f"strategy_events={harness.strategy_events[-12:]} mock_events={harness.mock_events[-12:]}",
            file=sys.stderr,
        )
        if output:
            print("".join(output[-160:]), file=sys.stderr)
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
