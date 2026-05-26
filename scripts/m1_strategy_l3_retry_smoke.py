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


class L3RetryHarness:
    def __init__(self):
        import rclpy
        from fsm_msgs.msg import FsmStateSnapshot, GraspPair, GridSlotState, WallGridSnapshot
        from fsm_msgs.srv import InjectFailure, TaskControl
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        self.GridSlotState = GridSlotState
        self.InjectFailure = InjectFailure
        self.TaskControl = TaskControl
        self.node = Node("m1_strategy_l3_retry_harness")

        state_qos = QoSProfile(depth=80, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pair_qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.system_state = ""
        self.task_state = ""
        self.wall_state = ""
        self.saw_run = False
        self.saw_complete = False
        self.latest_removed_count = 0
        self.latest_grid_slot_count = 0
        self.pair_count = 0
        self.phases_seen: set[int] = set()
        self.recovery_events: list[dict] = []
        self.saw_ik_switch_recovery = False

        self.node.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._on_system_state, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._on_task_state, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/wall_destacking_state", self._on_wall_state, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/active_substate", self._on_active_substate, state_qos)
        self.node.create_subscription(WallGridSnapshot, "/fsm/grid_snapshot", self._on_grid_snapshot, grid_qos)
        self.node.create_subscription(GraspPair, "/fsm/grasp_pair", self._on_grasp_pair, pair_qos)

        self.start_client = self.node.create_client(TaskControl, "/task/start")
        self.grasp_inject = self.node.create_client(InjectFailure, "/mock_pair_grasp_execution_node/inject_failure")

    def _on_system_state(self, msg):
        self.system_state = msg.current_state

    def _on_task_state(self, msg):
        self.task_state = msg.current_state
        if msg.current_state == "RUN_TASK":
            self.saw_run = True
        if msg.current_state == "COMPLETE_TASK":
            self.saw_complete = True

    def _on_wall_state(self, msg):
        self.wall_state = msg.current_state

    def _on_active_substate(self, msg):
        if msg.node_name != "wall_destacking_strategy_node" or msg.fsm_name != "WallRecoveryFSM":
            return
        try:
            extra = json.loads(msg.extra_json) if msg.extra_json else {}
        except json.JSONDecodeError:
            extra = {}
        self.recovery_events.append(extra)
        if len(self.recovery_events) > 80:
            self.recovery_events = self.recovery_events[-80:]
        if int(extra.get("error_code", -1)) == 5200 and extra.get("recovery_action") == "SWITCH_TARGET":
            self.saw_ik_switch_recovery = True

    def _on_grid_snapshot(self, msg):
        self.latest_grid_slot_count = len(msg.slots)
        self.latest_removed_count = sum(1 for slot in msg.slots if int(slot.status) == int(self.GridSlotState.STATUS_REMOVED))

    def _on_grasp_pair(self, msg):
        self.pair_count += 1
        self.phases_seen.add(int(msg.phase))

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.start_client.wait_for_service(timeout_sec=0.0) and self.grasp_inject.wait_for_service(timeout_sec=0.0):
                return
        raise RuntimeError("required services unavailable")

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def call_inject_once_ik_fail(self) -> None:
        request = self.InjectFailure.Request()
        request.failure_name = "IK_FAIL"
        request.params_json = json.dumps({"once": True}, sort_keys=True)
        response = self._spin_future(self.grasp_inject.call_async(request), 5.0, "inject IK_FAIL once")
        if not response.accepted:
            raise RuntimeError(f"IK_FAIL injection rejected: {response.message}")

    def call_start(self) -> None:
        request = self.TaskControl.Request()
        request.command = "start"
        request.task_id = "l3_e2e_02_retry"
        request.params_json = "{}"
        response = self._spin_future(self.start_client.call_async(request), 5.0, "task start")
        if not response.accepted:
            raise RuntimeError(f"task start rejected: {response.message}")

    def _spin_future(self, future, timeout_sec: float, label: str):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise RuntimeError(f"timeout waiting for {label}")

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
    harness = L3RetryHarness()
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
        harness.call_inject_once_ik_fail()
        harness.call_start()
        harness.spin_until(lambda: harness.saw_run, 5.0, "TaskFSM RUN_TASK")
        harness.spin_until(lambda: harness.latest_grid_slot_count == 25, 8.0, "5x5 grid")
        harness.spin_until(lambda: harness.saw_ik_switch_recovery, 20.0, "IK_FAIL SWITCH_TARGET recovery")
        harness.spin_until(
            lambda: (
                harness.saw_complete
                and harness.system_state == "STANDBY"
                and harness.task_state == "WAIT_TASK"
                and harness.latest_removed_count == 25
                and harness.pair_count >= 16
                and {0, 1}.issubset(harness.phases_seen)
            ),
            45.0,
            "retry wall completion",
        )

        print("M1 strategy L3 retry smoke passed")
        return 0
    except Exception as exc:
        print(f"M1 strategy L3 retry smoke failed: {exc}", file=sys.stderr)
        print(
            "state snapshot: "
            f"system={harness.system_state} task={harness.task_state} wall={harness.wall_state} "
            f"grid={harness.latest_grid_slot_count} removed={harness.latest_removed_count} "
            f"pairs={harness.pair_count} recovery={harness.recovery_events[-8:]}",
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
