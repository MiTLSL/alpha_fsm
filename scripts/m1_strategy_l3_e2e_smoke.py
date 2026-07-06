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


class L3Harness:
    def __init__(self):
        import rclpy
        from fsm_msgs.msg import FsmStateSnapshot, GraspPair, GridSlotState, WallGridSnapshot
        from fsm_msgs.srv import TaskControl
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        self.GridSlotState = GridSlotState
        self.TaskControl = TaskControl
        self.node = Node("m1_strategy_l3_e2e_harness")

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pair_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.system_state = ""
        self.task_state = ""
        self.wall_state = ""
        self.saw_run = False
        self.saw_complete = False
        self.latest_grid_slot_count = 0
        self.latest_removed_count = 0
        self.latest_grid_rows = 0
        self.latest_grid_cols = 0
        self.pair_count = 0
        self.phases_seen: set[int] = set()

        self.node.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._on_system_state, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._on_task_state, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/wall_destacking_state", self._on_wall_state, state_qos)
        self.node.create_subscription(WallGridSnapshot, "/fsm/grid_snapshot", self._on_grid_snapshot, grid_qos)
        self.node.create_subscription(GraspPair, "/fsm/grasp_pair", self._on_grasp_pair, pair_qos)
        self.start_client = self.node.create_client(TaskControl, "/task/start")

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

    def _on_grid_snapshot(self, msg):
        self.latest_grid_rows = int(msg.rows)
        self.latest_grid_cols = int(msg.cols)
        self.latest_grid_slot_count = len(msg.slots)
        self.latest_removed_count = sum(1 for slot in msg.slots if int(slot.status) == int(self.GridSlotState.STATUS_REMOVED))

    def _on_grasp_pair(self, msg):
        self.pair_count += 1
        self.phases_seen.add(int(msg.phase))

    def wait_for_start_service(self) -> None:
        if not self.start_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("service unavailable: /task/start")

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def call_start(self) -> None:
        request = self.TaskControl.Request()
        request.command = "start"
        request.task_id = "l3_e2e_01"
        request.params_json = "{}"
        future = self.start_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                response = future.result()
                if not response.accepted:
                    raise RuntimeError(f"task start rejected: {response.message}")
                return
        raise RuntimeError("task start service call timeout")

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
    harness = L3Harness()
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            _drain_output(selector, output)
            rclpy.spin_once(harness.node, timeout_sec=0.05)
            try:
                harness.wait_for_start_service()
                break
            except RuntimeError:
                pass
            if proc.poll() is not None:
                raise RuntimeError("bringup launch exited before /task/start was available")
        else:
            raise RuntimeError("/task/start service unavailable")

        harness.spin_until(lambda: harness.system_state == "STANDBY", 5.0, "RobotSystemFSM STANDBY")
        harness.call_start()
        harness.spin_until(lambda: harness.saw_run, 5.0, "TaskFSM RUN_TASK")
        harness.spin_until(lambda: harness.latest_grid_slot_count == 25 and harness.latest_grid_rows == 5 and harness.latest_grid_cols == 5, 8.0, "5x5 grid")
        harness.spin_until(lambda: harness.pair_count > 0, 5.0, "first grasp pair")
        harness.spin_until(
            lambda: (
                harness.saw_complete
                and harness.system_state == "STANDBY"
                and harness.task_state == "WAIT_TASK"
                and harness.latest_removed_count == 25
                and {0, 1}.issubset(harness.phases_seen)
            ),
            45.0,
            "single wall happy completion",
        )

        print("M1 strategy L3 E2E smoke passed")
        return 0
    except Exception as exc:
        print(f"M1 strategy L3 E2E smoke failed: {exc}", file=sys.stderr)
        print(
            "state snapshot: "
            f"system={harness.system_state} task={harness.task_state} wall={harness.wall_state} "
            f"grid={harness.latest_grid_slot_count} removed={harness.latest_removed_count} "
            f"pairs={harness.pair_count} phases={sorted(harness.phases_seen)}",
            file=sys.stderr,
        )
        if output:
            print("".join(output[-120:]), file=sys.stderr)
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
