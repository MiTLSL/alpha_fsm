#!/usr/bin/env python3
from __future__ import annotations

import json
import math
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
        from fsm_msgs.action import RunWallDestacking
        from fsm_msgs.msg import FsmStateSnapshot, GraspPair, VacuumCommand, WallGridSnapshot
        from fsm_msgs.srv import InjectFailure
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Float32MultiArray

        self.rclpy = rclpy
        self.RunWallDestacking = RunWallDestacking
        self.InjectFailure = InjectFailure
        self.PoseStamped = PoseStamped
        self.node = Node("m1_strategy_l2_failure_harness")

        state_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.active_events: list[tuple[str, str, dict]] = []
        self.wall_state = ""
        self.latest_recovery: dict = {}
        self.latest_grid_slot_count = 0
        self.latest_grid_occupied_count = 0
        self.pair_count = 0
        self.nav_health = True
        self.vacuum_min_left = 0.0
        self.vacuum_min_right = 0.0
        self.vacuum_cmd_count = 0

        self.node.create_subscription(FsmStateSnapshot, "/fsm/active_substate", self._on_active_substate, state_qos)
        self.node.create_subscription(FsmStateSnapshot, "/fsm/wall_destacking_state", self._on_wall_state, state_qos)
        self.node.create_subscription(WallGridSnapshot, "/fsm/grid_snapshot", self._on_grid_snapshot, grid_qos)
        self.node.create_subscription(GraspPair, "/fsm/grasp_pair", self._on_grasp_pair, 20)
        self.node.create_subscription(Bool, "/fsm/nav_health", self._on_nav_health, 10)
        self.node.create_subscription(Float32MultiArray, "/vacuum/pressure", self._on_vacuum_pressure, 20)
        self.node.create_subscription(VacuumCommand, "/vacuum/cmd", self._on_vacuum_cmd, 20)

        self.action_client = ActionClient(self.node, RunWallDestacking, "/run_wall_destacking")
        self.nav_inject = self.node.create_client(InjectFailure, "/mock_navigation_manager_node/inject_failure")
        self.perc_inject = self.node.create_client(InjectFailure, "/mock_perception_adapter_node/inject_failure")
        self.grasp_inject = self.node.create_client(InjectFailure, "/mock_pair_grasp_execution_node/inject_failure")
        self.vacuum_inject = self.node.create_client(InjectFailure, "/mock_vacuum_io_node/inject_failure")

    def _on_active_substate(self, msg):
        if msg.node_name != "wall_destacking_strategy_node":
            return
        try:
            extra = json.loads(msg.extra_json) if msg.extra_json else {}
        except json.JSONDecodeError:
            extra = {}
        event = (msg.fsm_name, msg.current_state, extra)
        self.active_events.append(event)
        if len(self.active_events) > 500:
            self.active_events = self.active_events[-500:]
        if msg.fsm_name == "WallRecoveryFSM":
            self.latest_recovery = extra

    def _on_wall_state(self, msg):
        self.wall_state = msg.current_state

    def _on_grid_snapshot(self, msg):
        self.latest_grid_slot_count = len(msg.slots)
        self.latest_grid_occupied_count = sum(1 for slot in msg.slots if int(slot.status) == 1)

    def _on_grasp_pair(self, msg):
        del msg
        self.pair_count += 1

    def _on_nav_health(self, msg):
        self.nav_health = bool(msg.data)

    def _on_vacuum_pressure(self, msg):
        if len(msg.data) >= 2:
            self.vacuum_min_left = min(float(self.vacuum_min_left), float(msg.data[0]))
            self.vacuum_min_right = min(float(self.vacuum_min_right), float(msg.data[1]))

    def _on_vacuum_cmd(self, msg):
        del msg
        self.vacuum_cmd_count += 1

    def wait_ready(self) -> None:
        clients = [self.nav_inject, self.perc_inject, self.grasp_inject, self.vacuum_inject]
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.action_client.wait_for_server(timeout_sec=0.0) and all(client.wait_for_service(timeout_sec=0.0) for client in clients):
                return
        raise RuntimeError("strategy action or inject services unavailable")

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin_future(self, future, timeout_sec: float, label: str):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise RuntimeError(f"timeout waiting for {label}")

    def inject(self, client, failure_name: str = "NONE", params: dict | None = None, duration_sec: float = 0.0) -> None:
        request = self.InjectFailure.Request()
        request.failure_name = failure_name
        request.duration_sec = float(duration_sec)
        request.params_json = json.dumps(params or {}, sort_keys=True)
        response = self.spin_future(client.call_async(request), 5.0, f"inject {failure_name}")
        if not response.accepted:
            raise RuntimeError(f"inject {failure_name} rejected: {response.message}")

    def reset_mocks(self, mode: str = "OBSERVATION") -> None:
        self.inject(self.nav_inject, "NONE")
        self.inject(self.perc_inject, "NONE", {"mode": mode})
        self.inject(self.grasp_inject, "NONE")
        self.inject(self.vacuum_inject, "NONE")
        self.spin_for(0.35)

    def reset_observations(self) -> None:
        self.active_events.clear()
        self.latest_recovery = {}
        self.latest_grid_slot_count = 0
        self.latest_grid_occupied_count = 0
        self.pair_count = 0
        self.vacuum_min_left = 0.0
        self.vacuum_min_right = 0.0

    def make_goal(self, task_id: str, overrides: dict | None = None):
        goal = self.RunWallDestacking.Goal()
        goal.task_id = task_id
        goal.start_wall_index = 0
        goal.max_walls = 1
        pose = self.PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.5
        pose.pose.position.z = 0.8
        pose.pose.orientation.w = 1.0
        goal.fixed_place_pose_robot = pose
        goal.config_overrides_json = json.dumps(overrides or {}, sort_keys=True)
        return goal

    def run_strategy(self, task_id: str, overrides: dict | None = None, timeout_sec: float = 45.0, during=None):
        self.reset_observations()
        send_future = self.action_client.send_goal_async(self.make_goal(task_id, overrides))
        goal_handle = self.spin_future(send_future, 5.0, f"send goal {task_id}")
        if not goal_handle.accepted:
            raise RuntimeError(f"strategy goal rejected: {task_id}")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if during is not None:
                during(self)
            if result_future.done():
                return result_future.result().result
        raise RuntimeError(f"strategy result timeout: {task_id}")

    def saw_event(self, fsm_name: str, state: str) -> bool:
        return any(fsm == fsm_name and current == state for fsm, current, _ in self.active_events)

    def saw_navigation_fine_align(self) -> bool:
        for fsm, state, extra in self.active_events:
            if fsm != "NavigationClient" or state != "FINE_ALIGN":
                continue
            value = float(extra.get("alignment_error_current", float("nan")))
            if not math.isnan(value):
                return True
        return False

    def expect_recovery(self, error_code: int, recovery_action: str) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if int(self.latest_recovery.get("error_code", -1)) == int(error_code):
                break
        actual_code = int(self.latest_recovery.get("error_code", -1))
        actual_action = str(self.latest_recovery.get("recovery_action", ""))
        if actual_code != int(error_code) or actual_action != recovery_action:
            raise AssertionError(f"expected recovery {error_code}/{recovery_action}, got {self.latest_recovery}")

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


def _assert_result(result, success: bool, error_code: int, label: str) -> None:
    if bool(result.success) != bool(success) or int(result.error_code) != int(error_code):
        raise AssertionError(
            f"{label}: expected success={success} error={error_code}, "
            f"got success={result.success} error={result.error_code} reason={result.failure_reason}"
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
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            _drain_output(selector, output)
            try:
                harness.wait_ready()
                break
            except RuntimeError:
                if proc.poll() is not None:
                    raise RuntimeError("bringup launch exited before strategy was ready")
        else:
            raise RuntimeError("strategy action/services unavailable")

        harness.reset_mocks()

        result = harness.run_strategy("l2_happy", timeout_sec=60.0)
        _assert_result(result, True, 0, "L2 happy")
        if not harness.saw_event("WallMappingFSM", "REPORT") or harness.latest_grid_slot_count != 25:
            raise AssertionError("L2-PERC-01: mapping did not publish 25-slot grid")
        if not harness.saw_navigation_fine_align():
            raise AssertionError("L2-NAV-03: FINE_ALIGN feedback was not observed")
        if harness.vacuum_min_left > -50.0 or harness.vacuum_min_right > -50.0:
            raise AssertionError(f"L2-GRASP-01: vacuum peak too weak: {harness.vacuum_min_left}, {harness.vacuum_min_right}")

        nav_cases = [
            ("L2-NAV-02", "LOCALIZATION_LOST", 4010, "RELOCALIZE", None),
            ("L2-NAV-05", "STUCK", 4022, "RETREAT_SAFE", None),
            ("L2-NAV-06", "GOAL_TIMEOUT", 4001, "RETRY_CURRENT_STATE", None),
            ("L2-NAV-07", "PATH_PLAN_FAIL", 4020, "REPLAN", None),
            ("L2-NAV-08", "LIFECYCLE_INACTIVE", 4050, "WAIT_MANUAL_RECOVERY", None),
        ]
        for label, failure, code, recovery, during in nav_cases:
            harness.reset_mocks()
            harness.inject(harness.nav_inject, failure)
            harness.spin_for(1.1 if failure == "LIFECYCLE_INACTIVE" else 0.2)
            result = harness.run_strategy(label.lower(), timeout_sec=20.0, during=during)
            _assert_result(result, False, code, label)
            harness.expect_recovery(code, recovery)
            if failure == "LIFECYCLE_INACTIVE" and harness.nav_health:
                raise AssertionError("L2-NAV-08: /fsm/nav_health did not become false")

        harness.reset_mocks()
        fine_align_injected = {"done": False}

        def inject_fine_align_fail(h: L2Harness) -> None:
            if fine_align_injected["done"]:
                return
            if h.latest_grid_slot_count == 25:
                h.inject(h.nav_inject, "FINE_ALIGN_FAIL")
                fine_align_injected["done"] = True

        result = harness.run_strategy("l2_nav_04", timeout_sec=25.0, during=inject_fine_align_fail)
        _assert_result(result, False, 4040, "L2-NAV-04")
        harness.expect_recovery(4040, "RETRY_CURRENT_STATE")

        perception_cases = [
            ("L2-PERC-02", "NONE", {"mode": "PARTIAL"}, 3111, "RETRY_CURRENT_STATE"),
            ("L2-PERC-03", "LIDAR_OFFLINE", {"mode": "OBSERVATION"}, 8011, "WAIT_MANUAL_RECOVERY"),
            ("L2-PERC-05", "STOP_PUBLISHING", {"mode": "OBSERVATION"}, 3100, "RETRY_CURRENT_STATE"),
            ("L2-PERC-07", "CAMERA_OFFLINE", {"mode": "OBSERVATION"}, 8010, "WAIT_MANUAL_RECOVERY"),
        ]
        for label, failure, params, code, recovery in perception_cases:
            harness.reset_mocks()
            harness.inject(harness.perc_inject, failure, params)
            harness.spin_for(0.35)
            result = harness.run_strategy(label.lower(), timeout_sec=20.0)
            _assert_result(result, False, code, label)
            harness.expect_recovery(code, recovery)

        harness.reset_mocks()
        harness.inject(harness.perc_inject, "INVALID_FRAME", {"mode": "OBSERVATION"})
        result = harness.run_strategy("l2_perc_04", timeout_sec=20.0)
        if int(result.error_code) in (8010, 8011, 8012, 9010):
            raise AssertionError(f"L2-PERC-04: invalid frame should not become external perception error, got {result.error_code}")
        if not any(fsm == "WallMappingFSM" and extra.get("invalid_frames", 0) > 0 for fsm, _, extra in harness.active_events):
            raise AssertionError("L2-PERC-04: invalid frame rejection was not observed in mapping extra_json")

        harness.reset_mocks()
        empty_injected = {"done": False}

        def inject_empty_after_mapping(h: L2Harness) -> None:
            if empty_injected["done"]:
                return
            if h.latest_grid_slot_count == 25:
                h.inject(h.perc_inject, "NONE", {"mode": "EMPTY"})
                empty_injected["done"] = True

        result = harness.run_strategy("l2_perc_06", timeout_sec=25.0, during=inject_empty_after_mapping)
        _assert_result(result, False, 3210, "L2-PERC-06")
        harness.expect_recovery(3210, "RETRY_CURRENT_STATE")

        grasp_cases = [
            ("L2-GRASP-02", "vacuum", "LEFT_NEVER_BUILDUP", 5105, "RETRY_CURRENT_STATE"),
            ("L2-GRASP-03", "grasp", "DROP_BOX", 5310, "WAIT_MANUAL_RECOVERY"),
            ("L2-GRASP-05", "grasp", "VACUUM_UNILATERAL", 5106, "SWITCH_TARGET"),
            ("L2-GRASP-06", "grasp", "VACUUM_LOST_DURING_CARRY", 5103, "WAIT_MANUAL_RECOVERY"),
        ]
        for label, target, failure, code, recovery in grasp_cases:
            harness.reset_mocks()
            client = harness.vacuum_inject if target == "vacuum" else harness.grasp_inject
            harness.inject(client, failure)
            harness.spin_for(0.2)
            result = harness.run_strategy(label.lower(), timeout_sec=35.0)
            _assert_result(result, False, code, label)
            harness.expect_recovery(code, recovery)

        harness.reset_mocks()
        cmd_count_before = harness.vacuum_cmd_count
        result = harness.run_strategy("l2_grasp_04", {"dry_run": True}, timeout_sec=60.0)
        _assert_result(result, True, 0, "L2-GRASP-04")
        if harness.vacuum_cmd_count != cmd_count_before:
            raise AssertionError("L2-GRASP-04: dry_run should not publish vacuum command")

        print("M1 strategy L2 failure smoke passed")
        return 0
    except Exception as exc:
        print(f"M1 strategy L2 failure smoke failed: {exc}", file=sys.stderr)
        print(
            "state snapshot: "
            f"wall={harness.wall_state} grid={harness.latest_grid_slot_count} "
            f"pairs={harness.pair_count} recovery={harness.latest_recovery} "
            f"nav_health={harness.nav_health} vacuum_min=({harness.vacuum_min_left},{harness.vacuum_min_right})",
            file=sys.stderr,
        )
        if harness.active_events:
            print(f"last active events: {harness.active_events[-20:]}", file=sys.stderr)
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
