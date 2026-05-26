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
        env["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin"
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"/usr/bin/python3 {Path(__file__).resolve()} --ros-python"
        )
        raise SystemExit(subprocess.call(["/bin/bash", "-lc", command], cwd=ROOT, env=env))


def _start_grasp_node(failure_mode: str = "NONE", stage_delay_sec: float = 0.03) -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run pair_grasp_execution pair_grasp_execution_node --ros-args "
        "-p business.pair_grasp_execution.backend_mode:=fake_real "
        f"-p business.pair_grasp_execution.fake_failure_mode:={failure_mode} "
        f"-p business.pair_grasp_execution.stage_delay_sec:={stage_delay_sec}"
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


class GraspFakeRealHarness:
    def __init__(self):
        import rclpy
        from fsm_msgs.action import ExecutePairGrasp
        from fsm_msgs.msg import GraspPair
        from geometry_msgs.msg import PoseStamped, Vector3
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray

        self.rclpy = rclpy
        self.ExecutePairGrasp = ExecutePairGrasp
        self.GraspPair = GraspPair
        self.PoseStamped = PoseStamped
        self.Vector3 = Vector3
        self.Float32MultiArray = Float32MultiArray
        self.node = Node("m2_pre_grasp_fake_real_harness")
        self.client = ActionClient(self.node, ExecutePairGrasp, "/execute_pair_grasp")
        self.pressure_pub = self.node.create_publisher(Float32MultiArray, "/vacuum/pressure_raw", 10)
        self.feedback_states: list[str] = []

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            self.publish_pressure()
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            self.publish_pressure()

    def publish_pressure(self) -> None:
        msg = self.Float32MultiArray()
        msg.data = [-60.0, -60.0]
        self.pressure_pub.publish(msg)

    def call_goal(self, dry_run: bool = True, cancel_after_sec: float | None = None, invalid_pair: bool = False):
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("pair_grasp_execution /execute_pair_grasp unavailable")
        self.feedback_states = []
        goal = self.ExecutePairGrasp.Goal()
        goal.grasp_pair = self.make_pair(invalid=invalid_pair)
        goal.timeout_sec = 5.0
        goal.dry_run = bool(dry_run)
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        self.rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None

        if cancel_after_sec is not None:
            self.spin_for(float(cancel_after_sec))
            cancel_future = handle.cancel_goal_async()
            self.rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=3.0)

        result_future = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=8.0)
        if not result_future.done():
            raise RuntimeError("pair grasp result timeout")
        return result_future.result().result

    def _on_feedback(self, feedback_msg) -> None:
        self.feedback_states.append(str(feedback_msg.feedback.current_state))

    def make_pair(self, invalid: bool = False):
        pair = self.GraspPair()
        pair.pair_id = "" if invalid else "pair_m2_pre_001"
        pair.task_id = "task_m2_pre"
        pair.wall_index = 0
        pair.phase = pair.PHASE_LEFT
        pair.left_slot_id = "r0_c0"
        pair.right_slot_id = "r0_c1"
        pair.left_box_pose_robot = self.make_pose(0.55, 0.24, 0.85)
        pair.right_box_pose_robot = self.make_pose(0.55, -0.24, 0.85)
        pair.left_box_size = self.make_size()
        pair.right_box_size = self.make_size()
        pair.fixed_place_pose_robot = self.make_pose(0.50, 0.0, 0.80)
        pair.grasp_mode = pair.MODE_DUAL
        return pair

    def make_pose(self, x: float, y: float, z: float):
        pose = self.PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def make_size(self):
        size = self.Vector3()
        size.x = 0.4
        size.y = 0.4
        size.z = 0.4
        return size

    def destroy(self):
        self.node.destroy_node()


def _drain_output(selector: selectors.DefaultSelector, output: list[str]) -> None:
    for key, _ in selector.select(timeout=0.01):
        line = key.fileobj.readline()
        if line:
            output.append(line)


def _stop_proc(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5.0)


def _run_case(harness: GraspFakeRealHarness, failure_mode: str, expected_code: int, expected_success: bool) -> None:
    proc = _start_grasp_node(failure_mode)
    output: list[str] = []
    selector = selectors.DefaultSelector()
    if proc.stdout:
        selector.register(proc.stdout, selectors.EVENT_READ)
    try:
        harness.spin_until(lambda: harness.client.wait_for_server(timeout_sec=0.05), 10.0, f"grasp action server {failure_mode}")
        result = harness.call_goal(dry_run=True)
        if result is None:
            raise AssertionError(f"{failure_mode}: goal was rejected")
        if result.success != expected_success or result.result.error_code != expected_code:
            raise AssertionError(
                f"{failure_mode}: expected success={expected_success} code={expected_code}, "
                f"got success={result.success} code={result.result.error_code}"
            )
        if expected_success and "REPORT" not in harness.feedback_states:
            raise AssertionError("success case did not reach REPORT feedback")
    except Exception:
        _drain_output(selector, output)
        if output:
            print("".join(output[-120:]), file=sys.stderr)
        raise
    finally:
        _stop_proc(proc)
        harness.spin_for(0.2)


def _run_cancel_case(harness: GraspFakeRealHarness) -> None:
    proc = _start_grasp_node("NONE", stage_delay_sec=0.15)
    try:
        harness.spin_until(lambda: harness.client.wait_for_server(timeout_sec=0.05), 10.0, "grasp action server cancel")
        result = harness.call_goal(dry_run=True, cancel_after_sec=0.2)
        if result is None:
            raise AssertionError("cancel case goal rejected")
        if result.result.result_code != result.result.CANCELLED:
            raise AssertionError(f"expected CANCELLED result, got {result.result.result_code}")
    finally:
        _stop_proc(proc)
        harness.spin_for(0.2)


def _run_invalid_pair_case(harness: GraspFakeRealHarness) -> None:
    proc = _start_grasp_node("NONE")
    try:
        harness.spin_until(lambda: harness.client.wait_for_server(timeout_sec=0.05), 10.0, "grasp action server invalid pair")
        result = harness.call_goal(dry_run=True, invalid_pair=True)
        if result is None:
            raise AssertionError("invalid pair goal rejected before structured result")
        if result.success or result.result.error_code != 5010:
            raise AssertionError(f"expected invalid pair code 5010, got {result.result.error_code}")
    finally:
        _stop_proc(proc)
        harness.spin_for(0.2)


def _run() -> int:
    _ensure_ros_python()
    import rclpy
    from fsm_core.error_code import ErrorCode

    rclpy.init()
    harness = GraspFakeRealHarness()
    try:
        _run_case(harness, "NONE", 0, True)
        _run_case(harness, "IK_FAIL", int(ErrorCode.E_PLAN_IK_FAIL), False)
        _run_case(harness, "TRAJ_FAIL", int(ErrorCode.E_PLAN_TRAJ_FAIL), False)
        _run_case(harness, "COLLISION", int(ErrorCode.E_PLAN_COLLISION_DETECTED), False)
        _run_cancel_case(harness)
        _run_invalid_pair_case(harness)
        print("M2 grasp fake-real smoke passed")
        return 0
    except Exception as exc:
        print(f"M2 grasp fake-real smoke failed: {exc}", file=sys.stderr)
        return 1
    finally:
        harness.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    if "--ros-python" in sys.argv:
        sys.argv.remove("--ros-python")
    raise SystemExit(_run())
