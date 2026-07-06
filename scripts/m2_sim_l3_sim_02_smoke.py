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


class L3Sim02Harness:
    def __init__(self):
        import rclpy
        from fsm_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped, Twist
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray

        self.rclpy = rclpy
        self.NavigateToPose = NavigateToPose
        self.PoseStamped = PoseStamped
        self.node = Node("m2_sim_l3_sim_02_harness")
        self.nav_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.feedback_states: list[str] = []
        self.feedback_errors: list[float] = []
        self.cmd_vel_samples: list[tuple[float, float]] = []
        self.latest_alignment: list[float] = []
        self.node.create_subscription(Twist, "/cmd_vel_align", self._on_cmd_vel, 10)
        self.node.create_subscription(Float32MultiArray, "/sim/fake_base_alignment", self._on_alignment, 10)

    def _on_cmd_vel(self, msg):
        self.cmd_vel_samples.append((float(msg.linear.x), float(msg.angular.z)))

    def _on_alignment(self, msg):
        self.latest_alignment = [float(value) for value in msg.data]

    def _on_feedback(self, msg):
        feedback = msg.feedback
        self.feedback_states.append(str(feedback.current_state))
        value = float(feedback.alignment_error_current)
        if value == value:
            self.feedback_errors.append(value)

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + float(duration_sec)
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def call_fine_align_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/navigate_to_pose unavailable")
        goal = self.NavigateToPose.Goal()
        goal.goal_type = "LEFT_PHASE"
        goal.target_pose = self.make_pose(1.0, 0.0, 0.0)
        goal.wall_frame_pose = self.make_pose(0.6, 0.0, 0.8)
        goal.phase = 1
        goal.desired_distance_to_wall = 0.60
        goal.desired_yaw_to_wall = 0.0
        goal.desired_lateral_offset = 0.0
        goal.require_fine_alignment = True
        goal.timeout_sec = 8.0
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        self.rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("navigation_manager rejected L3-SIM-02 goal")
        result_future = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=12.0)
        if not result_future.done():
            raise RuntimeError("navigation_manager L3-SIM-02 result timeout")
        return result_future.result().result

    def make_pose(self, x: float, y: float, z: float):
        pose = self.PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def destroy(self) -> None:
        self.node.destroy_node()


def _start_launch() -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch fsm_config bringup_with_kinematic_sim.launch.py "
        "start_core:=false mock_nav:=false use_fake_nav2:=true "
        "mock_grasp:=true mock_vacuum:=true "
        "sim_perception_output:=adapter_input use_real_perception_adapter:=true"
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
    harness = L3Sim02Harness()
    try:
        harness.spin_until(lambda: harness.nav_client.wait_for_server(timeout_sec=0.05), 15.0, "navigation action server")
        harness.spin_until(lambda: len(harness.latest_alignment) >= 4, 5.0, "fake base alignment state")
        harness.spin_for(0.75)
        initial_dist_error = abs(harness.latest_alignment[2])
        initial_yaw_error = abs(harness.latest_alignment[3])
        result = harness.call_fine_align_goal()
        if not result.success or result.error_code != 0:
            raise AssertionError(f"expected fine align success, got success={result.success} code={result.error_code}")
        if "FINE_ALIGN" not in harness.feedback_states:
            raise AssertionError(f"missing FINE_ALIGN feedback: {harness.feedback_states}")
        if not any(abs(linear) > 1e-4 or abs(angular) > 1e-4 for linear, angular in harness.cmd_vel_samples):
            raise AssertionError("navigation_manager did not publish non-zero /cmd_vel_align")
        if result.alignment_error > 0.06:
            raise AssertionError(f"final alignment_error too high: {result.alignment_error}")
        if not harness.feedback_errors or harness.feedback_errors[-1] > harness.feedback_errors[0]:
            raise AssertionError(f"alignment feedback did not improve: {harness.feedback_errors}")
        final_dist_error = abs(harness.latest_alignment[2]) if len(harness.latest_alignment) >= 4 else float("inf")
        final_yaw_error = abs(harness.latest_alignment[3]) if len(harness.latest_alignment) >= 4 else float("inf")
        if final_dist_error > initial_dist_error or final_yaw_error > initial_yaw_error:
            raise AssertionError(
                f"fake base state did not converge: initial=({initial_dist_error},{initial_yaw_error}) "
                f"final=({final_dist_error},{final_yaw_error})"
            )
        print("M2 L3-SIM-02 smoke passed")
        return 0
    except Exception as exc:
        _drain_output(selector, output)
        print(f"M2 L3-SIM-02 smoke failed: {exc}", file=sys.stderr)
        print(
            "state: "
            f"feedback={harness.feedback_states[-8:]} "
            f"errors={harness.feedback_errors[:3]}->{harness.feedback_errors[-3:]} "
            f"alignment={harness.latest_alignment} "
            f"cmd_samples={len(harness.cmd_vel_samples)}",
            file=sys.stderr,
        )
        if output:
            print("".join(output[-180:]), file=sys.stderr)
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
