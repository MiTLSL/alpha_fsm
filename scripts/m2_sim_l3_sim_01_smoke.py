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


class L3Sim01Harness:
    def __init__(self):
        import rclpy
        from box_perception_msgs.msg import BoxPerceptionResult
        from fsm_msgs.msg import BoxDetectionArray
        from rclpy.node import Node
        from visualization_msgs.msg import MarkerArray

        self.rclpy = rclpy
        self.node = Node("m2_sim_l3_sim_01_harness")
        self.latest_upstream: BoxPerceptionResult | None = None
        self.latest_detections: BoxDetectionArray | None = None
        self.latest_markers: MarkerArray | None = None
        self.node.create_subscription(BoxPerceptionResult, "/box_perception/result", self._on_upstream, 10)
        self.node.create_subscription(BoxDetectionArray, "/perception/box_detections", self._on_detections, 10)
        self.node.create_subscription(MarkerArray, "/sim/markers", self._on_markers, 10)

    def _on_upstream(self, msg):
        self.latest_upstream = msg

    def _on_detections(self, msg):
        self.latest_detections = msg

    def _on_markers(self, msg):
        self.latest_markers = msg

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def ready(self) -> bool:
        return (
            self.latest_upstream is not None
            and len(self.latest_upstream.boxes) == 25
            and self.latest_detections is not None
            and len(self.latest_detections.detections) == 25
            and self.latest_detections.header.frame_id == "base_link"
            and self.latest_markers is not None
            and len(self.latest_markers.markers) >= 25
        )

    def assert_ready(self) -> None:
        if self.latest_upstream is None:
            raise AssertionError("no /box_perception/result received")
        if len(self.latest_upstream.boxes) != 25:
            raise AssertionError(f"unexpected upstream box count: {len(self.latest_upstream.boxes)}")
        if self.latest_detections is None:
            raise AssertionError("no /perception/box_detections received")
        if len(self.latest_detections.detections) != 25:
            raise AssertionError(f"unexpected detection count: {len(self.latest_detections.detections)}")
        if self.latest_detections.header.frame_id != "base_link":
            raise AssertionError(f"unexpected detection frame: {self.latest_detections.header.frame_id}")
        if self.latest_markers is None or len(self.latest_markers.markers) < 25:
            count = 0 if self.latest_markers is None else len(self.latest_markers.markers)
            raise AssertionError(f"unexpected marker count: {count}")
        if not all(det.pose_valid for det in self.latest_detections.detections):
            raise AssertionError("all sim detections should be pose_valid")

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
        "start_core:=false mock_nav:=false mock_grasp:=false mock_vacuum:=false "
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
    harness = L3Sim01Harness()
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            _drain_output(selector, output)
            if proc.poll() is not None:
                raise RuntimeError("kinematic sim launch exited before L3-SIM-01 was ready")
            harness.spin_once()
            if harness.ready():
                harness.assert_ready()
                print("M2 L3-SIM-01 smoke passed")
                return 0
        harness.assert_ready()
        print("M2 L3-SIM-01 smoke passed")
        return 0
    except Exception as exc:
        print(f"M2 L3-SIM-01 smoke failed: {exc}", file=sys.stderr)
        print(
            "state: "
            f"upstream={0 if harness.latest_upstream is None else len(harness.latest_upstream.boxes)} "
            f"detections={0 if harness.latest_detections is None else len(harness.latest_detections.detections)} "
            f"markers={0 if harness.latest_markers is None else len(harness.latest_markers.markers)}",
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
