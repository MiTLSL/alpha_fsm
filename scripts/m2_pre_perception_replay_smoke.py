#!/usr/bin/env python3
from __future__ import annotations

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


class PerceptionReplayHarness:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth
        from box_perception_msgs.msg import BoxPerceptionResult, BoxResult
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import StaticTransformBroadcaster

        self.rclpy = rclpy
        self.TransformStamped = TransformStamped
        self.BoxPerceptionResult = BoxPerceptionResult
        self.BoxResult = BoxResult
        self.node = Node("m2_pre_perception_replay_harness")

        detection_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        health_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.latest_detections: BoxDetectionArray | None = None
        self.latest_health: PerceptionHealth | None = None
        self.detection_count = 0
        self.health_count = 0

        self.node.create_subscription(BoxDetectionArray, "/perception/box_detections", self._on_detections, detection_qos)
        self.node.create_subscription(PerceptionHealth, "/perception/health", self._on_health, health_qos)
        self.publisher = self.node.create_publisher(BoxPerceptionResult, "/box_perception/result", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)

    def _on_detections(self, msg):
        self.latest_detections = msg
        self.detection_count += 1

    def _on_health(self, msg):
        self.latest_health = msg
        self.health_count += 1

    def publish_static_tf(self) -> None:
        tf = self.TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "body"
        tf.transform.translation.x = 0.123
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def make_box(self, box_id: int, center, normal, confidence: float = 0.91):
        box = self.BoxResult()
        box.header.stamp = self.node.get_clock().now().to_msg()
        box.header.frame_id = "body"
        box.box_id = int(box_id)
        box.confidence = float(confidence)
        box.nearest_face_center.x = float(center[0])
        box.nearest_face_center.y = float(center[1])
        box.nearest_face_center.z = float(center[2])
        box.nearest_face_normal.x = float(normal[0])
        box.nearest_face_normal.y = float(normal[1])
        box.nearest_face_normal.z = float(normal[2])
        box.face_normal_0.x = float(normal[0])
        box.face_normal_0.y = float(normal[1])
        box.face_normal_0.z = float(normal[2])
        box.face_normal_1.x = 0.0
        box.face_normal_1.y = 0.0
        box.face_normal_1.z = -1.0
        box.face_inlier_count_0 = 25
        box.face_inlier_count_1 = 18
        box.bbox.x_offset = 12
        box.bbox.y_offset = 8
        box.bbox.width = 48
        box.bbox.height = 36
        return box

    def publish_sample(self) -> None:
        msg = self.BoxPerceptionResult()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.frame_id = 42
        msg.boxes = [
            self.make_box(1, (0.6, 0.20, 0.90), (1.0, 0.0, 0.0)),
            self.make_box(2, (0.6, -0.10, 1.10), (1.0, 0.0, 0.0), confidence=0.83),
        ]
        self.publisher.publish(msg)

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def destroy(self) -> None:
        self.node.destroy_node()


def _start_adapter() -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run perception_adapter perception_adapter_node"
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

    proc = _start_adapter()
    output: list[str] = []
    selector = selectors.DefaultSelector()
    if proc.stdout:
        selector.register(proc.stdout, selectors.EVENT_READ)

    rclpy.init()
    harness = PerceptionReplayHarness()
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            _drain_output(selector, output)
            if proc.poll() is not None:
                raise RuntimeError("perception_adapter exited before ready")
            harness.publish_static_tf()
            rclpy.spin_once(harness.node, timeout_sec=0.05)
            if harness.health_count > 0:
                break
        else:
            raise RuntimeError("perception_adapter start timeout")

        harness.spin_until(lambda: harness.health_count > 0, 3.0, "initial health")
        harness.publish_sample()
        harness.spin_until(
            lambda: harness.latest_detections is not None and harness.latest_detections.detections and harness.latest_health is not None,
            5.0,
            "detection and health",
        )
        harness.spin_until(lambda: harness.latest_health.error_code == 0, 3.0, "healthy perception")

        detections = harness.latest_detections
        health = harness.latest_health
        assert detections is not None
        assert health is not None
        if detections.header.frame_id != "base_link":
            raise AssertionError(f"unexpected detection frame: {detections.header.frame_id}")
        if len(detections.detections) != 2:
            raise AssertionError(f"unexpected detection count: {len(detections.detections)}")
        first = detections.detections[0]
        second = detections.detections[1]
        if not first.pose_valid or not second.pose_valid:
            raise AssertionError("pose_valid should be true for replay sample")
        if abs(first.pose.pose.position.x - 0.523) > 0.02:
            raise AssertionError(f"unexpected first box x: {first.pose.pose.position.x}")
        if abs(first.pose.pose.position.y - 0.20) > 0.02:
            raise AssertionError(f"unexpected first box y: {first.pose.pose.position.y}")
        if abs(second.pose.pose.position.x - 0.523) > 0.02:
            raise AssertionError(f"unexpected second box x: {second.pose.pose.position.x}")
        if health.error_code != 0:
            raise AssertionError(f"health should be ok, got {health.error_code}")
        if "m2_preintegration_adapter" not in health.details_json:
            raise AssertionError("health details missing mode tag")
        if "has_box_perception_msgs" not in health.details_json:
            raise AssertionError("health details missing import info")

        print("M2 perception replay smoke passed")
        return 0
    except Exception as exc:
        print(f"M2 perception replay smoke failed: {exc}", file=sys.stderr)
        print(
            f"detections={harness.detection_count} health={harness.health_count} "
            f"latest_health={getattr(harness.latest_health, 'details_json', '')}",
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
