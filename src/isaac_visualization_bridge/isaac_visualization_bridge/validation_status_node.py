from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from fsm_msgs.msg import FsmStateSnapshot
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ValidationStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("validation_status_node")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("status_topic", "/alfa/validation_status")
        self.declare_parameter("marker_topic", "/visualization_marker_array")

        self._clock_seen_at = 0.0
        self._command_seen_at = 0.0
        self._event_seen_at = 0.0
        self._windows_state_seen_at = 0.0
        self._last_command = ""
        self._last_event = ""
        self._last_windows_state = ""
        self._system_state: Optional[FsmStateSnapshot] = None
        self._task_state: Optional[FsmStateSnapshot] = None
        self._behavior_state = ""

        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/alfa/command_json", self._on_command, 10)
        self.create_subscription(String, "/alfa/fsm_event_json", self._on_event, 10)
        self.create_subscription(String, "/alfa/state_json", self._on_windows_state, 10)
        self.create_subscription(String, "/behavior_state", self._on_behavior, 10)
        self.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._on_system, 10)
        self.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._on_task, 10)

        self._status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self._marker_pub = self.create_publisher(MarkerArray, self.get_parameter("marker_topic").value, 10)

        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("validation_status_node ready")

    def _on_clock(self, msg: Clock) -> None:
        del msg
        self._clock_seen_at = time.monotonic()

    def _on_command(self, msg: String) -> None:
        self._command_seen_at = time.monotonic()
        self._last_command = msg.data

    def _on_event(self, msg: String) -> None:
        self._event_seen_at = time.monotonic()
        self._last_event = msg.data

    def _on_windows_state(self, msg: String) -> None:
        self._windows_state_seen_at = time.monotonic()
        self._last_windows_state = msg.data

    def _on_behavior(self, msg: String) -> None:
        self._behavior_state = msg.data

    def _on_system(self, msg: FsmStateSnapshot) -> None:
        self._system_state = msg

    def _on_task(self, msg: FsmStateSnapshot) -> None:
        self._task_state = msg

    def _fresh(self, timestamp: float, max_age: float = 3.0) -> bool:
        return timestamp > 0.0 and (time.monotonic() - timestamp) <= max_age

    def _snapshot_state(self, snapshot: Optional[FsmStateSnapshot]) -> str:
        return snapshot.current_state if snapshot is not None else "UNKNOWN"

    def _publish(self) -> None:
        status = {
            "clock_seen": self._fresh(self._clock_seen_at),
            "command_seen": self._fresh(self._command_seen_at),
            "fsm_event_seen": self._fresh(self._event_seen_at),
            "windows_state_seen": self._fresh(self._windows_state_seen_at),
            "system_state": self._snapshot_state(self._system_state),
            "task_state": self._snapshot_state(self._task_state),
            "last_command_preview": self._last_command[:240],
            "last_event_preview": self._last_event[:240],
            "last_windows_state_preview": self._last_windows_state[:240],
        }

        msg = String()
        msg.data = json.dumps(status, sort_keys=True, ensure_ascii=False)
        self._status_pub.publish(msg)
        self._marker_pub.publish(self._make_markers(status))

    def _make_markers(self, status: dict) -> MarkerArray:
        lines = [
            f"clock: {'OK' if status['clock_seen'] else 'WAIT'}",
            f"command: {'OK' if status['command_seen'] else 'WAIT'}",
            f"fsm_event: {'OK' if status['fsm_event_seen'] else 'WAIT'}",
            f"windows_state: {'OK' if status['windows_state_seen'] else 'WAIT'}",
            f"system: {status['system_state']}",
            f"task: {status['task_state']}",
        ]
        if self._behavior_state:
            lines.append(f"summary: {self._behavior_state[:80]}")

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter("frame_id").value)
        marker.ns = "alfa_validation"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.5
        marker.pose.position.z = 1.2
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 0.1
        marker.color.g = 0.9 if status["clock_seen"] and status["command_seen"] else 0.55
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = "\n".join(lines)
        return MarkerArray(markers=[marker])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidationStatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
