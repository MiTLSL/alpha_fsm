from __future__ import annotations

import json
from typing import Optional

import rclpy
from fsm_msgs.msg import FsmStateSnapshot
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_msgs.msg import BehaviorState
from std_msgs.msg import String


class StateSummaryNode(Node):
    def __init__(self) -> None:
        super().__init__("state_summary_node")
        self.declare_parameter("system_state_topic", "/fsm/system_state")
        self.declare_parameter("task_state_topic", "/fsm/task_state")
        self.declare_parameter("wall_state_topic", "/fsm/wall_destacking_state")
        self.declare_parameter("active_substate_topic", "/fsm/active_substate")
        self.declare_parameter("behavior_state_topic", "/behavior_state")
        self.declare_parameter("structured_state_topic", "/robot/behavior_state")
        self.declare_parameter("publish_rate_hz", 2.0)

        self._system: Optional[FsmStateSnapshot] = None
        self._task: Optional[FsmStateSnapshot] = None
        self._wall: Optional[FsmStateSnapshot] = None
        self._active: Optional[FsmStateSnapshot] = None

        self.create_subscription(
            FsmStateSnapshot,
            self.get_parameter("system_state_topic").value,
            self._on_system,
            10,
        )
        self.create_subscription(
            FsmStateSnapshot,
            self.get_parameter("task_state_topic").value,
            self._on_task,
            10,
        )
        self.create_subscription(
            FsmStateSnapshot,
            self.get_parameter("wall_state_topic").value,
            self._on_wall,
            10,
        )
        self.create_subscription(
            FsmStateSnapshot,
            self.get_parameter("active_substate_topic").value,
            self._on_active,
            10,
        )

        self._text_pub = self.create_publisher(String, self.get_parameter("behavior_state_topic").value, 10)
        self._state_pub = self.create_publisher(
            BehaviorState,
            self.get_parameter("structured_state_topic").value,
            10,
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("state_summary_node ready")

    def _on_system(self, msg: FsmStateSnapshot) -> None:
        self._system = msg

    def _on_task(self, msg: FsmStateSnapshot) -> None:
        self._task = msg

    def _on_wall(self, msg: FsmStateSnapshot) -> None:
        self._wall = msg

    def _on_active(self, msg: FsmStateSnapshot) -> None:
        self._active = msg

    def _state(self, snapshot: Optional[FsmStateSnapshot]) -> str:
        return snapshot.current_state if snapshot is not None else "UNKNOWN"

    def _task_id(self) -> str:
        for snapshot in (self._task, self._system, self._wall):
            if snapshot is not None and snapshot.task_id:
                return snapshot.task_id
        return ""

    def _last_error(self) -> int:
        for snapshot in (self._task, self._system, self._wall):
            if snapshot is not None and snapshot.last_error_code:
                return int(snapshot.last_error_code)
        return 0

    def _publish(self) -> None:
        summary = {
            "system_state": self._state(self._system),
            "task_state": self._state(self._task),
            "wall_state": self._state(self._wall),
            "active_substate": self._state(self._active),
            "task_id": self._task_id(),
            "last_error_code": self._last_error(),
        }

        text = String()
        text.data = json.dumps(summary, sort_keys=True)
        self._text_pub.publish(text)

        msg = BehaviorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.system_state = summary["system_state"]
        msg.task_state = summary["task_state"]
        msg.wall_state = summary["wall_state"]
        msg.active_substate = summary["active_substate"]
        msg.task_id = summary["task_id"]
        msg.last_error_code = int(summary["last_error_code"])
        msg.summary_json = text.data
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateSummaryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
