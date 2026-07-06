from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    from isaac_sim_bridge.alfa_command import command_json, default_alfa_command
except ImportError:
    def default_alfa_command() -> dict[str, Any]:
        return {
            "base": {"linear": 0.0, "yaw": 0.0},
            "turn_joint": 0.0,
            "updown": 0.0,
            "warehouse_door": None,
            "container_door": None,
            "arm": {
                "left": {f"joint{i}": 0.0 for i in range(1, 7)},
                "right": {f"joint{i}": 0.0 for i in range(1, 7)},
            },
            "suction": {"left": "open", "right": "open"},
            "reset": False,
        }

    def command_json(command: dict[str, Any]) -> str:
        return json.dumps(command, sort_keys=True, ensure_ascii=False, allow_nan=False)


def _clip(value: float, limit: float = 1.0) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


class CmdVelToAlfaNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_alfa_node")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/alfa/command_json")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("open_doors", True)

        self._last_twist = Twist()
        self._last_receive_time = 0.0
        self._last_sent_signature = ""

        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self._on_cmd_vel,
            10,
        )
        self._pub = self.create_publisher(String, self.get_parameter("command_topic").value, 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"cmd_vel_to_alfa_node bridging {self.get_parameter('cmd_vel_topic').value} "
            f"-> {self.get_parameter('command_topic').value}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_receive_time = time.monotonic()

    def _publish(self) -> None:
        timeout = max(0.0, float(self.get_parameter("command_timeout_sec").value))
        active = self._last_receive_time > 0.0 and (time.monotonic() - self._last_receive_time) <= timeout
        linear_scale = float(self.get_parameter("linear_scale").value)
        yaw_scale = float(self.get_parameter("yaw_scale").value)

        linear = _clip(float(self._last_twist.linear.x) * linear_scale) if active else 0.0
        yaw = _clip(float(self._last_twist.angular.z) * yaw_scale) if active else 0.0

        command = default_alfa_command()
        command["base"] = {"linear": linear, "yaw": yaw}
        if bool(self.get_parameter("open_doors").value):
            command["warehouse_door"] = "open"
            command["container_door"] = "open"
        command["meta"] = {
            "source": "wsl_ros2",
            "kind": "cmd_vel",
            "active": active,
            "cmd_vel": {
                "linear_x": float(self._last_twist.linear.x),
                "angular_z": float(self._last_twist.angular.z),
            },
        }

        payload = command_json(command)
        signature = json.dumps(command["base"], sort_keys=True) + str(active)
        if signature != self._last_sent_signature or active:
            msg = String()
            msg.data = payload
            self._pub.publish(msg)
            self._last_sent_signature = signature


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToAlfaNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
