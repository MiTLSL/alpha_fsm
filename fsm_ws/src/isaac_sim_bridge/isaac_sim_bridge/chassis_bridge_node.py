from __future__ import annotations

import json

from .config import load_sim_parameters


def main(args=None):
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("isaac_chassis_bridge_node requires ROS2 rclpy") from exc

    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_service_name, get_topic_name
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    class IsaacChassisBridgeNode(SkeletonNodeMixin, Node):
        def __init__(self):
            super().__init__("isaac_chassis_bridge_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="IsaacChassisBridge")
            load_sim_parameters(self)
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "IsaacChassisBridge")
            self._enabled = bool(self.config.get("sim.isaac.chassis.initial_enabled", True))
            self._fault = bool(self.config.get("sim.isaac.chassis.initial_fault", False))
            self._heartbeat_ok = True
            self._last_estop = False
            self._status_rate_hz = float(self.config.get("sim.isaac.chassis.status_publish_rate_hz", 10.0))
            self._status_pub = self.create_publisher(
                DiagnosticArray,
                get_topic_name(self, "chassis_status", "/chassis_node/status"),
                10,
            )
            self._estop_sub = self.create_subscription(
                Bool,
                get_topic_name(self, "estop", "/estop"),
                self._on_estop_cmd,
                10,
            )
            self._safety_estop_pub = self.create_publisher(
                Bool,
                get_topic_name(self, "safety_estop", "/safety/estop"),
                10,
            )
            self._reset_srv = self.create_service(
                Trigger,
                get_service_name(self, "chassis_reset_fault", "/chassis_node/reset_fault"),
                self._handle_reset_fault,
            )
            self._enable_srv = self.create_service(
                Trigger,
                get_service_name(self, "chassis_enable", "/chassis_node/enable"),
                self._handle_enable,
            )
            self._timer = self.create_timer(1.0 / max(self._status_rate_hz, 0.1), self._publish_status)
            self.get_logger().info("isaac_chassis_bridge_node ready")

        def _on_estop_cmd(self, msg):
            self._last_estop = bool(msg.data)
            if self._last_estop:
                self._enabled = False
                self._ready_state = "ESTOP"
            else:
                self._ready_state = "READY" if not self._fault else "FAULT"
            safety = Bool()
            safety.data = self._last_estop
            self._safety_estop_pub.publish(safety)
            self.publish_state_heartbeat()

        def _handle_reset_fault(self, request, response):
            del request
            self._fault = False
            self._ready_state = "READY" if not self._last_estop else "ESTOP"
            response.success = True
            response.message = "isaac chassis fault reset"
            return response

        def _handle_enable(self, request, response):
            del request
            if self._last_estop:
                response.success = False
                response.message = "cannot enable while estop is active"
                return response
            if self._fault:
                response.success = False
                response.message = "cannot enable while chassis fault is active"
                return response
            self._enabled = True
            self._ready_state = "READY"
            response.success = True
            response.message = "isaac chassis enabled"
            return response

        def _publish_status(self):
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "isaac_chassis"
            status.hardware_id = "isaac_sim"
            if self._fault:
                status.level = DiagnosticStatus.ERROR
                status.message = "fault"
            elif self._last_estop:
                status.level = DiagnosticStatus.WARN
                status.message = "estop"
            elif not self._enabled:
                status.level = DiagnosticStatus.WARN
                status.message = "disabled"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "ok"
            status.values = [
                KeyValue(key="enabled", value=str(self._enabled).lower()),
                KeyValue(key="heartbeat_ok", value=str(self._heartbeat_ok).lower()),
                KeyValue(key="fault", value=str(self._fault).lower()),
                KeyValue(key="estop", value=str(self._last_estop).lower()),
                KeyValue(
                    key="details_json",
                    value=json.dumps({"mode": "isaac_chassis_bridge", "requires_isaac_sdk": False}, sort_keys=True),
                ),
            ]
            msg.status.append(status)
            self._status_pub.publish(msg)

    rclpy.init(args=args)
    node = IsaacChassisBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
