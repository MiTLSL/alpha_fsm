from __future__ import annotations

import json


SAFETY_NORMAL = "NORMAL"
SAFETY_WARNING = "WARNING"
SAFETY_EMERGENCY = "EMERGENCY"


class SafetyMonitorNodeMixin:
    def on_button(self, msg):
        self._hardware_estop = bool(msg.data)
        self._refresh_estop_state()

    def on_sw_estop(self, request, response):
        self._software_estop = bool(request.data)
        self._refresh_estop_state()
        response.success = True
        response.message = "software estop set" if self._software_estop else "software estop released"
        self.publish_status()
        return response

    def on_system_state(self, msg):
        self._last_system_state = msg.current_state
        if self._safety_state == SAFETY_EMERGENCY and not self._estop and msg.current_state in ("SELF_CHECK", "STANDBY"):
            self._set_safety_state(SAFETY_NORMAL)

    def _refresh_estop_state(self) -> None:
        sources = []
        if self._hardware_estop:
            sources.append("hardware")
        if self._software_estop:
            sources.append("software")
        self._estop = bool(sources)
        self._estop_source = ",".join(sources)
        if self._estop:
            self._set_safety_state(SAFETY_EMERGENCY)
        elif self._safety_state != SAFETY_EMERGENCY:
            self._set_safety_state(SAFETY_WARNING if self._warning_active else SAFETY_NORMAL)

    def _set_safety_state(self, state: str) -> None:
        if self._safety_state == state:
            return
        self._safety_state = state
        self._ready_state = state
        self.publish_state_heartbeat()

    def publish_status(self):
        from fsm_msgs.msg import SafetyStatus
        from std_msgs.msg import Bool

        if self._estop and self._safety_state != SAFETY_EMERGENCY:
            self._set_safety_state(SAFETY_EMERGENCY)

        status = SafetyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.estop = self._estop
        status.safety_zone_violated = False
        status.collision_risk = False
        status.communication_ok = True
        status.estop_source = self._estop_source
        status.details_json = json.dumps(
            {
                "safety_state": self._safety_state,
                "hardware_estop": self._hardware_estop,
                "software_estop": self._software_estop,
                "last_system_state": self._last_system_state,
            },
            sort_keys=True,
        )
        self._status_pub.publish(status)

        estop = Bool()
        estop.data = self._estop
        self._estop_pub.publish(estop)

        twist_mux_estop = Bool()
        twist_mux_estop.data = self._estop
        self._twist_mux_estop_pub.publish(twist_mux_estop)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("safety_monitor_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_service_name, get_topic_name, make_qos_profile
    from fsm_msgs.msg import FsmStateSnapshot, SafetyStatus
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    class SafetyMonitorNode(SkeletonNodeMixin, SafetyMonitorNodeMixin, Node):
        def __init__(self):
            super().__init__("safety_monitor_node")
            self.init_fsm_node_base(ready_state="NORMAL", heartbeat_fsm_name="SafetyMonitorFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "SafetyMonitorFSM")
            self._safety_state = SAFETY_NORMAL
            self._estop = False
            self._hardware_estop = False
            self._software_estop = False
            self._estop_source = ""
            self._warning_active = False
            self._last_system_state = ""
            qos = make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1)
            self._status_pub = self.create_publisher(SafetyStatus, get_topic_name(self, "safety_status", "/safety/status"), qos)
            self._estop_pub = self.create_publisher(Bool, get_topic_name(self, "safety_estop", "/safety/estop"), qos)
            self._twist_mux_estop_pub = self.create_publisher(Bool, get_topic_name(self, "estop", "/estop"), qos)
            self._button_sub = self.create_subscription(Bool, get_topic_name(self, "estop_button", "/estop_button"), self.on_button, 10)
            self._system_state_sub = self.create_subscription(
                FsmStateSnapshot,
                get_topic_name(self, "fsm_system_state", "/fsm/system_state"),
                self.on_system_state,
                qos,
            )
            self._sw_estop_srv = self.create_service(SetBool, get_service_name(self, "safety_sw_estop", "/safety/sw_estop"), self.on_sw_estop)
            self._timer = self.create_timer(0.1, self.publish_status)
            self.get_logger().info("safety_monitor_node skeleton ready")

    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
