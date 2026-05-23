from __future__ import annotations


class SafetyMonitorNodeMixin:
    def on_button(self, msg):
        self._estop = bool(msg.data)
        self._estop_source = "hardware" if self._estop else ""

    def on_sw_estop(self, request, response):
        self._estop = bool(request.data)
        self._estop_source = "software" if self._estop else ""
        response.success = True
        response.message = "software estop set" if self._estop else "software estop released"
        self.publish_status()
        return response

    def publish_status(self):
        from fsm_msgs.msg import SafetyStatus
        from std_msgs.msg import Bool

        status = SafetyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.estop = self._estop
        status.safety_zone_violated = False
        status.collision_risk = False
        status.communication_ok = True
        status.estop_source = self._estop_source
        status.details_json = "{}"
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
    from fsm_msgs.msg import SafetyStatus
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    class SafetyMonitorNode(SkeletonNodeMixin, SafetyMonitorNodeMixin, Node):
        def __init__(self):
            super().__init__("safety_monitor_node")
            self.init_fsm_node_base(ready_state="NORMAL", heartbeat_fsm_name="SafetyMonitorFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "SafetyMonitorFSM")
            self._estop = False
            self._estop_source = ""
            qos = make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1)
            self._status_pub = self.create_publisher(SafetyStatus, get_topic_name(self, "safety_status", "/safety/status"), qos)
            self._estop_pub = self.create_publisher(Bool, get_topic_name(self, "safety_estop", "/safety/estop"), qos)
            self._twist_mux_estop_pub = self.create_publisher(Bool, get_topic_name(self, "estop", "/estop"), qos)
            self._button_sub = self.create_subscription(Bool, get_topic_name(self, "estop_button", "/estop_button"), self.on_button, 10)
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
