from __future__ import annotations


def main(args=None):
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_chassis_status_publisher requires ROS2 rclpy") from exc

    class MockChassisStatusPublisher(Node):
        def __init__(self):
            super().__init__("mock_chassis_status_publisher")
            self.enabled = bool(self.declare_parameter("enabled", True).value)
            self.fault = bool(self.declare_parameter("fault", False).value)
            self.heartbeat_ok = bool(self.declare_parameter("heartbeat_ok", True).value)
            self.status_topic = str(self.declare_parameter("status_topic", "/chassis_node/status").value)
            self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
            self.pub = self.create_publisher(DiagnosticArray, self.status_topic, 10)
            self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self.publish_status)
            self.get_logger().info(f"mock_chassis_status_publisher ready topic={self.status_topic}")

        def publish_status(self):
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "chassis"
            status.level = DiagnosticStatus.ERROR if self.fault else DiagnosticStatus.OK
            status.message = "fault" if self.fault else "ok"
            status.values = [
                KeyValue(key="enabled", value=str(self.enabled).lower()),
                KeyValue(key="heartbeat_ok", value=str(self.heartbeat_ok).lower()),
                KeyValue(key="fault", value=str(self.fault).lower()),
            ]
            msg.status.append(status)
            self.pub.publish(msg)

    rclpy.init(args=args)
    node = MockChassisStatusPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
