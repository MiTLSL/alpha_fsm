from __future__ import annotations


class VacuumIoNodeMixin:
    def on_cmd(self, msg):
        self._left_on = bool(msg.left_on)
        self._right_on = bool(msg.right_on)
        self._left_model.set_enabled(self._left_on)
        self._right_model.set_enabled(self._right_on)

    def publish_pressure(self):
        from std_msgs.msg import Bool, Float32MultiArray

        pressure = Float32MultiArray()
        pressure.data = [float(self._left_model.sample()), float(self._right_model.sample())]
        self._pressure_pub.publish(pressure)

        health = Bool()
        health.data = True
        self._health_pub.publish(health)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("vacuum_io_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_core.vacuum_model import VacuumPressureModel
    from fsm_msgs.msg import VacuumCommand
    from std_msgs.msg import Bool, Float32MultiArray

    class VacuumIoNode(SkeletonNodeMixin, VacuumIoNodeMixin, Node):
        def __init__(self):
            super().__init__("vacuum_io_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="VacuumIO")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "VacuumIO")
            self._left_on = False
            self._right_on = False
            buildup_ms = float(self.config.get("business.vacuum.mock_buildup_time_ms", 150.0))
            release_ms = float(self.config.get("business.vacuum.mock_release_time_ms", 100.0))
            self._left_model = VacuumPressureModel(self.get_clock(), buildup_ms, release_ms)
            self._right_model = VacuumPressureModel(self.get_clock(), buildup_ms, release_ms)
            self._cmd_sub = self.create_subscription(
                VacuumCommand,
                get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"),
                self.on_cmd,
                10,
            )
            self._pressure_pub = self.create_publisher(
                Float32MultiArray,
                get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"),
                10,
            )
            self._health_pub = self.create_publisher(
                Bool,
                get_topic_name(self, "vacuum_health", "/vacuum/health"),
                make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1),
            )
            rate = float(self.config.get("business.vacuum.pressure_publish_rate_hz", 20.0))
            self._timer = self.create_timer(1.0 / max(rate, 1.0), self.publish_pressure)
            self.get_logger().info("vacuum_io_node skeleton ready")

    rclpy.init(args=args)
    node = VacuumIoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
