from __future__ import annotations

from .common import FailureInjectionMixin, VacuumPressureModel


class MockVacuumIoMixin(FailureInjectionMixin):
    def on_cmd(self, msg):
        self._left_on = bool(msg.left_on)
        self._right_on = bool(msg.right_on)
        self._last_command_source = int(msg.command_source)
        self._left_model.set_enabled(self._left_on)
        self._right_model.set_enabled(self._right_on)

    def publish_pressure(self):
        from std_msgs.msg import Bool, Float32MultiArray

        health = Bool()
        health.data = self._current_failure != "SENSOR_OFFLINE"
        self._health_pub.publish(health)
        if not health.data:
            return

        self._left_model.release_time_ms = 2000.0 if self._current_failure == "RELEASE_TOO_SLOW" else self._release_time_ms
        self._right_model.release_time_ms = 2000.0 if self._current_failure == "RELEASE_TOO_SLOW" else self._release_time_ms
        left_failure = "NONE"
        right_failure = "NONE"
        if self._current_failure == "LEFT_NEVER_BUILDUP":
            left_failure = "NEVER_BUILDUP"
        elif self._current_failure == "RIGHT_NEVER_BUILDUP":
            right_failure = "NEVER_BUILDUP"
        elif self._current_failure == "LEAK_AFTER_ATTACH":
            left_failure = right_failure = "LEAK_AFTER_ATTACH"

        msg = Float32MultiArray()
        msg.data = [float(self._left_model.sample(left_failure)), float(self._right_model.sample(right_failure))]
        self._pressure_pub.publish(msg)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_vacuum_io_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_msgs.msg import VacuumCommand
    from std_msgs.msg import Bool, Float32MultiArray

    class MockVacuumIoNode(SkeletonNodeMixin, MockVacuumIoMixin, Node):
        def __init__(self):
            super().__init__("mock_vacuum_io_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="MockVacuumIO")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "MockVacuumIO")
            self.init_failure_injection()
            self._left_on = False
            self._right_on = False
            self._last_command_source = -1
            buildup_ms = float(self.config.get("business.vacuum.mock_buildup_time_ms", 150.0))
            self._release_time_ms = float(self.config.get("business.vacuum.mock_release_time_ms", 100.0))
            self._left_model = VacuumPressureModel(self.get_clock(), buildup_ms, self._release_time_ms)
            self._right_model = VacuumPressureModel(self.get_clock(), buildup_ms, self._release_time_ms)
            self._cmd_sub = self.create_subscription(VacuumCommand, get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"), self.on_cmd, 10)
            self._pressure_pub = self.create_publisher(Float32MultiArray, get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"), 10)
            self._health_pub = self.create_publisher(Bool, get_topic_name(self, "vacuum_health", "/vacuum/health"), make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1))
            self._inject_srv = self.create_inject_failure_service()
            self._timer = self.create_timer(0.05, self.publish_pressure)
            self.get_logger().info("mock_vacuum_io_node ready")

    rclpy.init(args=args)
    node = MockVacuumIoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
