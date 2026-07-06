from __future__ import annotations


class MockSafetyButtonMixin:
    def set_button(self, request, response):
        self._pressed = bool(request.data)
        response.success = True
        response.message = "pressed" if self._pressed else "released"
        self.publish_button()
        return response

    def press(self, request, response):
        del request
        self._pressed = True
        response.success = True
        response.message = "pressed"
        self.publish_button()
        return response

    def release(self, request, response):
        del request
        self._pressed = False
        response.success = True
        response.message = "released"
        self.publish_button()
        return response

    def publish_button(self):
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = self._pressed
        self._button_pub.publish(msg)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_safety_button requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    class MockSafetyButtonNode(SkeletonNodeMixin, MockSafetyButtonMixin, Node):
        def __init__(self):
            super().__init__("mock_safety_button")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="MockSafetyButton")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "MockSafetyButton")
            self._pressed = False
            self._button_pub = self.create_publisher(Bool, get_topic_name(self, "estop_button", "/estop_button"), 10)
            self._set_srv = self.create_service(SetBool, "~/set_pressed", self.set_button)
            from std_srvs.srv import Trigger

            self._press_srv = self.create_service(Trigger, "~/press", self.press)
            self._release_srv = self.create_service(Trigger, "~/release", self.release)
            self._timer = self.create_timer(0.1, self.publish_button)
            self.get_logger().info("mock_safety_button ready")

    rclpy.init(args=args)
    node = MockSafetyButtonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
