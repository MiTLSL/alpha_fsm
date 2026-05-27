from __future__ import annotations

from .adapter import PerceptionAdapterNodeMixin


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("perception_adapter_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener

    class PerceptionAdapterNode(SkeletonNodeMixin, PerceptionAdapterNodeMixin, Node):
        def __init__(self):
            super().__init__("perception_adapter_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="PerceptionAdapter")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "PerceptionAdapter")
            self._detections_pub = self.create_publisher(
                BoxDetectionArray,
                get_topic_name(self, "perception_detections", "/perception/box_detections"),
                5,
            )
            self._health_pub = self.create_publisher(
                PerceptionHealth,
                get_topic_name(self, "perception_health", "/perception/health"),
                make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1),
            )
            self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.init_upstream_adapter()
            if self._upstream_msg_type is not None:
                self.create_upstream_subscription(self._upstream_msg_type)
            self._health_timer = self.create_timer(1.0, self.publish_health)
            self.get_logger().info("perception_adapter_node ready")

    rclpy.init(args=args)
    node = PerceptionAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
