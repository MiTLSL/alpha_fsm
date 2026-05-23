from __future__ import annotations


class PerceptionAdapterNodeMixin:
    def publish_health(self):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.msg import PerceptionHealth

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_ok = False
        msg.lidar_ok = False
        msg.yolo_ok = False
        msg.tf_ok = False
        msg.detection_publish_rate_hz = 0.0
        msg.camera_frame_age_ms = -1.0
        msg.lidar_frame_age_ms = -1.0
        msg.upstream_result_age_ms = -1.0
        msg.error_code = int(ErrorCode.E_EXT_PERC_OFFLINE)
        msg.details_json = '{"mode":"skeleton","reason":"no upstream data"}'
        self._health_pub.publish(msg)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("perception_adapter_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth

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
            self._health_timer = self.create_timer(1.0, self.publish_health)
            self.get_logger().info("perception_adapter_node skeleton ready")

    rclpy.init(args=args)
    node = PerceptionAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
