from __future__ import annotations

import json
import time


class PerceptionAdapterNodeMixin:
    def init_upstream_adapter(self):
        self._upstream_msg_type = None
        self._upstream_import_error = ""
        self._upstream_subscription = None
        self._last_upstream_monotonic = 0.0
        self._last_conversion_error = ""
        self._frame_seq = 0
        self._detection_publish_count = 0
        self._detection_rate_window_start = time.monotonic()

        try:
            from box_perception_msgs.msg import BoxPerceptionResult
        except ImportError as exc:
            self._upstream_import_error = str(exc)
            self.get_logger().warning(
                "box_perception_msgs is not available; perception_adapter will only publish offline health. "
                "Treat this as an M2-A02 prerequisite, not a fake upstream implementation."
            )
            return
        self._upstream_msg_type = BoxPerceptionResult

    def create_upstream_subscription(self, msg_type) -> None:
        from fsm_core.ros2_helpers import get_topic_name

        self._upstream_subscription = self.create_subscription(
            msg_type,
            get_topic_name(self, "box_perception_result", "/box_perception/result"),
            self.on_box_perception_result,
            5,
        )

    def on_box_perception_result(self, msg):
        self._last_upstream_monotonic = time.monotonic()
        try:
            converted = self._convert_box_perception_result(msg)
        except Exception as exc:  # pragma: no cover - depends on optional upstream message shape
            self._last_conversion_error = str(exc)
            self.get_logger().warning(f"failed to adapt /box_perception/result: {exc}")
            return
        self._last_conversion_error = ""
        self._detections_pub.publish(converted)
        self._detection_publish_count += 1

    def publish_health(self):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.msg import PerceptionHealth

        now = time.monotonic()
        has_upstream_type = self._upstream_msg_type is not None
        upstream_seen = self._last_upstream_monotonic > 0.0
        upstream_age_ms = (now - self._last_upstream_monotonic) * 1000.0 if upstream_seen else -1.0
        upstream_timeout_ms = float(self.config.get("business.perception_adapter.result_timeout_ms", 1000.0))
        upstream_fresh = upstream_seen and upstream_age_ms <= upstream_timeout_ms and not self._last_conversion_error

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_ok = bool(upstream_fresh)
        msg.lidar_ok = bool(upstream_fresh)
        msg.yolo_ok = bool(upstream_fresh)
        msg.tf_ok = bool(upstream_fresh)
        msg.detection_publish_rate_hz = float(self._detection_publish_rate_hz(now))
        msg.camera_frame_age_ms = upstream_age_ms
        msg.lidar_frame_age_ms = upstream_age_ms
        msg.upstream_result_age_ms = upstream_age_ms
        if not has_upstream_type:
            msg.error_code = int(ErrorCode.E_EXT_PERC_OFFLINE)
            reason = "box_perception_msgs missing"
        elif not upstream_seen:
            msg.error_code = int(ErrorCode.E_EXT_PERC_OFFLINE)
            reason = "no upstream data"
        elif self._last_conversion_error:
            msg.error_code = int(ErrorCode.E_EXT_PERC_INTERNAL)
            reason = "upstream conversion failed"
        elif not upstream_fresh:
            msg.error_code = int(ErrorCode.E_EXT_PERC_RATE_LOW)
            reason = "upstream result timeout"
        else:
            msg.error_code = 0
            reason = "upstream fresh"
        msg.details_json = json.dumps(
            {
                "mode": "strict_adapter_skeleton",
                "reason": reason,
                "has_box_perception_msgs": has_upstream_type,
                "m2_prerequisite": not has_upstream_type,
                "import_error": self._upstream_import_error,
                "last_conversion_error": self._last_conversion_error,
            },
            sort_keys=True,
        )
        self._health_pub.publish(msg)

    def _convert_box_perception_result(self, msg):
        from fsm_msgs.msg import BoxDetection, BoxDetectionArray

        items = self._extract_result_items(msg)
        output = BoxDetectionArray()
        output.header.stamp = getattr(getattr(msg, "header", None), "stamp", self.get_clock().now().to_msg())
        output.header.frame_id = "base_link"
        output.frame_seq = int(self._frame_seq)
        output.inference_latency_ms = float(getattr(msg, "inference_latency_ms", getattr(msg, "latency_ms", 0.0)))
        self._frame_seq += 1

        for index, item in enumerate(items):
            det = BoxDetection()
            det.header = output.header
            det.detection_id = str(getattr(item, "detection_id", getattr(item, "id", f"box_{output.frame_seq}_{index}")))
            det.pose = self._pose_from_upstream_item(item, output.header.stamp)
            det.size.x = float(getattr(getattr(item, "size", None), "x", self.config.get("business.box_size.length", 0.4)))
            det.size.y = float(getattr(getattr(item, "size", None), "y", self.config.get("business.box_size.width", 0.4)))
            det.size.z = float(getattr(getattr(item, "size", None), "z", self.config.get("business.box_size.height", 0.4)))
            det.confidence = float(getattr(item, "confidence", getattr(item, "score", 1.0)))
            det.class_label = str(getattr(item, "class_label", "box"))
            det.pose_valid = bool(getattr(item, "pose_valid", True))
            output.detections.append(det)
        return output

    def _extract_result_items(self, msg) -> list[object]:
        for field_name in ("boxes", "detections", "results", "box_results"):
            value = getattr(msg, field_name, None)
            if value is not None:
                return list(value)
        raise ValueError("unsupported BoxPerceptionResult shape: missing boxes/detections/results")

    def _pose_from_upstream_item(self, item, stamp):
        pose = getattr(item, "pose", None) or getattr(item, "box_pose", None) or getattr(item, "target_pose", None)
        if pose is not None:
            return self._transform_pose_to_base_link(pose, stamp)
        center = getattr(item, "nearest_face_center", None) or getattr(item, "center", None)
        if center is None:
            raise ValueError("upstream item has no pose or nearest_face_center")
        from geometry_msgs.msg import PoseStamped

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = str(getattr(item, "frame_id", getattr(getattr(item, "header", None), "frame_id", "")) or "body")
        pose_stamped.header.stamp = stamp
        pose_stamped.pose.position.x = float(getattr(center, "x", 0.0))
        pose_stamped.pose.position.y = float(getattr(center, "y", 0.0))
        pose_stamped.pose.position.z = float(getattr(center, "z", 0.0))
        pose_stamped.pose.orientation.w = 1.0
        return self._transform_pose_to_base_link(pose_stamped, stamp)

    def _transform_pose_to_base_link(self, pose, stamp):
        from geometry_msgs.msg import PoseStamped

        if hasattr(pose, "pose"):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = str(getattr(getattr(pose, "header", None), "frame_id", "") or "base_link")
            pose_stamped.header.stamp = getattr(getattr(pose, "header", None), "stamp", stamp)
            pose_stamped.pose = pose.pose
        else:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = pose
        if pose_stamped.header.frame_id != "base_link":
            raise RuntimeError("TF2 body/camera to base_link conversion is an M2-A02 prerequisite")
        return pose_stamped

    def _detection_publish_rate_hz(self, now: float) -> float:
        elapsed = max(now - self._detection_rate_window_start, 1e-6)
        return float(self._detection_publish_count / elapsed)


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
            self.init_upstream_adapter()
            if self._upstream_msg_type is not None:
                self.create_upstream_subscription(self._upstream_msg_type)
            self._health_timer = self.create_timer(1.0, self.publish_health)
            self.get_logger().info("perception_adapter_node skeleton ready")

    rclpy.init(args=args)
    node = PerceptionAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
