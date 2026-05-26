from __future__ import annotations

import json
import math
import time


class PerceptionAdapterNodeMixin:
    def init_upstream_adapter(self):
        self._upstream_msg_type = None
        self._upstream_import_error = ""
        self._upstream_subscription = None
        self._last_upstream_monotonic = 0.0
        self._last_conversion_error = ""
        self._last_tf_error = ""
        self._last_tf_ok = False
        self._last_used_static_fallback = False
        self._frame_seq = 0
        self._detection_publish_count = 0
        self._detection_rate_window_start = time.monotonic()
        self._base_frame = str(self.config.get("interfaces.frames.base_link", "base_link"))
        self._default_upstream_frame = str(self.config.get("interfaces.frames.body", "body"))
        self._tf_lookup_timeout_sec = float(self.config.get("business.perception_adapter.tf_timeout_ms", 200.0)) / 1000.0
        self._allow_static_tf_fallback = bool(self.config.get("business.tf_static_fallback.enabled", True))
        self._static_tf_fallback_frames = {
            str(frame)
            for frame in self.config.get(
                "business.tf_static_fallback.allowed_source_frames",
                ["body", "camera_link", "depth_camera_link", "camera_color_optical_frame"],
            )
        }

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
        self.get_logger().info("box_perception_msgs detected; subscribing to /box_perception/result")

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
        tf_ready = bool(upstream_fresh and (self._last_tf_ok or self._last_used_static_fallback))

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_ok = bool(upstream_fresh)
        msg.lidar_ok = bool(upstream_fresh)
        msg.yolo_ok = bool(upstream_fresh)
        msg.tf_ok = bool(tf_ready)
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
                "mode": "m2_preintegration_adapter",
                "reason": reason,
                "base_frame": self._base_frame,
                "default_upstream_frame": self._default_upstream_frame,
                "has_box_perception_msgs": has_upstream_type,
                "m2_prerequisite": not has_upstream_type,
                "import_error": self._upstream_import_error,
                "last_conversion_error": self._last_conversion_error,
                "last_tf_error": self._last_tf_error,
                "last_tf_ok": bool(self._last_tf_ok),
                "used_static_fallback": bool(self._last_used_static_fallback),
            },
            sort_keys=True,
        )
        self._health_pub.publish(msg)

    def _convert_box_perception_result(self, msg):
        from fsm_msgs.msg import BoxDetection, BoxDetectionArray

        items = self._extract_result_items(msg)
        output = BoxDetectionArray()
        output.header.stamp = self._message_stamp_or_now(msg)
        output.header.frame_id = self._base_frame
        output.frame_seq = int(self._frame_seq)
        output.inference_latency_ms = float(getattr(msg, "inference_latency_ms", getattr(msg, "latency_ms", 0.0)))
        self._frame_seq += 1

        for index, item in enumerate(items):
            det = BoxDetection()
            det.header = output.header
            det.detection_id = self._detection_id(item, output.frame_seq, index)
            det.pose = self._pose_from_upstream_item(item, output.header.stamp)
            det.size.x = float(getattr(getattr(item, "size", None), "x", self.config.get("business.box_size.length", 0.4)))
            det.size.y = float(getattr(getattr(item, "size", None), "y", self.config.get("business.box_size.width", 0.4)))
            det.size.z = float(getattr(getattr(item, "size", None), "z", self.config.get("business.box_size.height", 0.4)))
            det.confidence = float(getattr(item, "confidence", getattr(item, "score", 1.0)))
            det.class_label = str(getattr(item, "class_label", "box"))
            det.pose_valid = bool(getattr(item, "pose_valid", self._box_result_pose_valid(item)))
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

        normal = getattr(item, "nearest_face_normal", None)
        normal_xyz = self._normal_xyz_or_default(normal)
        box_depth = float(self.config.get("business.box_size.length", 0.4))
        center_offset_sign = float(self.config.get("business.perception_adapter.box_center_offset_sign", -1.0))
        center_offset = 0.5 * box_depth * center_offset_sign

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._source_frame_for_item(item)
        pose_stamped.header.stamp = stamp
        pose_stamped.pose.position.x = float(getattr(center, "x", 0.0)) + center_offset * normal_xyz[0]
        pose_stamped.pose.position.y = float(getattr(center, "y", 0.0)) + center_offset * normal_xyz[1]
        pose_stamped.pose.position.z = float(getattr(center, "z", 0.0)) + center_offset * normal_xyz[2]
        qx, qy, qz, qw = self._orientation_from_face_normal(normal_xyz)
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        return self._transform_pose_to_base_link(pose_stamped, stamp)

    def _transform_pose_to_base_link(self, pose, stamp):
        from geometry_msgs.msg import PoseStamped

        if hasattr(pose, "pose"):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = str(getattr(getattr(pose, "header", None), "frame_id", "") or self._base_frame)
            pose_stamped.header.stamp = getattr(getattr(pose, "header", None), "stamp", stamp)
            pose_stamped.pose = pose.pose
        else:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self._base_frame
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = pose
        if pose_stamped.header.frame_id == self._base_frame:
            pose_stamped.header.frame_id = self._base_frame
            self._last_tf_ok = True
            self._last_tf_error = ""
            self._last_used_static_fallback = False
            return pose_stamped
        pose_stamped.header.stamp = self._stamp_or_now(pose_stamped.header.stamp)
        try:
            transform = self._lookup_transform_to_base(pose_stamped.header.frame_id, pose_stamped.header.stamp)
        except Exception as exc:
            self._last_tf_ok = False
            self._last_tf_error = str(exc)
            if not self._can_use_static_tf_fallback(pose_stamped.header.frame_id):
                raise
            self._last_used_static_fallback = True
            pose_stamped.header.frame_id = self._base_frame
            return pose_stamped
        self._last_tf_ok = True
        self._last_tf_error = ""
        self._last_used_static_fallback = False
        return self._apply_transform(pose_stamped, transform)

    def _can_use_static_tf_fallback(self, source_frame: str) -> bool:
        return bool(self._allow_static_tf_fallback and str(source_frame) in self._static_tf_fallback_frames)

    def _message_stamp_or_now(self, msg):
        return self._stamp_or_now(getattr(getattr(msg, "header", None), "stamp", None))

    def _stamp_or_now(self, stamp):
        if stamp is None or (int(getattr(stamp, "sec", 0)) == 0 and int(getattr(stamp, "nanosec", 0)) == 0):
            return self.get_clock().now().to_msg()
        return stamp

    def _detection_id(self, item, frame_seq: int, index: int) -> str:
        value = getattr(item, "detection_id", None)
        if value is not None:
            return str(value)
        value = getattr(item, "id", None)
        if value is not None:
            return str(value)
        value = getattr(item, "box_id", None)
        if value is not None:
            return f"box_{int(value)}"
        return f"box_{int(frame_seq)}_{int(index)}"

    def _source_frame_for_item(self, item) -> str:
        header_frame = str(getattr(getattr(item, "header", None), "frame_id", "") or "")
        return header_frame or self._default_upstream_frame

    def _box_result_pose_valid(self, item) -> bool:
        center = getattr(item, "nearest_face_center", None) or getattr(item, "center", None)
        normal = getattr(item, "nearest_face_normal", None)
        if center is None or normal is None:
            return False
        if self._vector_length((float(normal.x), float(normal.y), float(normal.z))) <= 1e-6:
            return False
        min_inliers = int(self.config.get("business.perception_adapter.min_face_inlier_count", 0))
        if min_inliers <= 0:
            return True
        return int(getattr(item, "face_inlier_count_0", 0)) >= min_inliers or int(getattr(item, "face_inlier_count_1", 0)) >= min_inliers

    def _normal_xyz_or_default(self, normal) -> tuple[float, float, float]:
        if normal is None:
            return (1.0, 0.0, 0.0)
        return self._normalize((float(getattr(normal, "x", 0.0)), float(getattr(normal, "y", 0.0)), float(getattr(normal, "z", 0.0))))

    def _orientation_from_face_normal(self, normal_xyz: tuple[float, float, float]) -> tuple[float, float, float, float]:
        x_axis = self._normalize(normal_xyz)
        up = (0.0, 0.0, 1.0)
        if abs(self._dot(x_axis, up)) > 0.95:
            up = (0.0, 1.0, 0.0)
        y_axis = self._normalize(self._cross(up, x_axis))
        z_axis = self._normalize(self._cross(x_axis, y_axis))
        return self._quaternion_from_axes(x_axis, y_axis, z_axis)

    def _lookup_transform_to_base(self, source_frame: str, stamp):
        if getattr(self, "_tf_buffer", None) is None:
            raise RuntimeError("TF2 buffer is not initialized")
        from rclpy.duration import Duration
        from rclpy.time import Time

        lookup_time = Time.from_msg(stamp)
        return self._tf_buffer.lookup_transform(
            self._base_frame,
            source_frame,
            lookup_time,
            timeout=Duration(seconds=max(self._tf_lookup_timeout_sec, 0.0)),
        )

    def _apply_transform(self, pose_stamped, transform):
        from geometry_msgs.msg import PoseStamped

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))
        point = (
            float(pose_stamped.pose.position.x),
            float(pose_stamped.pose.position.y),
            float(pose_stamped.pose.position.z),
        )
        rotated = self._rotate_vector(rot, point)
        out = PoseStamped()
        out.header.stamp = pose_stamped.header.stamp
        out.header.frame_id = self._base_frame
        out.pose.position.x = rotated[0] + float(translation.x)
        out.pose.position.y = rotated[1] + float(translation.y)
        out.pose.position.z = rotated[2] + float(translation.z)
        pose_q = (
            float(pose_stamped.pose.orientation.x),
            float(pose_stamped.pose.orientation.y),
            float(pose_stamped.pose.orientation.z),
            float(pose_stamped.pose.orientation.w),
        )
        qx, qy, qz, qw = self._normalize_quaternion(self._quat_multiply(rot, pose_q))
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        return out

    def _detection_publish_rate_hz(self, now: float) -> float:
        elapsed = max(now - self._detection_rate_window_start, 1e-6)
        return float(self._detection_publish_count / elapsed)

    @staticmethod
    def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])

    @staticmethod
    def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            float(a[1] * b[2] - a[2] * b[1]),
            float(a[2] * b[0] - a[0] * b[2]),
            float(a[0] * b[1] - a[1] * b[0]),
        )

    @staticmethod
    def _vector_length(value: tuple[float, float, float]) -> float:
        return math.sqrt(float(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]))

    def _normalize(self, value: tuple[float, float, float]) -> tuple[float, float, float]:
        length = self._vector_length(value)
        if length <= 1e-9:
            return (1.0, 0.0, 0.0)
        return (float(value[0] / length), float(value[1] / length), float(value[2] / length))

    @staticmethod
    def _normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        length = math.sqrt(float(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]))
        if length <= 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (float(q[0] / length), float(q[1] / length), float(q[2] / length), float(q[3] / length))

    @staticmethod
    def _quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    def _rotate_vector(self, q: tuple[float, float, float, float], value: tuple[float, float, float]) -> tuple[float, float, float]:
        q = self._normalize_quaternion(q)
        vector_q = (float(value[0]), float(value[1]), float(value[2]), 0.0)
        q_conj = (-q[0], -q[1], -q[2], q[3])
        rotated = self._quat_multiply(self._quat_multiply(q, vector_q), q_conj)
        return (float(rotated[0]), float(rotated[1]), float(rotated[2]))

    def _quaternion_from_axes(
        self,
        x_axis: tuple[float, float, float],
        y_axis: tuple[float, float, float],
        z_axis: tuple[float, float, float],
    ) -> tuple[float, float, float, float]:
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
        return self._normalize_quaternion(q)


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
