from __future__ import annotations

from .geometry import cross, dot, normalize, quaternion_from_axes, vector_length
from .tf_adapter import TfAdapterMixin


class BoxResultConverterMixin(TfAdapterMixin):
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
        if vector_length((float(normal.x), float(normal.y), float(normal.z))) <= 1e-6:
            return False
        min_inliers = int(self.config.get("business.perception_adapter.min_face_inlier_count", 0))
        if min_inliers <= 0:
            return True
        return int(getattr(item, "face_inlier_count_0", 0)) >= min_inliers or int(getattr(item, "face_inlier_count_1", 0)) >= min_inliers

    def _normal_xyz_or_default(self, normal) -> tuple[float, float, float]:
        if normal is None:
            return (1.0, 0.0, 0.0)
        return normalize((float(getattr(normal, "x", 0.0)), float(getattr(normal, "y", 0.0)), float(getattr(normal, "z", 0.0))))

    def _orientation_from_face_normal(self, normal_xyz: tuple[float, float, float]) -> tuple[float, float, float, float]:
        x_axis = normalize(normal_xyz)
        up = (0.0, 0.0, 1.0)
        if abs(dot(x_axis, up)) > 0.95:
            up = (0.0, 1.0, 0.0)
        y_axis = normalize(cross(up, x_axis))
        z_axis = normalize(cross(x_axis, y_axis))
        return quaternion_from_axes(x_axis, y_axis, z_axis)
