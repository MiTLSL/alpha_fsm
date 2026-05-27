from __future__ import annotations

from .geometry import normalize_quaternion, quat_multiply, rotate_vector


class TfAdapterMixin:
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
        rotated = rotate_vector(rot, point)
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
        qx, qy, qz, qw = normalize_quaternion(quat_multiply(rot, pose_q))
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        return out
