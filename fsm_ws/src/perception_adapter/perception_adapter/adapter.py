from __future__ import annotations

import time

from .converter import BoxResultConverterMixin
from .health import PerceptionHealthMixin


class PerceptionAdapterNodeMixin(BoxResultConverterMixin, PerceptionHealthMixin):
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
