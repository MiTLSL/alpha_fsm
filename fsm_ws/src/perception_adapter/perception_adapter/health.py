from __future__ import annotations

import json
import time


class PerceptionHealthMixin:
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

    def _detection_publish_rate_hz(self, now: float) -> float:
        elapsed = max(now - self._detection_rate_window_start, 1e-6)
        return float(self._detection_publish_count / elapsed)
