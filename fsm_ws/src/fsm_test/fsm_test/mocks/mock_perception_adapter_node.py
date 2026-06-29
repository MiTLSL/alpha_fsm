from __future__ import annotations

import json

from .common import FailureInjectionMixin, json_details, make_pose_stamped


class MockPerceptionAdapterMixin(FailureInjectionMixin):
    VALID_MODES = ("OBSERVATION", "LEFT_PHASE", "RIGHT_PHASE", "EMPTY", "PARTIAL", "JITTER")

    def on_config_reloaded(self) -> None:
        self._mode = str(self.get_parameter("mode").value)
        if self._mode not in self.VALID_MODES:
            self.get_logger().warning(f"unsupported mock perception mode {self._mode}, fallback to OBSERVATION")
            self._mode = "OBSERVATION"

    def handle_inject_failure(self, request, response):
        response = super().handle_inject_failure(request, response)
        if not request.params_json:
            return response
        try:
            params = json.loads(request.params_json)
        except json.JSONDecodeError as exc:
            response.accepted = False
            response.message = f"invalid params_json: {exc}"
            return response
        mode = str(params.get("mode", "")).strip()
        if not mode:
            return response
        if mode not in self.VALID_MODES:
            response.accepted = False
            response.message = f"unsupported mode: {mode}"
            return response
        self._mode = mode
        response.message = f"{response.message}; mode set to {self._mode}"
        return response

    def publish_mock_data(self):
        self.publish_health()
        if self._current_failure == "STOP_PUBLISHING":
            return
        self.publish_detections()

    def publish_health(self):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.msg import PerceptionHealth

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_ok = self._current_failure != "CAMERA_OFFLINE"
        msg.lidar_ok = self._current_failure != "LIDAR_OFFLINE"
        msg.yolo_ok = self._current_failure != "YOLO_FALLBACK"
        msg.tf_ok = self._current_failure != "TF_TIMEOUT"
        msg.detection_publish_rate_hz = 1.0 if self._current_failure == "RATE_LOW" else float(self._publish_rate_hz)
        msg.camera_frame_age_ms = 10.0 if msg.camera_ok else 9999.0
        msg.lidar_frame_age_ms = 10.0 if msg.lidar_ok else 9999.0
        msg.upstream_result_age_ms = 10.0 if msg.yolo_ok else 9999.0
        if not msg.camera_ok:
            msg.error_code = int(ErrorCode.E_EXT_PERC_CAMERA_FAIL)
        elif not msg.lidar_ok:
            msg.error_code = int(ErrorCode.E_EXT_PERC_LIDAR_FAIL)
        elif not msg.yolo_ok:
            msg.error_code = int(ErrorCode.E_EXT_PERC_YOLO_FAIL)
        elif not msg.tf_ok:
            msg.error_code = int(ErrorCode.E_COMM_TF_LOOKUP_FAIL)
        elif self._current_failure == "RATE_LOW":
            msg.error_code = int(ErrorCode.E_EXT_PERC_RATE_LOW)
        else:
            msg.error_code = 0
        msg.details_json = json_details(mock=True, mode=self._mode, failure=self._current_failure)
        self._health_pub.publish(msg)

    def publish_detections(self):
        from fsm_msgs.msg import BoxDetection, BoxDetectionArray

        msg = BoxDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body" if self._current_failure == "INVALID_FRAME" else "base_link"
        msg.frame_seq = self._frame_seq
        msg.inference_latency_ms = 5.0
        self._frame_seq += 1

        if self._mode == "EMPTY":
            self._detections_pub.publish(msg)
            return

        cols_by_mode = {
            "OBSERVATION": range(5),
            "LEFT_PHASE": self._left_phase_cols,
            "RIGHT_PHASE": self._right_phase_cols,
            "PARTIAL": range(5),
            "JITTER": range(5),
        }
        positions = [(row, col) for row in range(5) for col in cols_by_mode.get(self._mode, range(5))]
        if self._mode == "PARTIAL":
            positions = positions[:6]

        for index, (row, col) in enumerate(positions):
            det = BoxDetection()
            det.header = msg.header
            det.detection_id = f"mock_{self._frame_seq}_{index}"
            jitter_y = 0.15 if self._mode == "JITTER" and index % 2 == 0 else 0.0
            jitter_z = -0.15 if self._mode == "JITTER" and index % 3 == 0 else 0.0
            det.pose = make_pose_stamped(msg.header.frame_id, 0.6, (2 - col) * 0.4 + jitter_y, 1.8 - row * 0.4 + jitter_z)
            det.size.x = 0.4
            det.size.y = 0.4
            det.size.z = 0.4
            det.confidence = 0.2 if self._current_failure == "YOLO_FALLBACK" else 0.95
            det.class_label = "box"
            det.pose_valid = self._current_failure != "TF_TIMEOUT"
            msg.detections.append(det)
        self._detections_pub.publish(msg)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("mock_perception_adapter_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth

    class MockPerceptionAdapterNode(SkeletonNodeMixin, MockPerceptionAdapterMixin, Node):
        def __init__(self):
            super().__init__("mock_perception_adapter_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="MockPerceptionAdapter")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "MockPerceptionAdapter")
            self.init_failure_injection()
            self.declare_parameter("mode", "OBSERVATION")
            self.on_config_reloaded()
            self._left_phase_cols = [int(col) for col in self.config.get("business.left_phase_cols", [0, 1, 2])]
            self._right_phase_cols = [int(col) for col in self.config.get("business.right_phase_cols", [2, 3, 4])]
            self._publish_rate_hz = float(self.config.get("business.perception_adapter.publish_rate_hz", 10.0))
            self._frame_seq = 0
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
            self._inject_srv = self.create_inject_failure_service()
            self._timer = self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self.publish_mock_data)
            self.get_logger().info("mock_perception_adapter_node ready")

    rclpy.init(args=args)
    node = MockPerceptionAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
