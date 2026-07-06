from __future__ import annotations

import json
import time

from .config import load_sim_parameters
from .scene_truth import ViewFilter, filter_visible_boxes, load_box_truths, yaw_to_quaternion


def main(args=None):
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("isaac_ground_truth_perception_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name, make_qos_profile
    from fsm_msgs.msg import BoxDetection, BoxDetectionArray, PerceptionHealth

    class IsaacGroundTruthPerceptionNode(SkeletonNodeMixin, Node):
        def __init__(self):
            super().__init__("isaac_ground_truth_perception_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="IsaacGroundTruthPerception")
            load_sim_parameters(self)
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "IsaacGroundTruthPerception")
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
            self._configure()
            self._publish_count = 0
            self._rate_window_start = time.monotonic()
            period = 1.0 / max(self._publish_rate_hz, 0.1)
            self._timer = self.create_timer(period, self._publish_tick)
            self._health_timer = self.create_timer(1.0, self._publish_health)
            self.get_logger().info(
                f"isaac_ground_truth_perception_node ready boxes={len(self._boxes)} source={self._source_mode}"
            )

        def on_config_reloaded(self):
            load_sim_parameters(self)
            self._configure()

        def _configure(self):
            self._source_mode = str(self.config.get("sim.isaac.perception.source_mode", "param_truth"))
            self._base_frame = str(self.config.get("interfaces.frames.base_link", "base_link"))
            default_size = (
                float(self.config.get("business.box_size.length", 0.4)),
                float(self.config.get("business.box_size.width", 0.4)),
                float(self.config.get("business.box_size.height", 0.4)),
            )
            scene_file = str(self.config.get("sim.isaac.perception.scene_truth_file", ""))
            boxes_json = str(self.config.get("sim.isaac.perception.boxes_json", ""))
            self._boxes = load_box_truths(
                scene_file=scene_file,
                boxes_json=boxes_json,
                default_frame=self._base_frame,
                default_size=default_size,
            )
            self._publish_rate_hz = float(self.config.get("sim.isaac.perception.publish_rate_hz", 10.0))
            self._latency_ms = float(self.config.get("sim.isaac.perception.synthetic_latency_ms", 0.0))
            self._view = ViewFilter(
                max_distance_m=float(self.config.get("sim.isaac.perception.view.max_distance_m", 3.0)),
                horizontal_fov_rad=float(self.config.get("sim.isaac.perception.view.horizontal_fov_rad", 2.2)),
                z_min=float(self.config.get("sim.isaac.perception.view.z_min", -0.2)),
                z_max=float(self.config.get("sim.isaac.perception.view.z_max", 2.8)),
            )
            self._ready_state = "READY" if self._source_mode in ("param_truth", "isaac_sdk") else "CONFIG_ERROR"

        def _publish_tick(self):
            if self._ready_state == "CONFIG_ERROR":
                return
            msg = BoxDetectionArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame
            msg.frame_seq = int(self._publish_count)
            msg.inference_latency_ms = float(self._latency_ms)
            for truth in filter_visible_boxes(self._boxes, self._view):
                det = BoxDetection()
                det.header = msg.header
                det.detection_id = truth.detection_id
                det.pose.header = msg.header
                det.pose.pose.position.x = truth.center[0]
                det.pose.pose.position.y = truth.center[1]
                det.pose.pose.position.z = truth.center[2]
                qx, qy, qz, qw = yaw_to_quaternion(truth.yaw)
                det.pose.pose.orientation.x = qx
                det.pose.pose.orientation.y = qy
                det.pose.pose.orientation.z = qz
                det.pose.pose.orientation.w = qw
                det.size.x = truth.size[0]
                det.size.y = truth.size[1]
                det.size.z = truth.size[2]
                det.confidence = float(truth.confidence)
                det.class_label = "box"
                det.pose_valid = True
                msg.detections.append(det)
            self._detections_pub.publish(msg)
            self._publish_count += 1

        def _publish_health(self):
            from fsm_core.error_code import ErrorCode

            now = time.monotonic()
            elapsed = max(now - self._rate_window_start, 1e-6)
            msg = PerceptionHealth()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.camera_ok = True
            msg.lidar_ok = True
            msg.yolo_ok = True
            msg.tf_ok = True
            msg.detection_publish_rate_hz = float(self._publish_count / elapsed)
            msg.camera_frame_age_ms = 0.0
            msg.lidar_frame_age_ms = 0.0
            msg.upstream_result_age_ms = 0.0
            if self._ready_state == "CONFIG_ERROR":
                msg.error_code = int(ErrorCode.E_SYS_CONFIG_INVALID)
                reason = "unsupported source_mode"
            else:
                msg.error_code = 0
                reason = "ground truth boxes published"
            msg.details_json = json.dumps(
                {
                    "mode": "isaac_ground_truth",
                    "source_mode": self._source_mode,
                    "reason": reason,
                    "base_frame": self._base_frame,
                    "configured_boxes": len(self._boxes),
                    "isaac_sdk_required": self._source_mode == "isaac_sdk",
                },
                sort_keys=True,
            )
            self._health_pub.publish(msg)

    rclpy.init(args=args)
    node = IsaacGroundTruthPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
