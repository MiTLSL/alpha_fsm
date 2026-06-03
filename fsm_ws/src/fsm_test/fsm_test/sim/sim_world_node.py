from __future__ import annotations

import math

from .scene_loader import SceneTruth, build_scene


class SimWorldMixin:
    def _load_sim_config(self) -> None:
        from ament_index_python.packages import get_package_share_directory
        from pathlib import Path

        from fsm_core.ros2_helpers import declare_parameters_from_dict, load_yaml

        config_dir = Path(get_package_share_directory("fsm_config")) / "params"
        self.config.update(declare_parameters_from_dict(self, load_yaml(config_dir / "sim.yaml")))

    def _configure_sim(self) -> None:
        self._enabled = bool(self.config.get("sim.enabled", False))
        self._output_mode = str(self.config.get("sim.sensor.output_mode", "adapter_input"))
        if self._output_mode not in ("adapter_input", "adapter_bypass"):
            self.get_logger().warning(f"unsupported sim output_mode={self._output_mode}; falling back to adapter_input")
            self._output_mode = "adapter_input"
        self._publish_markers = bool(self.config.get("sim.debug.publish_markers", True))
        self._publish_rate_hz = float(self.config.get("sim.sensor.publish_rate_hz", 10.0))
        self._confidence = float(self.config.get("sim.sensor.confidence", 0.95))
        self._face_inlier_count = int(self.config.get("sim.sensor.face_inlier_count", 32))
        self._frame_seq = 0
        self._scene: SceneTruth = build_scene(self.config)
        self._alignment_distance: float | None = None
        self._alignment_yaw: float | None = None

    def _publish_static_tf(self) -> None:
        from geometry_msgs.msg import TransformStamped

        base_frame = str(self.config.get("sim.frames.base_link", self.config.get("interfaces.frames.base_link", "base_link")))
        body_frame = str(self.config.get("sim.frames.body", self.config.get("interfaces.frames.body", "body")))
        camera_frame = str(self.config.get("sim.frames.camera_link", self.config.get("interfaces.frames.camera_link", "camera_link")))

        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child in (
            (base_frame, body_frame),
            (body_frame, camera_frame),
        ):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self._static_tf_broadcaster.sendTransform(transforms)

    def _on_fake_base_alignment(self, msg) -> None:
        data = list(getattr(msg, "data", []))
        if len(data) < 2:
            return
        self._alignment_distance = float(data[0])
        self._alignment_yaw = float(data[1])

    def _publish_tick(self) -> None:
        if not self._enabled:
            return
        if self._output_mode == "adapter_input":
            self._publish_box_perception_result()
        else:
            self._publish_detection_bypass()
        if self._publish_markers:
            self._publish_markers_msg()
        self._frame_seq += 1

    def _publish_box_perception_result(self) -> None:
        from box_perception_msgs.msg import BoxPerceptionResult, BoxResult

        msg = BoxPerceptionResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._scene.frame_id
        msg.frame_id = int(self._frame_seq)
        for truth in self._scene.boxes:
            center, face_center, normal = self._box_measurement(truth)
            box = BoxResult()
            box.header = msg.header
            box.box_id = int(truth.box_id)
            box.confidence = float(self._confidence)
            box.nearest_face_center.x = face_center[0]
            box.nearest_face_center.y = face_center[1]
            box.nearest_face_center.z = face_center[2]
            box.nearest_face_normal.x = normal[0]
            box.nearest_face_normal.y = normal[1]
            box.nearest_face_normal.z = normal[2]
            box.face_normal_0.x = normal[0]
            box.face_normal_0.y = normal[1]
            box.face_normal_0.z = normal[2]
            box.face_normal_1.z = 1.0
            box.face_inlier_count_0 = self._face_inlier_count
            box.face_inlier_count_1 = self._face_inlier_count
            box.bbox.x_offset = int(truth.col * 10)
            box.bbox.y_offset = int(truth.row * 10)
            box.bbox.width = 8
            box.bbox.height = 8
            msg.boxes.append(box)
        self._box_result_pub.publish(msg)

    def _publish_detection_bypass(self) -> None:
        from fsm_msgs.msg import BoxDetection, BoxDetectionArray

        msg = BoxDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.config.get("interfaces.frames.base_link", "base_link"))
        msg.frame_seq = int(self._frame_seq)
        msg.inference_latency_ms = 0.0
        for truth in self._scene.boxes:
            center, _, _ = self._box_measurement(truth)
            det = BoxDetection()
            det.header = msg.header
            det.detection_id = f"sim_{truth.wall_index}_{truth.row}_{truth.col}"
            det.pose.header = msg.header
            det.pose.pose.position.x = center[0]
            det.pose.pose.position.y = center[1]
            det.pose.pose.position.z = center[2]
            qx, qy, qz, qw = self._quaternion_from_yaw(self._alignment_yaw or 0.0)
            det.pose.pose.orientation.x = qx
            det.pose.pose.orientation.y = qy
            det.pose.pose.orientation.z = qz
            det.pose.pose.orientation.w = qw
            det.size.x = truth.size[0]
            det.size.y = truth.size[1]
            det.size.z = truth.size[2]
            det.confidence = float(self._confidence)
            det.class_label = "box"
            det.pose_valid = True
            msg.detections.append(det)
        self._detections_pub.publish(msg)

    def _publish_markers_msg(self) -> None:
        from visualization_msgs.msg import Marker, MarkerArray

        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for truth in self._scene.boxes:
            center, _, _ = self._box_measurement(truth)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self._scene.frame_id
            marker.ns = "sim_boxes"
            marker.id = int(truth.box_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            qx, qy, qz, qw = self._quaternion_from_yaw(self._alignment_yaw or 0.0)
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = truth.size[0]
            marker.scale.y = truth.size[1]
            marker.scale.z = truth.size[2]
            marker.color.r = 0.2
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.55
            msg.markers.append(marker)
        self._marker_pub.publish(msg)

    def _box_measurement(self, truth):
        if self._alignment_distance is None or self._alignment_yaw is None:
            return truth.center, truth.nearest_face_center, truth.nearest_face_normal
        normal = (math.cos(self._alignment_yaw), math.sin(self._alignment_yaw), 0.0)
        center = (float(self._alignment_distance), truth.center[1], truth.center[2])
        face_center = (
            center[0] + normal[0] * truth.size[0] * 0.5,
            center[1] + normal[1] * truth.size[0] * 0.5,
            center[2],
        )
        return center, face_center, normal

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        half = float(yaw) * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("sim_world_node requires ROS2 rclpy") from exc

    from box_perception_msgs.msg import BoxPerceptionResult
    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_topic_name
    from fsm_msgs.msg import BoxDetectionArray
    from std_msgs.msg import Float32MultiArray
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import MarkerArray

    class SimWorldNode(SkeletonNodeMixin, SimWorldMixin, Node):
        def __init__(self):
            super().__init__("sim_world_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="SimWorld")
            self._load_sim_config()
            self._configure_sim()
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "SimWorld")
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
            self._box_result_pub = self.create_publisher(
                BoxPerceptionResult,
                get_topic_name(self, "box_perception_result", "/box_perception/result"),
                10,
            )
            self._detections_pub = self.create_publisher(
                BoxDetectionArray,
                get_topic_name(self, "perception_detections", "/perception/box_detections"),
                5,
            )
            self._alignment_sub = self.create_subscription(
                Float32MultiArray,
                str(self.config.get("sim.fake_base.alignment_topic", "/sim/fake_base_alignment")),
                self._on_fake_base_alignment,
                10,
            )
            self._marker_pub = self.create_publisher(
                MarkerArray,
                str(self.config.get("sim.debug.marker_topic", "/sim/markers")),
                1,
            )
            self._publish_static_tf()
            self._timer = self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._publish_tick)
            self.get_logger().info(
                f"sim_world_node ready output_mode={self._output_mode} boxes={len(self._scene.boxes)} frame={self._scene.frame_id}"
            )

        def on_config_reloaded(self) -> None:
            self._load_sim_config()
            self._configure_sim()
            self._publish_static_tf()

    rclpy.init(args=args)
    node = SimWorldNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
