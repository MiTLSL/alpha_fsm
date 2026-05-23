from __future__ import annotations


class WallDestackingStrategyNodeMixin:
    def on_detections(self, msg):
        self._last_detection_count = len(msg.detections)

    def on_perception_health(self, msg):
        self._last_perception_error = int(msg.error_code)

    def on_safety_status(self, msg):
        self._estop = bool(msg.estop)

    async def execute_wall_destacking(self, goal_handle):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import RunWallDestacking

        feedback = RunWallDestacking.Feedback()
        feedback.current_wall_index = goal_handle.request.start_wall_index
        feedback.current_phase = 0
        feedback.current_state = "NOT_IMPLEMENTED"
        feedback.phase_progress_percent = 0
        feedback.elapsed_sec = 0.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = RunWallDestacking.Result()
        result.success = False
        result.walls_completed = 0
        result.total_boxes_picked = 0
        result.error_code = int(ErrorCode.E_WALL_UNKNOWN)
        result.failure_reason = "wall_destacking_strategy_node skeleton: not implemented"
        return result


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("wall_destacking_strategy_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_topic_name
    from fsm_msgs.action import RunWallDestacking
    from fsm_msgs.msg import BoxDetectionArray, PerceptionHealth, SafetyStatus

    class WallDestackingStrategyNode(SkeletonNodeMixin, WallDestackingStrategyNodeMixin, Node):
        def __init__(self):
            super().__init__("wall_destacking_strategy_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="WallDestackingFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_wall_destacking_state", "/fsm/wall_destacking_state", "WallDestackingFSM")
            self._last_detection_count = 0
            self._last_perception_error = 0
            self._estop = False
            self._detections_sub = self.create_subscription(BoxDetectionArray, get_topic_name(self, "perception_detections", "/perception/box_detections"), self.on_detections, 10)
            self._perception_health_sub = self.create_subscription(PerceptionHealth, get_topic_name(self, "perception_health", "/perception/health"), self.on_perception_health, 10)
            self._safety_sub = self.create_subscription(SafetyStatus, get_topic_name(self, "safety_status", "/safety/status"), self.on_safety_status, 10)
            self._action_server = ActionServer(
                self,
                RunWallDestacking,
                get_action_name(self, "run_wall_destacking", "/run_wall_destacking"),
                self.execute_wall_destacking,
            )
            self.get_logger().info("wall_destacking_strategy_node skeleton ready")

    rclpy.init(args=args)
    node = WallDestackingStrategyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
