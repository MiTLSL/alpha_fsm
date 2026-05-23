from __future__ import annotations


def _make_pose_stamped(frame_id: str = "map"):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


class NavigationManagerNodeMixin:
    def publish_nav_health(self):
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = False
        self._nav_health_pub.publish(msg)

    async def execute_navigation_goal(self, goal_handle):
        from fsm_msgs.action import NavigateToPose
        from fsm_core.error_code import ErrorCode

        feedback = NavigateToPose.Feedback()
        feedback.current_state = "NOT_IMPLEMENTED"
        feedback.distance_remaining = 0.0
        feedback.estimated_time_remaining = 0.0
        feedback.alignment_error_current = float("nan")
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.success = False
        result.actual_base_pose = _make_pose_stamped("map")
        result.position_error = 0.0
        result.yaw_error = 0.0
        result.alignment_error = float("nan")
        result.workpose_valid = False
        result.error_code = int(ErrorCode.E_NAV_UNKNOWN)
        result.failure_reason = "navigation_manager_node skeleton: not implemented"
        return result

    def handle_base_recovery(self, request, response):
        from fsm_core.constants import ClearErrorStage
        from fsm_core.error_code import ErrorCode

        if request.command == request.RELEASE_ESTOP:
            response.stage_reached = int(ClearErrorStage.ESTOP_RELEASED)
        elif request.command == request.RESET_FAULT:
            response.stage_reached = int(ClearErrorStage.FAULT_RESET)
        elif request.command == request.ENABLE_CHASSIS:
            response.stage_reached = int(ClearErrorStage.CHASSIS_ENABLED)
        else:
            response.stage_reached = int(ClearErrorStage.NONE)
            response.success = False
            response.error_code = int(ErrorCode.E_SYS_CONFIG_INVALID)
            response.message = "unknown base recovery command"
            return response

        response.success = False
        response.error_code = int(ErrorCode.E_NAV_UNKNOWN)
        response.message = "navigation_manager_node skeleton: base recovery not implemented"
        return response


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("navigation_manager_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_service_name, get_topic_name
    from fsm_msgs.action import NavigateToPose
    from fsm_msgs.srv import BaseRecoveryCommand
    from std_msgs.msg import Bool

    class NavigationManagerNode(SkeletonNodeMixin, NavigationManagerNodeMixin, Node):
        def __init__(self):
            super().__init__("navigation_manager_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="BaseNavigationFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "BaseNavigationFSM")
            self._action_server = ActionServer(
                self,
                NavigateToPose,
                get_action_name(self, "navigate_to_pose", "/navigate_to_pose"),
                self.execute_navigation_goal,
            )
            self._recovery_srv = self.create_service(
                BaseRecoveryCommand,
                get_service_name(self, "nav_base_recovery", "/nav/base_recovery"),
                self.handle_base_recovery,
            )
            self._nav_health_pub = self.create_publisher(Bool, get_topic_name(self, "nav_health", "/fsm/nav_health"), 1)
            self._nav_health_timer = self.create_timer(1.0, self.publish_nav_health)
            self.get_logger().info("navigation_manager_node skeleton ready")

    rclpy.init(args=args)
    node = NavigationManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
