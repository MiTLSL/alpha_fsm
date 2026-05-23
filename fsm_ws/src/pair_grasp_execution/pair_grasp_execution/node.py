from __future__ import annotations


def _make_pose_stamped(frame_id: str = "base_link"):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


class PairGraspExecutionNodeMixin:
    def on_pressure_raw(self, msg):
        self._last_pressure_raw = [float(value) for value in msg.data[:2]]

    def publish_pressure_forward(self):
        from std_msgs.msg import Float32MultiArray

        msg = Float32MultiArray()
        msg.data = list(self._last_pressure_raw)
        self._vacuum_pressure_pub.publish(msg)

    async def execute_pair_grasp_goal(self, goal_handle):
        from fsm_core.error_code import ErrorCode
        from fsm_msgs.action import ExecutePairGrasp

        feedback = ExecutePairGrasp.Feedback()
        feedback.current_state = "NOT_IMPLEMENTED"
        feedback.current_stage = "PLAN"
        feedback.progress_percent = 0.0
        feedback.vacuum_left_kpa = 0.0
        feedback.vacuum_right_kpa = 0.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = ExecutePairGrasp.Result()
        result.success = False
        result.result.pair_id = goal_handle.request.grasp_pair.pair_id
        result.result.result_code = result.result.FAILED_BOTH
        result.result.left_result = 2
        result.result.right_result = 2
        result.result.failed_stage = "PLAN"
        result.result.error_code = int(ErrorCode.E_GRASP_UNKNOWN)
        result.result.final_robot_pose = _make_pose_stamped("base_link")
        return result


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("pair_grasp_execution_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_topic_name
    from fsm_msgs.action import ExecutePairGrasp
    from fsm_msgs.msg import VacuumCommand
    from std_msgs.msg import Float32MultiArray

    class PairGraspExecutionNode(SkeletonNodeMixin, PairGraspExecutionNodeMixin, Node):
        def __init__(self):
            super().__init__("pair_grasp_execution_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="PairGraspExecutionFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "PairGraspExecutionFSM")
            self._last_pressure_raw = [0.0, 0.0]
            self._action_server = ActionServer(
                self,
                ExecutePairGrasp,
                get_action_name(self, "execute_pair_grasp", "/execute_pair_grasp"),
                self.execute_pair_grasp_goal,
            )
            self._vacuum_cmd_pub = self.create_publisher(VacuumCommand, get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"), 10)
            self._vacuum_pressure_pub = self.create_publisher(Float32MultiArray, get_topic_name(self, "vacuum_pressure", "/vacuum/pressure"), 10)
            self._vacuum_pressure_sub = self.create_subscription(Float32MultiArray, get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"), self.on_pressure_raw, 10)
            self._pressure_forward_timer = self.create_timer(0.05, self.publish_pressure_forward)
            self.get_logger().info("pair_grasp_execution_node skeleton ready")

    rclpy.init(args=args)
    node = PairGraspExecutionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
