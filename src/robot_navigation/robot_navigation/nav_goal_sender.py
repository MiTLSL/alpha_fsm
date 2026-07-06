from __future__ import annotations

import math

import rclpy
from fsm_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class NavGoalSender(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal_sender")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_type", "VALIDATION_GOAL")
        self.declare_parameter("x", -2.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("phase", 0)
        self.declare_parameter("desired_distance_to_wall", 0.8)
        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("require_fine_alignment", False)
        self._client = ActionClient(self, NavigateToPose, self.get_parameter("action_name").value)

    def send(self) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"action server not available: {self.get_parameter('action_name').value}")
            return False

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = str(self.get_parameter("frame_id").value)
        target.pose.position.x = float(self.get_parameter("x").value)
        target.pose.position.y = float(self.get_parameter("y").value)
        target.pose.position.z = float(self.get_parameter("z").value)
        qx, qy, qz, qw = _yaw_to_quaternion(float(self.get_parameter("yaw").value))
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.goal_type = str(self.get_parameter("goal_type").value)
        goal.target_pose = target
        goal.wall_frame_pose = target
        goal.phase = int(self.get_parameter("phase").value)
        goal.desired_distance_to_wall = float(self.get_parameter("desired_distance_to_wall").value)
        goal.desired_yaw_to_wall = float(self.get_parameter("yaw").value)
        goal.desired_lateral_offset = 0.0
        goal.require_fine_alignment = bool(self.get_parameter("require_fine_alignment").value)
        goal.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.get_logger().info(
            f"sending navigation goal x={target.pose.position.x:.3f} "
            f"y={target.pose.position.y:.3f} yaw={self.get_parameter('yaw').value}"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("navigation goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"navigation result success={result.success} "
            f"error_code={result.error_code} reason={result.failure_reason}"
        )
        return bool(result.success)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"nav feedback state={feedback.current_state} "
            f"distance={feedback.distance_remaining:.3f} "
            f"eta={feedback.estimated_time_remaining:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavGoalSender()
    exit_code = 1
    try:
        exit_code = 0 if node.send() else 2
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
