from __future__ import annotations

import math
import time


class FakeNav2BaseMixin:
    def _load_sim_config(self) -> None:
        from ament_index_python.packages import get_package_share_directory
        from pathlib import Path

        from fsm_core.ros2_helpers import declare_parameters_from_dict, load_yaml

        config_dir = Path(get_package_share_directory("fsm_config")) / "params"
        self.config.update(declare_parameters_from_dict(self, load_yaml(config_dir / "sim.yaml")))

    def _configure_fake_base(self) -> None:
        self._enabled = bool(self.config.get("sim.fake_base.enabled", False))
        self._map_frame = str(self.config.get("sim.frames.map", self.config.get("interfaces.frames.map", "map")))
        self._odom_frame = str(self.config.get("sim.fake_base.odom_frame", "odom"))
        self._base_frame = str(self.config.get("sim.frames.base_link", self.config.get("interfaces.frames.base_link", "base_link")))
        self._d2_base_frame = str(self.config.get("sim.frames.d2_base_link", self.config.get("interfaces.frames.d2_base_link", "2d_base_link")))
        self._alignment_topic = str(self.config.get("sim.fake_base.alignment_topic", "/sim/fake_base_alignment"))
        self._nav2_action_name = str(self.config.get("interfaces.actions.nav2_navigate_to_pose", "/nav2/navigate_to_pose"))
        self._desired_distance = float(
            self.config.get("sim.fake_base.desired_distance_to_wall", self.config.get("business.desired_distance_to_wall", 0.60))
        )
        self._desired_yaw = float(self.config.get("sim.fake_base.desired_yaw_to_wall", 0.0))
        self._dist_error = float(self.config.get("sim.fake_base.initial_position_offset_m", 0.10))
        self._yaw_error = float(self.config.get("sim.fake_base.initial_yaw_offset_rad", 0.0873))
        self._linear_deadband = float(self.config.get("sim.fake_base.command_deadband.linear_x", 0.0005))
        self._angular_deadband = float(self.config.get("sim.fake_base.command_deadband.angular_z", 0.0005))
        self._goal_duration_sec = float(self.config.get("sim.fake_base.nav2_goal_duration_sec", 0.30))
        self._integration_rate_hz = float(self.config.get("sim.fake_base.integration_rate_hz", 50.0))
        self._amcl_rate_hz = float(self.config.get("sim.fake_base.amcl_publish_rate_hz", 20.0))
        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._base_x = 0.0
        self._base_y = 0.0
        self._base_yaw = 0.0
        self._last_integrate_monotonic = time.monotonic()

    def _handle_lifecycle(self, request, response):
        del request
        response.success = True
        response.message = "fake navigation lifecycle active"
        return response

    def _handle_clear_costmap(self, request, response):
        del request
        return response

    def _on_align_cmd(self, msg) -> None:
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_angular_z = float(msg.angular.z)

    def _integrate(self) -> None:
        now = time.monotonic()
        dt = max(now - self._last_integrate_monotonic, 0.0)
        self._last_integrate_monotonic = now
        if not self._enabled:
            return

        linear = 0.0 if abs(self._cmd_linear_x) < self._linear_deadband else self._cmd_linear_x
        angular = 0.0 if abs(self._cmd_angular_z) < self._angular_deadband else self._cmd_angular_z
        self._dist_error = self._approach_zero(self._dist_error, linear * dt)
        self._yaw_error = self._approach_zero(self._yaw_error, angular * dt)
        self._base_x += linear * math.cos(self._base_yaw) * dt
        self._base_y += linear * math.sin(self._base_yaw) * dt
        self._base_yaw += angular * dt
        self._publish_alignment_state()
        self._publish_tf()

    @staticmethod
    def _approach_zero(error: float, correction: float) -> float:
        if error == 0.0 or correction == 0.0:
            return float(error)
        if error > 0.0:
            return max(0.0, error - abs(correction))
        return min(0.0, error + abs(correction))

    def _publish_alignment_state(self) -> None:
        from std_msgs.msg import Float32MultiArray

        msg = Float32MultiArray()
        measured_distance = self._desired_distance + self._dist_error
        measured_yaw = self._desired_yaw + self._yaw_error
        msg.data = [float(measured_distance), float(measured_yaw), float(self._dist_error), float(self._yaw_error)]
        self._alignment_pub.publish(msg)

    def _publish_amcl(self) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = self._base_x
        msg.pose.pose.position.y = self._base_y
        qx, qy, qz, qw = self._quaternion_from_yaw(self._base_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self._amcl_pub.publish(msg)

    def _publish_tf(self) -> None:
        from geometry_msgs.msg import TransformStamped

        stamp = self.get_clock().now().to_msg()
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self._map_frame
        map_to_odom.child_frame_id = self._odom_frame
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self._odom_frame
        odom_to_base.child_frame_id = self._base_frame
        odom_to_base.transform.translation.x = self._base_x
        odom_to_base.transform.translation.y = self._base_y
        qx, qy, qz, qw = self._quaternion_from_yaw(self._base_yaw)
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw

        map_to_d2_base = TransformStamped()
        map_to_d2_base.header.stamp = stamp
        map_to_d2_base.header.frame_id = self._map_frame
        map_to_d2_base.child_frame_id = self._d2_base_frame
        map_to_d2_base.transform.translation.x = self._base_x
        map_to_d2_base.transform.translation.y = self._base_y
        map_to_d2_base.transform.rotation.x = qx
        map_to_d2_base.transform.rotation.y = qy
        map_to_d2_base.transform.rotation.z = qz
        map_to_d2_base.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform([map_to_odom, odom_to_base, map_to_d2_base])

    async def _execute_nav2(self, goal_handle):
        from nav2_msgs.action import NavigateToPose

        request = goal_handle.request
        target = request.pose
        self._base_x = float(target.pose.position.x)
        self._base_y = float(target.pose.position.y)
        self._base_yaw = self._yaw_from_pose(target)
        self._dist_error = float(self.config.get("sim.fake_base.initial_position_offset_m", 0.10))
        self._yaw_error = float(self.config.get("sim.fake_base.initial_yaw_offset_rad", 0.0873))
        self._publish_alignment_state()

        steps = 5
        for index in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.current_pose = target
            feedback.distance_remaining = float(max(steps - index - 1, 0)) / float(steps)
            feedback.estimated_time_remaining.sec = 0
            feedback.estimated_time_remaining.nanosec = int(max(self._goal_duration_sec / steps, 0.0) * 1e9)
            goal_handle.publish_feedback(feedback)
            await self._sleep(max(self._goal_duration_sec / steps, 0.01))

        goal_handle.succeed()
        return NavigateToPose.Result()

    async def _sleep(self, duration_sec: float) -> None:
        from rclpy.task import Future

        future = Future()

        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(float(duration_sec), wake)
        await future

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        half = float(yaw) * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    @staticmethod
    def _yaw_from_pose(pose_stamped) -> float:
        q = pose_stamped.pose.orientation
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("fake_nav2_base_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_service_name, get_topic_name
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap
    from std_msgs.msg import Float32MultiArray
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster

    class FakeNav2BaseNode(SkeletonNodeMixin, FakeNav2BaseMixin, Node):
        def __init__(self):
            super().__init__("fake_nav2_base_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="FakeNav2Base")
            self._load_sim_config()
            self._configure_fake_base()
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "FakeNav2Base")
            self._tf_broadcaster = TransformBroadcaster(self)
            self._alignment_pub = self.create_publisher(Float32MultiArray, self._alignment_topic, 10)
            self._amcl_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                get_topic_name(self, "amcl_pose", "/amcl_pose"),
                10,
            )
            self._align_cmd_sub = self.create_subscription(
                Twist,
                get_topic_name(self, "cmd_vel_align", "/cmd_vel_align"),
                self._on_align_cmd,
                10,
            )
            self._nav2_server = ActionServer(self, NavigateToPose, self._nav2_action_name, self._execute_nav2)
            self._lifecycle_navigation_srv = self.create_service(
                Trigger,
                get_service_name(self, "lifecycle_manager_navigation_is_active", "/lifecycle_manager_navigation/is_active"),
                self._handle_lifecycle,
            )
            self._lifecycle_localization_srv = self.create_service(
                Trigger,
                get_service_name(self, "lifecycle_manager_localization_is_active", "/lifecycle_manager_localization/is_active"),
                self._handle_lifecycle,
            )
            self._local_costmap_srv = self.create_service(
                ClearEntireCostmap,
                get_service_name(self, "local_costmap_clear", "/local_costmap/clear_entirely_local_costmap"),
                self._handle_clear_costmap,
            )
            self._global_costmap_srv = self.create_service(
                ClearEntireCostmap,
                get_service_name(self, "global_costmap_clear", "/global_costmap/clear_entirely_global_costmap"),
                self._handle_clear_costmap,
            )
            self._integrate_timer = self.create_timer(1.0 / max(self._integration_rate_hz, 1.0), self._integrate)
            self._amcl_timer = self.create_timer(1.0 / max(self._amcl_rate_hz, 1.0), self._publish_amcl)
            self._publish_alignment_state()
            self._publish_tf()
            self.get_logger().info(
                f"fake_nav2_base_node ready action={self._nav2_action_name} "
                f"initial_error=({self._dist_error:.3f}, {self._yaw_error:.3f})"
            )

        def on_config_reloaded(self) -> None:
            self._load_sim_config()
            self._configure_fake_base()

    rclpy.init(args=args)
    node = FakeNav2BaseNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
