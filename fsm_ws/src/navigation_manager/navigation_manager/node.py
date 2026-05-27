from __future__ import annotations

from .backend import NavigationManagerNodeMixin


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.executors import MultiThreadedExecutor
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
            self.init_navigation_backend()
            self._action_server = ActionServer(
                self,
                NavigateToPose,
                get_action_name(self, "navigate_to_pose", "/navigate_to_pose"),
                self.execute_navigation_goal,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
                callback_group=self._io_callback_group,
            )
            self._recovery_srv = self.create_service(
                BaseRecoveryCommand,
                get_service_name(self, "nav_base_recovery", "/nav/base_recovery"),
                self.handle_base_recovery,
                callback_group=self._io_callback_group,
            )
            self._nav_health_pub = self.create_publisher(Bool, get_topic_name(self, "nav_health", "/fsm/nav_health"), 1)
            self._nav_health_timer = self.create_timer(1.0, self.publish_nav_health)
            self.get_logger().info(
                f"navigation_manager_node ready backend_mode={self._backend_mode} nav2_action={self._nav2_action_name}"
            )

    rclpy.init(args=args)
    node = NavigationManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
