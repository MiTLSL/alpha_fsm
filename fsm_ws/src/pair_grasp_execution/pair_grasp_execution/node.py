from __future__ import annotations

from .backend import PairGraspExecutionNodeMixin


def main(args=None):
    try:
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("pair_grasp_execution_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_action_name, get_topic_name
    from fsm_msgs.action import ExecutePairGrasp
    from fsm_msgs.msg import VacuumCommand
    from std_msgs.msg import Bool, Float32MultiArray

    class PairGraspExecutionNode(SkeletonNodeMixin, PairGraspExecutionNodeMixin, Node):
        def __init__(self):
            super().__init__("pair_grasp_execution_node")
            self.init_fsm_node_base(ready_state="READY", heartbeat_fsm_name="PairGraspExecutionFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_active_substate", "/fsm/active_substate", "PairGraspExecutionFSM")
            self._last_pressure_raw = [0.0, 0.0]
            self._estop = False
            self.init_grasp_backend()
            self._action_server = ActionServer(
                self,
                ExecutePairGrasp,
                get_action_name(self, "execute_pair_grasp", "/execute_pair_grasp"),
                self.execute_pair_grasp_goal,
                goal_callback=self.handle_goal,
                cancel_callback=self.handle_cancel,
                callback_group=self._io_callback_group,
            )
            self._vacuum_cmd_pub = self.create_publisher(VacuumCommand, get_topic_name(self, "vacuum_cmd", "/vacuum/cmd"), 10)
            self._vacuum_pressure_pub = self.create_publisher(Float32MultiArray, get_topic_name(self, "vacuum_pressure", "/vacuum/pressure"), 10)
            self._vacuum_pressure_sub = self.create_subscription(
                Float32MultiArray,
                get_topic_name(self, "vacuum_pressure_raw", "/vacuum/pressure_raw"),
                self.on_pressure_raw,
                10,
                callback_group=self._io_callback_group,
            )
            self._estop_sub = self.create_subscription(
                Bool,
                get_topic_name(self, "safety_estop", "/safety/estop"),
                self.on_estop,
                10,
                callback_group=self._io_callback_group,
            )
            self._pressure_forward_timer = self.create_timer(0.05, self.publish_pressure_forward)
            self.get_logger().info(
                f"pair_grasp_execution_node ready backend_mode={self._backend_mode} moveit_action={self._moveit_action_name}"
            )

    rclpy.init(args=args)
    node = PairGraspExecutionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
