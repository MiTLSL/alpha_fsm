from __future__ import annotations


class TaskManagerNodeMixin:
    def handle_task_control(self, request, response):
        self._task_state = {
            "start": "ACCEPT_TASK",
            "pause": "PAUSED",
            "resume": "RUN_TASK",
            "cancel": "CANCEL_TASK",
        }.get(request.command, self._task_state)
        if request.task_id:
            self._task_id = request.task_id
        response.accepted = request.command in ("start", "pause", "resume", "cancel")
        response.message = "accepted" if response.accepted else f"unsupported command: {request.command}"
        response.current_task_state = self._task_state
        return response

    def handle_clear_error(self, request, response):
        del request
        response.cleared = False
        response.message = "task_manager_node skeleton: clear_error protocol not implemented"
        response.stage_reached = 0
        return response

    def publish_task_state(self):
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = "TaskFSM"
        msg.current_state = self._task_state
        msg.task_id = self._task_id
        msg.extra_json = "{}"
        self._task_state_pub.publish(msg)


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("task_manager_node requires ROS2 rclpy") from exc

    from fsm_core.node_base import SkeletonNodeMixin
    from fsm_core.ros2_helpers import get_service_name, get_topic_name, make_qos_profile
    from fsm_msgs.msg import FsmStateSnapshot
    from fsm_msgs.srv import ClearError, TaskControl

    class TaskManagerNode(SkeletonNodeMixin, TaskManagerNodeMixin, Node):
        def __init__(self):
            super().__init__("task_manager_node")
            self.init_fsm_node_base(ready_state="STANDBY", heartbeat_fsm_name="RobotSystemFSM")
            self.create_reload_service()
            self.create_state_heartbeat("fsm_system_state", "/fsm/system_state", "RobotSystemFSM")
            self._task_id = ""
            self._task_state = "WAIT_TASK"
            qos = make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1)
            self._task_state_pub = self.create_publisher(FsmStateSnapshot, get_topic_name(self, "fsm_task_state", "/fsm/task_state"), qos)
            self._task_timer = self.create_timer(1.0, self.publish_task_state)
            self._task_start_srv = self.create_service(TaskControl, get_service_name(self, "task_start", "/task/start"), self.handle_task_control)
            self._task_pause_srv = self.create_service(TaskControl, get_service_name(self, "task_pause", "/task/pause"), self.handle_task_control)
            self._task_resume_srv = self.create_service(TaskControl, get_service_name(self, "task_resume", "/task/resume"), self.handle_task_control)
            self._task_cancel_srv = self.create_service(TaskControl, get_service_name(self, "task_cancel", "/task/cancel"), self.handle_task_control)
            self._clear_error_srv = self.create_service(ClearError, get_service_name(self, "clear_error", "/clear_error"), self.handle_clear_error)
            self.get_logger().info("task_manager_node skeleton ready")

    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
