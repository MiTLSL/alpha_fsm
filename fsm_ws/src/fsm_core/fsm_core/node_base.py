from __future__ import annotations

from typing import Any

from .ros2_helpers import declare_common_parameters, get_service_name, get_topic_name, make_qos_profile


class SkeletonNodeMixin:
    """ROS2 节点公共基础能力。

    继承类需要同时继承 rclpy.node.Node。本 mixin 只放 M1 基础能力：
    参数声明、reload_config、状态心跳、常用 QoS。
    """

    def init_fsm_node_base(self, ready_state: str = "READY", heartbeat_fsm_name: str = "NodeSkeleton") -> None:
        self.config: dict[str, Any] = declare_common_parameters(self)
        self._ready_state = ready_state
        self._heartbeat_fsm_name = heartbeat_fsm_name
        self._fsm_state_pub = None
        self._reload_srv = None
        self._heartbeat_timer = None

    def create_reload_service(self):
        from std_srvs.srv import Trigger

        service_name = get_service_name(self, "reload_config", "/reload_config")
        private_name = service_name.lstrip("/") or "reload_config"
        self._reload_srv = self.create_service(Trigger, f"~/{private_name}", self._on_reload_config)
        return self._reload_srv

    def create_state_heartbeat(self, topic_key: str, default_topic: str, fsm_name: str, rate_hz: float = 1.0):
        from fsm_msgs.msg import FsmStateSnapshot

        topic = get_topic_name(self, topic_key, default_topic)
        qos = make_qos_profile("RELIABLE", "TRANSIENT_LOCAL", 1)
        self._heartbeat_fsm_name = fsm_name
        self._fsm_state_pub = self.create_publisher(FsmStateSnapshot, topic, qos)
        period = 1.0 / max(float(rate_hz), 0.1)
        self._heartbeat_timer = self.create_timer(period, self.publish_state_heartbeat)
        return self._fsm_state_pub

    def publish_state_heartbeat(self) -> None:
        if self._fsm_state_pub is None:
            return
        from fsm_msgs.msg import FsmStateSnapshot

        msg = FsmStateSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.fsm_name = self._heartbeat_fsm_name
        msg.current_state = self._ready_state
        msg.extra_json = "{}"
        self._fsm_state_pub.publish(msg)

    def _on_reload_config(self, request, response):
        del request
        try:
            self.config = declare_common_parameters(self)
            hook = getattr(self, "on_config_reloaded", None)
            if callable(hook):
                hook()
            response.success = True
            response.message = "config reloaded"
        except Exception as exc:  # pragma: no cover - 防御性兜底
            response.success = False
            response.message = str(exc)
        return response
