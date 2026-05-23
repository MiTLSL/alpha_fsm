from __future__ import annotations

import json

from fsm_core.vacuum_model import VacuumPressureModel


class FailureInjectionMixin:
    def init_failure_injection(self) -> None:
        self._current_failure = "NONE"
        self._failure_reset_timer = None

    def create_inject_failure_service(self):
        from fsm_msgs.srv import InjectFailure

        return self.create_service(InjectFailure, "~/inject_failure", self.handle_inject_failure)

    def handle_inject_failure(self, request, response):
        self._current_failure = request.failure_name or "NONE"
        response.accepted = True
        response.message = f"failure set to {self._current_failure}"
        response.current_failure = self._current_failure

        if self._failure_reset_timer is not None:
            self._failure_reset_timer.cancel()
            self._failure_reset_timer = None
        if request.duration_sec > 0.0:
            self._failure_reset_timer = self.create_timer(float(request.duration_sec), self._reset_failure_once)
        return response

    def _reset_failure_once(self):
        self._current_failure = "NONE"
        if self._failure_reset_timer is not None:
            self._failure_reset_timer.cancel()
            self._failure_reset_timer = None


def json_details(**kwargs) -> str:
    return json.dumps(kwargs, ensure_ascii=False, sort_keys=True)


def make_pose_stamped(frame_id: str, x: float, y: float, z: float):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    return pose

