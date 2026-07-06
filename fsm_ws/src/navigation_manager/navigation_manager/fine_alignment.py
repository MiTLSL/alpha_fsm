from __future__ import annotations

import math
import time

from .geometry import angle_delta, yaw_from_pose
from .nav_logic import alignment_velocity


class FineAlignmentMixin:
    def on_box_detections(self, msg):
        self._last_box_detections = [
            det
            for det in msg.detections
            if bool(det.pose_valid) and float(det.confidence) >= self._fine_align_min_detection_confidence
        ]
        self._last_detection_monotonic = time.monotonic()

    async def _run_fine_alignment(self, goal_handle, request) -> tuple[bool, int, str, float]:
        from fsm_core.error_code import ErrorCode

        deadline = time.monotonic() + max(self._fine_align_timeout_sec, 0.1)
        pass_count = 0
        final_alignment_error = float("nan")

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._publish_zero_align_velocity()
                return False, int(ErrorCode.E_NAV_GOAL_CANCELLED), "cancelled during fine alignment", final_alignment_error
            if getattr(self, "_estop", False):
                self._publish_zero_align_velocity()
                return False, int(ErrorCode.E_SAFETY_ESTOP_HW), "estop during fine alignment", final_alignment_error
            if not self._localization_ok():
                self._publish_zero_align_velocity()
                return False, int(ErrorCode.E_NAV_LOCALIZATION_LOST), "localization lost during fine alignment", final_alignment_error
            if not self._chassis_ok():
                self._publish_zero_align_velocity()
                return False, int(ErrorCode.E_NAV_STUCK), "chassis fault during fine alignment", final_alignment_error

            measurement = self._alignment_measurement(request)
            if measurement is None:
                self._publish_feedback(goal_handle, "FINE_ALIGN", distance=0.0, eta=max(deadline - time.monotonic(), 0.0))
                if time.monotonic() - self._last_detection_monotonic > self._fine_align_feedback_timeout_sec:
                    self._publish_zero_align_velocity()
                    return False, int(ErrorCode.E_NAV_FINE_ALIGN_NO_FEEDBACK), "no fresh perception feedback for fine alignment", final_alignment_error
                await self._sleep(0.05)
                continue

            dist_error, yaw_error = measurement
            final_alignment_error = math.sqrt(dist_error * dist_error + yaw_error * yaw_error)
            self._publish_feedback(
                goal_handle,
                "FINE_ALIGN",
                distance=abs(dist_error),
                eta=max(deadline - time.monotonic(), 0.0),
                alignment_error=final_alignment_error,
            )

            if abs(dist_error) <= self._fine_align_dist_tolerance and abs(yaw_error) <= self._fine_align_yaw_tolerance:
                pass_count += 1
                self._publish_zero_align_velocity()
                if pass_count >= max(self._fine_align_pass_frames, 1):
                    return True, 0, "", final_alignment_error
            else:
                pass_count = 0
                self._publish_align_velocity(dist_error, yaw_error)

            await self._sleep(0.05)

        self._publish_zero_align_velocity()
        return False, int(ErrorCode.E_NAV_FINE_ALIGN_FAIL), "fine alignment timeout", final_alignment_error

    def _alignment_measurement(self, request):
        if not self._last_box_detections:
            return None
        if time.monotonic() - self._last_detection_monotonic > self._fine_align_feedback_timeout_sec:
            return None

        detections = sorted(
            self._last_box_detections,
            key=lambda det: (abs(float(det.pose.pose.position.y) - float(request.desired_lateral_offset)), -float(det.confidence)),
        )
        sample = detections[: min(len(detections), 5)]
        if not sample:
            return None

        distance = sum(float(det.pose.pose.position.x) for det in sample) / len(sample)
        yaw_values = [yaw_from_pose(det.pose) for det in sample]
        yaw_sin = sum(math.sin(value) for value in yaw_values)
        yaw_cos = sum(math.cos(value) for value in yaw_values)
        measured_yaw = math.atan2(yaw_sin, yaw_cos)
        dist_error = distance - float(request.desired_distance_to_wall)
        yaw_error = angle_delta(measured_yaw, float(request.desired_yaw_to_wall))
        return dist_error, yaw_error

    def _publish_align_velocity(self, dist_error: float, yaw_error: float) -> None:
        from geometry_msgs.msg import Twist

        msg = Twist()
        linear, angular = alignment_velocity(
            dist_error,
            yaw_error,
            linear_gain=self._fine_align_linear_gain,
            angular_gain=self._fine_align_angular_gain,
            max_linear_x=self._fine_align_max_linear_x,
            max_angular_z=self._fine_align_max_angular_z,
            min_linear_x=self._fine_align_min_linear_x,
            min_angular_z=self._fine_align_min_angular_z,
            dist_deadband=self._fine_align_dist_tolerance * 0.5,
            yaw_deadband=self._fine_align_yaw_tolerance * 0.5,
        )
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_align_pub.publish(msg)

    def _publish_zero_align_velocity(self) -> None:
        from geometry_msgs.msg import Twist

        for _ in range(max(int(getattr(self, "_fine_align_stop_repeats", 1)), 1)):
            self._cmd_vel_align_pub.publish(Twist())
