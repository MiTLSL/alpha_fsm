from __future__ import annotations

from dataclasses import dataclass

from .common import PoseData


@dataclass(frozen=True)
class NavResult:
    success: bool
    actual_base_pose: PoseData = PoseData.zero()
    position_error: float = 0.0
    yaw_error: float = 0.0
    alignment_error: float = 0.0
    workpose_valid: bool = False
    error_code: int = 0

    @classmethod
    def from_msg(cls, msg) -> "NavResult":
        return cls(
            success=bool(msg.success),
            actual_base_pose=PoseData.from_msg(msg.actual_base_pose),
            position_error=float(msg.position_error),
            yaw_error=float(msg.yaw_error),
            alignment_error=float(msg.alignment_error),
            workpose_valid=bool(msg.workpose_valid),
            error_code=int(msg.error_code),
        )

    def to_msg(self):
        from fsm_msgs.msg import NavResult as NavResultMsg

        msg = NavResultMsg()
        msg.success = bool(self.success)
        msg.actual_base_pose = self.actual_base_pose.to_msg()
        msg.position_error = float(self.position_error)
        msg.yaw_error = float(self.yaw_error)
        msg.alignment_error = float(self.alignment_error)
        msg.workpose_valid = bool(self.workpose_valid)
        msg.error_code = int(self.error_code)
        return msg
