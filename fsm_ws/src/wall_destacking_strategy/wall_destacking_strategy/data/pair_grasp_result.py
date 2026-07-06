from __future__ import annotations

from dataclasses import dataclass

from .common import PoseData


@dataclass(frozen=True)
class PairGraspResult:
    pair_id: str
    result_code: int
    left_result: int
    right_result: int
    failed_stage: str = ""
    error_code: int = 0
    vacuum_left_kpa: float = 0.0
    vacuum_right_kpa: float = 0.0
    execution_time_sec: float = 0.0
    final_robot_pose: PoseData = PoseData.zero()

    @classmethod
    def from_msg(cls, msg) -> "PairGraspResult":
        return cls(
            pair_id=msg.pair_id,
            result_code=int(msg.result_code),
            left_result=int(msg.left_result),
            right_result=int(msg.right_result),
            failed_stage=msg.failed_stage,
            error_code=int(msg.error_code),
            vacuum_left_kpa=float(msg.vacuum_left_kpa),
            vacuum_right_kpa=float(msg.vacuum_right_kpa),
            execution_time_sec=float(msg.execution_time_sec),
            final_robot_pose=PoseData.from_msg(msg.final_robot_pose),
        )

    def to_msg(self):
        from fsm_msgs.msg import PairGraspResult as PairGraspResultMsg

        msg = PairGraspResultMsg()
        msg.pair_id = self.pair_id
        msg.result_code = int(self.result_code)
        msg.left_result = int(self.left_result)
        msg.right_result = int(self.right_result)
        msg.failed_stage = self.failed_stage
        msg.error_code = int(self.error_code)
        msg.vacuum_left_kpa = float(self.vacuum_left_kpa)
        msg.vacuum_right_kpa = float(self.vacuum_right_kpa)
        msg.execution_time_sec = float(self.execution_time_sec)
        msg.final_robot_pose = self.final_robot_pose.to_msg()
        return msg
