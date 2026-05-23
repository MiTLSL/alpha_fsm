from __future__ import annotations

from dataclasses import dataclass

from fsm_core.constants import SlotStatus

from .common import PoseData, TimeData


@dataclass(frozen=True)
class GridSlot:
    slot_id: str
    wall_index: int
    row_index: int
    col_index: int
    status: int = SlotStatus.UNKNOWN
    expected_pose_robot: PoseData = PoseData.zero()
    latest_pose_robot: PoseData = PoseData.zero()
    visible: bool = False
    confidence: float = 0.0
    retry_count: int = 0
    last_seen_time: TimeData = TimeData()
    last_error_code: int = 0

    def row_major_index(self, cols: int = 5) -> int:
        return self.row_index * cols + self.col_index

    @classmethod
    def make(cls, task_id: str, wall_index: int, row: int, col: int, status: int = SlotStatus.OCCUPIED) -> "GridSlot":
        del task_id
        return cls(
            slot_id=f"wall_{wall_index}_row_{row}_col_{col}",
            wall_index=wall_index,
            row_index=row,
            col_index=col,
            status=status,
        )

    @classmethod
    def from_msg(cls, msg) -> "GridSlot":
        return cls(
            slot_id=msg.slot_id,
            wall_index=int(msg.wall_index),
            row_index=int(msg.row_index),
            col_index=int(msg.col_index),
            status=int(msg.status),
            expected_pose_robot=PoseData.from_msg(msg.expected_pose_robot),
            latest_pose_robot=PoseData.from_msg(msg.latest_pose_robot),
            visible=bool(msg.visible),
            confidence=float(msg.confidence),
            retry_count=int(msg.retry_count),
            last_seen_time=TimeData.from_msg(msg.last_seen_time),
            last_error_code=int(msg.last_error_code),
        )

    def to_msg(self):
        from fsm_msgs.msg import GridSlotState

        msg = GridSlotState()
        msg.slot_id = self.slot_id
        msg.wall_index = int(self.wall_index)
        msg.row_index = int(self.row_index)
        msg.col_index = int(self.col_index)
        msg.status = int(self.status)
        msg.expected_pose_robot = self.expected_pose_robot.to_msg()
        msg.latest_pose_robot = self.latest_pose_robot.to_msg()
        msg.visible = bool(self.visible)
        msg.confidence = float(self.confidence)
        msg.retry_count = int(self.retry_count)
        self.last_seen_time.fill_msg(msg.last_seen_time)
        msg.last_error_code = int(self.last_error_code)
        return msg
