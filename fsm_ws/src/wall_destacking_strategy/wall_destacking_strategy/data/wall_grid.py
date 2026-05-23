from __future__ import annotations

from dataclasses import dataclass, field

from fsm_core.constants import SlotStatus

from .common import HeaderData, PoseData
from .grid_slot import GridSlot


@dataclass(frozen=True)
class WallGrid:
    task_id: str
    wall_index: int
    rows: int = 5
    cols: int = 5
    wall_frame_pose: PoseData = PoseData.zero()
    slots: tuple[GridSlot, ...] = field(default_factory=tuple)
    status: int = 0
    header: HeaderData = HeaderData(frame_id="base_link")

    def __post_init__(self) -> None:
        expected = self.rows * self.cols
        if self.slots and len(self.slots) != expected:
            raise ValueError(f"WallGrid slots length must be {expected}, got {len(self.slots)}")
        for index, slot in enumerate(self.slots):
            if slot.row_major_index(self.cols) != index:
                raise ValueError(f"slot {slot.slot_id} is not row-major at index {index}")

    @classmethod
    def empty(cls, task_id: str, wall_index: int, rows: int = 5, cols: int = 5) -> "WallGrid":
        slots = tuple(
            GridSlot.make(task_id, wall_index, row, col, status=SlotStatus.OCCUPIED)
            for row in range(rows)
            for col in range(cols)
        )
        return cls(task_id=task_id, wall_index=wall_index, rows=rows, cols=cols, slots=slots)

    def slot_at(self, row: int, col: int) -> GridSlot:
        return self.slots[row * self.cols + col]

    @classmethod
    def from_msg(cls, msg) -> "WallGrid":
        return cls(
            task_id=msg.task_id,
            wall_index=int(msg.wall_index),
            rows=int(msg.rows),
            cols=int(msg.cols),
            wall_frame_pose=PoseData.from_msg(msg.wall_frame_pose),
            slots=tuple(GridSlot.from_msg(slot) for slot in msg.slots),
            status=int(msg.status),
            header=HeaderData.from_msg(msg.header),
        )

    def to_msg(self):
        from fsm_msgs.msg import WallGridSnapshot

        msg = WallGridSnapshot()
        self.header.fill_msg(msg.header)
        msg.task_id = self.task_id
        msg.wall_index = int(self.wall_index)
        msg.rows = int(self.rows)
        msg.cols = int(self.cols)
        msg.wall_frame_pose = self.wall_frame_pose.to_msg()
        msg.slots = [slot.to_msg() for slot in self.slots]
        msg.status = int(self.status)
        return msg
