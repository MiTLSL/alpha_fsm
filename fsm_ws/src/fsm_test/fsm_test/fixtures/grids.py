from fsm_core.constants import SlotStatus
from wall_destacking_strategy.data import GridSlot, WallGrid


def standard_grid(task_id: str = "task001", wall_index: int = 0) -> WallGrid:
    return WallGrid.empty(task_id, wall_index)


def missing_corner_grid(task_id: str = "task001", wall_index: int = 0) -> WallGrid:
    grid = WallGrid.empty(task_id, wall_index)
    slots = list(grid.slots)
    slots[0] = GridSlot(
        slot_id=slots[0].slot_id,
        wall_index=slots[0].wall_index,
        row_index=slots[0].row_index,
        col_index=slots[0].col_index,
        status=SlotStatus.REMOVED,
        expected_pose_robot=slots[0].expected_pose_robot,
        latest_pose_robot=slots[0].latest_pose_robot,
        visible=False,
        confidence=0.0,
        retry_count=0,
        last_seen_time=slots[0].last_seen_time,
        last_error_code=0,
    )
    return WallGrid(
        task_id=grid.task_id,
        wall_index=grid.wall_index,
        rows=grid.rows,
        cols=grid.cols,
        wall_frame_pose=grid.wall_frame_pose,
        slots=tuple(slots),
        status=grid.status,
        header=grid.header,
    )


def single_column_grid(task_id: str = "task001", wall_index: int = 0) -> WallGrid:
    return WallGrid.empty(task_id, wall_index, rows=5, cols=1)
