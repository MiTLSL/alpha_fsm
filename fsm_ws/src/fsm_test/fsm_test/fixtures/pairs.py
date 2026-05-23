from fsm_core.constants import GraspMode
from wall_destacking_strategy.data import GraspPair, HeaderData, PoseData, QuaternionData, Vector3Data


def standard_pair(pair_id: str = "task001_w0_lp_p0001") -> GraspPair:
    return GraspPair(
        pair_id=pair_id,
        task_id="task001",
        wall_index=0,
        phase=0,
        left_slot_id="wall_0_row_0_col_0",
        right_slot_id="wall_0_row_0_col_1",
        left_box_pose_robot=PoseData(
            header=HeaderData(frame_id="base_link"),
            position=Vector3Data(0.6, 0.20, 0.8),
            orientation=QuaternionData(),
        ),
        right_box_pose_robot=PoseData(
            header=HeaderData(frame_id="base_link"),
            position=Vector3Data(0.6, -0.20, 0.8),
            orientation=QuaternionData(),
        ),
        left_box_size=Vector3Data(0.4, 0.4, 0.4),
        right_box_size=Vector3Data(0.4, 0.4, 0.4),
        grasp_mode=GraspMode.DUAL,
    )
