import unittest

from fsm_core.constants import GraspMode
from wall_destacking_strategy.data import GraspPair, PoseData, Vector3Data, WallGrid


class TestDataModels(unittest.TestCase):
    def test_wall_grid_row_major_indices(self):
        grid = WallGrid.empty("task001", 0)
        self.assertEqual(len(grid.slots), 25)
        self.assertEqual(grid.slot_at(0, 0).slot_id, "wall_0_row_0_col_0")
        self.assertEqual(grid.slot_at(0, 4).row_major_index(), 4)
        self.assertEqual(grid.slot_at(4, 0).row_major_index(), 20)
        self.assertEqual(grid.slot_at(4, 4).row_major_index(), 24)

    def test_grasp_pair_single_arm_unused_side_is_zero(self):
        pair = GraspPair(
            pair_id="task001_w0_lp_p0001",
            task_id="task001",
            wall_index=0,
            phase=0,
            left_slot_id="wall_0_row_0_col_0",
            left_box_pose_robot=PoseData.zero(),
            left_box_size=Vector3Data(0.4, 0.4, 0.4),
            grasp_mode=GraspMode.LEFT_ONLY,
        )
        self.assertEqual(pair.right_slot_id, "")
        self.assertTrue(pair.right_box_pose_robot.is_zero())
        self.assertTrue(pair.right_box_size.is_zero())

    def test_grasp_pair_dual_requires_left_y_greater_than_right_y(self):
        with self.assertRaises(ValueError):
            GraspPair(
                pair_id="task001_w0_lp_p0002",
                task_id="task001",
                wall_index=0,
                phase=0,
                left_slot_id="left",
                right_slot_id="right",
                left_box_pose_robot=PoseData.zero(),
                right_box_pose_robot=PoseData.zero(),
                left_box_size=Vector3Data(0.4, 0.4, 0.4),
                right_box_size=Vector3Data(0.4, 0.4, 0.4),
                grasp_mode=GraspMode.DUAL,
            )
