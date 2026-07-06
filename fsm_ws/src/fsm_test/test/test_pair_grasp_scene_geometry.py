import unittest
from types import SimpleNamespace

from pair_grasp_execution.scene_geometry import (
    make_box_wall_opening_obstacles,
    make_container_obstacles,
    selected_box_object_ids,
    slot_indices,
)


def _pose(x, y, z):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="base_link"),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _size(x=0.4, y=0.4, z=0.4):
    return SimpleNamespace(x=x, y=y, z=z)


def _pair():
    return SimpleNamespace(
        MODE_DUAL=0,
        MODE_LEFT_ONLY=1,
        MODE_RIGHT_ONLY=2,
        pair_id="task001_w0_p0_0001",
        grasp_mode=0,
        left_slot_id="wall_0_row_0_col_1",
        right_slot_id="wall_0_row_0_col_3",
        left_box_pose_robot=_pose(0.6, 0.4, 1.8),
        right_box_pose_robot=_pose(0.6, -0.4, 1.8),
        left_box_size=_size(),
        right_box_size=_size(),
    )


class TestPairGraspSceneGeometry(unittest.TestCase):
    def test_slot_indices_parse_contract_slot_id(self):
        self.assertEqual(slot_indices("wall_2_row_4_col_3"), (4, 3))
        self.assertIsNone(slot_indices("box_7"))

    def test_selected_box_object_ids_are_stable(self):
        self.assertEqual(
            selected_box_object_ids(_pair()),
            ["box_wall_0_row_0_col_1", "box_wall_0_row_0_col_3"],
        )

    def test_container_obstacles_follow_old_motion_scene_shape(self):
        config = {
            "business.pair_grasp_execution.collision_scene.enable_container_obstacle": True,
            "business.pair_grasp_execution.collision_scene.container.width": 2.0,
            "business.pair_grasp_execution.collision_scene.container.height": 2.4,
            "business.pair_grasp_execution.collision_scene.container.length": 8.0,
            "business.pair_grasp_execution.collision_scene.container.wall_thickness": 0.04,
            "business.pair_grasp_execution.collision_scene.container.floor_z": 0.0,
        }
        obstacles = make_container_obstacles(config.get, "base_link")
        self.assertEqual([item.object_id for item in obstacles], ["container_left_wall", "container_right_wall", "container_ceiling"])
        self.assertEqual(obstacles[0].size, (8.0, 0.04, 2.4))
        self.assertAlmostEqual(obstacles[2].center[2], 2.42)

    def test_box_wall_opening_keeps_active_pair_holes(self):
        config = {
            "business.pair_grasp_execution.collision_scene.enable_static_box_wall_obstacles": True,
            "business.pair_grasp_execution.collision_scene.static_box_obstacle_inset": 0.002,
            "business.pair_grasp_execution.collision_scene.container.center_y": 0.0,
            "business.pair_grasp_execution.collision_scene.container.width": 2.2,
            "business.pair_grasp_execution.collision_scene.container.floor_z": 0.0,
        }
        obstacles = make_box_wall_opening_obstacles(_pair(), config.get, "base_link")
        ids = [item.object_id for item in obstacles]
        self.assertIn("task001_w0_p0_0001_static_wall_positive_y", ids)
        self.assertIn("task001_w0_p0_0001_static_wall_negative_y", ids)
        self.assertIn("task001_w0_p0_0001_static_wall_between", ids)
        self.assertIn("task001_w0_p0_0001_static_wall_below", ids)
        self.assertTrue(all(all(value > 0.0 for value in item.size) for item in obstacles))


if __name__ == "__main__":
    unittest.main()
