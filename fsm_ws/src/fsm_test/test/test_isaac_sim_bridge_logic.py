from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from isaac_sim_bridge.scene_truth import ViewFilter, filter_visible_boxes, load_box_truths, yaw_to_quaternion


class TestIsaacSimBridgeLogic(unittest.TestCase):
    def test_load_box_truths_from_json(self):
        boxes = load_box_truths(
            scene_file="",
            boxes_json=json.dumps(
                [
                    {
                        "id": "/World/Boxes/box_01",
                        "frame_id": "base_link",
                        "center": {"x": 0.8, "y": 0.1, "z": 1.2},
                        "size": {"length": 0.4, "width": 0.5, "height": 0.6},
                        "yaw": 0.2,
                    }
                ]
            ),
            default_frame="base_link",
            default_size=(0.4, 0.4, 0.4),
        )

        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0].detection_id, "World_Boxes_box_01")
        self.assertEqual(boxes[0].center, (0.8, 0.1, 1.2))
        self.assertEqual(boxes[0].size, (0.4, 0.5, 0.6))

    def test_load_box_truths_from_yaml_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "truth.yaml"
            path.write_text(
                """
boxes:
  - id: box_w0_r0_c0
    center: [0.7, 0.0, 1.0]
    size: [0.4, 0.4, 0.4]
""",
                encoding="utf-8",
            )
            boxes = load_box_truths(
                scene_file=str(path),
                boxes_json="",
                default_frame="base_link",
                default_size=(0.5, 0.5, 0.5),
            )

        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0].frame_id, "base_link")
        self.assertEqual(boxes[0].center, (0.7, 0.0, 1.0))

    def test_view_filter_keeps_only_visible_boxes(self):
        boxes = load_box_truths(
            scene_file="",
            boxes_json=json.dumps(
                [
                    {"id": "front", "center": {"x": 1.0, "y": 0.0, "z": 1.0}},
                    {"id": "far", "center": {"x": 5.0, "y": 0.0, "z": 1.0}},
                    {"id": "side", "center": {"x": 0.2, "y": 2.0, "z": 1.0}},
                    {"id": "high", "center": {"x": 1.0, "y": 0.0, "z": 5.0}},
                ]
            ),
            default_frame="base_link",
            default_size=(0.4, 0.4, 0.4),
        )

        visible = filter_visible_boxes(
            boxes,
            ViewFilter(max_distance_m=2.0, horizontal_fov_rad=math.radians(90.0), z_min=0.0, z_max=2.0),
        )

        self.assertEqual([box.detection_id for box in visible], ["front"])

    def test_yaw_to_quaternion(self):
        _, _, z, w = yaw_to_quaternion(math.pi)
        self.assertAlmostEqual(abs(z), 1.0)
        self.assertAlmostEqual(abs(w), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
