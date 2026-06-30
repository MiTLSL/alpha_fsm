import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


_SRDF = (
    Path(__file__).resolve().parents[1]
    / "l3_sim_03_assets"
    / "moveit_config"
    / "config"
    / "alfa_robot.srdf"
)


class TestPairGraspSelfCollisionContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(_SRDF).getroot()

    def test_dual_arm_with_base_group_covers_both_arms_and_lift(self):
        group = self._group("dual_v5_arm_with_base")
        self.assertIsNotNone(group)
        self.assertEqual(self._joint_names(group), {"updown"})
        self.assertEqual(self._nested_group_names(group), {"left_v5_arm", "right_v5_arm"})

    def test_srdf_keeps_left_right_arm_collisions_enabled(self):
        disabled_pairs = self._disabled_collision_pairs()
        for left_index in range(1, 7):
            for right_index in range(1, 7):
                pair = frozenset({f"left_v5_link{left_index}", f"right_v5_link{right_index}"})
                self.assertNotIn(pair, disabled_pairs)

    def test_srdf_keeps_arm_vs_body_collision_enabled(self):
        disabled_pairs = self._disabled_collision_pairs()
        body_links = {"base_link", "pitch", "turn"}
        for side in ("left", "right"):
            for arm_index in range(1, 7):
                for body_link in body_links:
                    pair = frozenset({f"{side}_v5_link{arm_index}", body_link})
                    self.assertNotIn(pair, disabled_pairs)

    def _group(self, name):
        for group in self.root.findall("group"):
            if group.attrib.get("name") == name:
                return group
        return None

    @staticmethod
    def _joint_names(group):
        return {item.attrib["name"] for item in group.findall("joint")}

    @staticmethod
    def _nested_group_names(group):
        return {item.attrib["name"] for item in group.findall("group")}

    def _disabled_collision_pairs(self):
        pairs = set()
        for item in self.root.findall("disable_collisions"):
            pairs.add(frozenset({item.attrib["link1"], item.attrib["link2"]}))
        return pairs


if __name__ == "__main__":
    unittest.main()
