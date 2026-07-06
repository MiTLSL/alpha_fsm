import math
import unittest
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from fsm_core.error_code import ErrorCode
from navigation_manager.nav_logic import (
    alignment_velocity,
    box_face_goal_pose,
    map_nav2_status_to_error,
    parse_chassis_diagnostics,
)


def _diagnostic(level=0, values=None, name="chassis", message=""):
    pairs = []
    for key, value in (values or {}).items():
        pairs.append(SimpleNamespace(key=key, value=value))
    return SimpleNamespace(level=level, name=name, message=message, values=pairs)


class TestNavigationManagerLogic(unittest.TestCase):
    def test_nav2_status_maps_to_fsm_errors(self):
        self.assertEqual(
            map_nav2_status_to_error(GoalStatus.STATUS_CANCELED, GoalStatus, ErrorCode),
            int(ErrorCode.E_NAV_GOAL_CANCELLED),
        )
        self.assertEqual(
            map_nav2_status_to_error(GoalStatus.STATUS_ABORTED, GoalStatus, ErrorCode),
            int(ErrorCode.E_NAV_PATH_PLAN_FAIL),
        )
        self.assertEqual(
            map_nav2_status_to_error(GoalStatus.STATUS_UNKNOWN, GoalStatus, ErrorCode),
            int(ErrorCode.E_NAV_UNKNOWN),
        )

    def test_chassis_diagnostics_parse_enabled_heartbeat_and_fault(self):
        ok = parse_chassis_diagnostics([
            _diagnostic(values={"enabled": "true", "heartbeat_ok": "true", "fault": "false"})
        ])
        self.assertTrue(ok.enabled)
        self.assertTrue(ok.heartbeat_ok)
        self.assertFalse(ok.fault)
        self.assertFalse(ok.stale)

        faulted = parse_chassis_diagnostics([
            _diagnostic(level=2, values={"enabled": "true", "heartbeat_ok": "true", "fault_a": "true"})
        ])
        self.assertTrue(faulted.fault)

    def test_alignment_velocity_applies_deadband_min_and_limits(self):
        self.assertEqual(
            alignment_velocity(
                0.005,
                0.01,
                linear_gain=1.0,
                angular_gain=1.0,
                max_linear_x=0.05,
                max_angular_z=0.2,
                dist_deadband=0.01,
                yaw_deadband=0.02,
            ),
            (0.0, 0.0),
        )
        linear, angular = alignment_velocity(
            0.02,
            -0.20,
            linear_gain=0.5,
            angular_gain=2.0,
            max_linear_x=0.05,
            max_angular_z=0.2,
            min_linear_x=0.02,
            min_angular_z=0.05,
        )
        self.assertAlmostEqual(linear, 0.02)
        self.assertAlmostEqual(angular, -0.2)

    def test_box_face_goal_pose_offsets_along_normal_and_faces_box(self):
        center = SimpleNamespace(x=1.0, y=2.0, z=0.0)
        normal = SimpleNamespace(x=1.0, y=0.0, z=0.0)
        goal = box_face_goal_pose(center, normal, 0.5, "map")
        self.assertAlmostEqual(goal.pose.position.x, 1.5)
        self.assertAlmostEqual(goal.pose.position.y, 2.0)
        yaw = 2.0 * math.atan2(goal.pose.orientation.z, goal.pose.orientation.w)
        self.assertAlmostEqual(abs(yaw), math.pi)


if __name__ == "__main__":
    unittest.main()
