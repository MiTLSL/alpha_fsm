import unittest

from fsm_core.ros2_helpers import declare_parameters_from_dict


class _Parameter:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self):
        from rclpy.exceptions import ParameterUninitializedException

        self._values = {
            "business.pair_grasp_execution.attached_box_touch_links": ParameterUninitializedException(
                "business.pair_grasp_execution.attached_box_touch_links"
            ),
            "business.pair_grasp_execution.backend_mode": "dry_run",
        }
        self.declared = []

    def has_parameter(self, name):
        return name in self._values

    def declare_parameter(self, name, default):
        self.declared.append((name, default))
        self._values[name] = default

    def get_parameter(self, name):
        value = self._values[name]
        if isinstance(value, Exception):
            raise value
        return _Parameter(value)


class TestRos2Helpers(unittest.TestCase):
    def test_uninitialized_empty_array_uses_yaml_default(self):
        node = _FakeNode()
        config = declare_parameters_from_dict(
            node,
            {
                "business": {
                    "pair_grasp_execution": {
                        "attached_box_touch_links": [],
                        "backend_mode": "dry_run",
                    }
                }
            },
        )

        self.assertEqual(config["business.pair_grasp_execution.attached_box_touch_links"], [])
        self.assertEqual(config["business.pair_grasp_execution.backend_mode"], "dry_run")
        self.assertEqual(node.declared, [])

