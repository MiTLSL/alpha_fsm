import importlib
import unittest


class TestMockModules(unittest.TestCase):
    def test_mock_entry_modules_import(self):
        modules = [
            "fsm_test.mocks.mock_perception_adapter_node",
            "fsm_test.mocks.mock_navigation_manager_node",
            "fsm_test.mocks.mock_pair_grasp_execution_node",
            "fsm_test.mocks.mock_vacuum_io_node",
            "fsm_test.mocks.mock_safety_button",
            "fsm_test.sim.sim_world_node",
            "fsm_test.sim.fake_nav2_base_node",
        ]
        for module_name in modules:
            module = importlib.import_module(module_name)
            self.assertTrue(callable(module.main))
