import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _common_params():
    config_dir = get_package_share_directory("fsm_config")
    return [
        os.path.join(config_dir, "params", "business.yaml"),
        os.path.join(config_dir, "params", "fsm.yaml"),
        os.path.join(config_dir, "params", "interfaces.yaml"),
        os.path.join(config_dir, "params", "error_codes.yaml"),
        os.path.join(config_dir, "params", "logging.yaml"),
    ]


def generate_launch_description():
    common_params = _common_params()
    return LaunchDescription([
        Node(package="safety_monitor", executable="safety_monitor_node", parameters=common_params),
        Node(package="task_manager", executable="task_manager_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_navigation_manager_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_safety_button", parameters=common_params),
    ])
