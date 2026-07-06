import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("fsm_config")
    common_params = [
        os.path.join(config_dir, "params", "business.yaml"),
        os.path.join(config_dir, "params", "fsm.yaml"),
        os.path.join(config_dir, "params", "interfaces.yaml"),
        os.path.join(config_dir, "params", "error_codes.yaml"),
        os.path.join(config_dir, "params", "logging.yaml"),
    ]
    return LaunchDescription([
        Node(package="wall_destacking_strategy", executable="wall_destacking_strategy_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_perception_adapter_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_navigation_manager_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_pair_grasp_execution_node", parameters=common_params),
    ])
