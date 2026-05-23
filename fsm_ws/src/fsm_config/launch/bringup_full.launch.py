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
        Node(package="wall_destacking_strategy", executable="wall_destacking_strategy_node", parameters=common_params),
        Node(package="perception_adapter", executable="perception_adapter_node", parameters=common_params),
        Node(package="navigation_manager", executable="navigation_manager_node", parameters=common_params),
        Node(package="vacuum_io", executable="vacuum_io_node", parameters=common_params),
        Node(package="pair_grasp_execution", executable="pair_grasp_execution_node", parameters=common_params),
    ])
