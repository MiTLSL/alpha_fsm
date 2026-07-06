import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
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
    mock_nav = LaunchConfiguration("mock_nav")
    mock_grasp = LaunchConfiguration("mock_grasp")
    mock_perception = LaunchConfiguration("mock_perception")
    mock_vacuum = LaunchConfiguration("mock_vacuum")

    return LaunchDescription([
        DeclareLaunchArgument("mock_nav", default_value="true"),
        DeclareLaunchArgument("mock_grasp", default_value="true"),
        DeclareLaunchArgument("mock_perception", default_value="true"),
        DeclareLaunchArgument("mock_vacuum", default_value="true"),
        Node(package="safety_monitor", executable="safety_monitor_node", parameters=common_params),
        Node(package="task_manager", executable="task_manager_node", parameters=common_params),
        Node(package="wall_destacking_strategy", executable="wall_destacking_strategy_node", parameters=common_params),
        Node(package="fsm_test", executable="mock_safety_button", parameters=common_params),
        Node(package="fsm_test", executable="mock_perception_adapter_node", parameters=common_params, condition=IfCondition(mock_perception)),
        Node(package="perception_adapter", executable="perception_adapter_node", parameters=common_params, condition=UnlessCondition(mock_perception)),
        Node(package="fsm_test", executable="mock_navigation_manager_node", parameters=common_params, condition=IfCondition(mock_nav)),
        Node(package="navigation_manager", executable="navigation_manager_node", parameters=common_params, condition=UnlessCondition(mock_nav)),
        Node(package="fsm_test", executable="mock_pair_grasp_execution_node", parameters=common_params, condition=IfCondition(mock_grasp)),
        Node(package="pair_grasp_execution", executable="pair_grasp_execution_node", parameters=common_params, condition=UnlessCondition(mock_grasp)),
        Node(package="fsm_test", executable="mock_vacuum_io_node", parameters=common_params, condition=IfCondition(mock_vacuum)),
        Node(package="vacuum_io", executable="vacuum_io_node", parameters=common_params, condition=UnlessCondition(mock_vacuum)),
    ])
