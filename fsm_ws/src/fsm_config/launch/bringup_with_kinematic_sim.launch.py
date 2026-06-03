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
    config_dir = get_package_share_directory("fsm_config")
    common_params = _common_params()
    sim_params = common_params + [
        os.path.join(config_dir, "params", "sim.yaml"),
        {
            "sim.enabled": True,
            "sim.sensor.output_mode": LaunchConfiguration("sim_perception_output"),
        },
    ]

    start_core = LaunchConfiguration("start_core")
    use_real_perception_adapter = LaunchConfiguration("use_real_perception_adapter")
    use_fake_nav2 = LaunchConfiguration("use_fake_nav2")
    mock_nav = LaunchConfiguration("mock_nav")
    mock_grasp = LaunchConfiguration("mock_grasp")
    mock_vacuum = LaunchConfiguration("mock_vacuum")

    return LaunchDescription([
        DeclareLaunchArgument("start_core", default_value="true"),
        DeclareLaunchArgument("sim_perception_output", default_value="adapter_input"),
        DeclareLaunchArgument("use_real_perception_adapter", default_value="true"),
        DeclareLaunchArgument("use_fake_nav2", default_value="true"),
        DeclareLaunchArgument("mock_nav", default_value="true"),
        DeclareLaunchArgument("mock_grasp", default_value="true"),
        DeclareLaunchArgument("mock_vacuum", default_value="true"),
        Node(package="fsm_test", executable="sim_world_node", parameters=sim_params),
        Node(
            package="fsm_test",
            executable="fake_nav2_base_node",
            parameters=sim_params + [{"sim.fake_base.enabled": True}],
            condition=IfCondition(use_fake_nav2),
        ),
        Node(
            package="perception_adapter",
            executable="perception_adapter_node",
            parameters=common_params,
            condition=IfCondition(use_real_perception_adapter),
        ),
        Node(
            package="safety_monitor",
            executable="safety_monitor_node",
            parameters=common_params,
            condition=IfCondition(start_core),
        ),
        Node(
            package="task_manager",
            executable="task_manager_node",
            parameters=common_params,
            condition=IfCondition(start_core),
        ),
        Node(
            package="wall_destacking_strategy",
            executable="wall_destacking_strategy_node",
            parameters=common_params,
            condition=IfCondition(start_core),
        ),
        Node(package="fsm_test", executable="mock_navigation_manager_node", parameters=common_params, condition=IfCondition(mock_nav)),
        Node(package="navigation_manager", executable="navigation_manager_node", parameters=common_params, condition=UnlessCondition(mock_nav)),
        Node(package="fsm_test", executable="mock_pair_grasp_execution_node", parameters=common_params, condition=IfCondition(mock_grasp)),
        Node(package="pair_grasp_execution", executable="pair_grasp_execution_node", parameters=common_params, condition=UnlessCondition(mock_grasp)),
        Node(package="fsm_test", executable="mock_vacuum_io_node", parameters=common_params, condition=IfCondition(mock_vacuum)),
        Node(package="vacuum_io", executable="vacuum_io_node", parameters=common_params, condition=UnlessCondition(mock_vacuum)),
    ])
