from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration("auto_start")
    wait_for_clock = LaunchConfiguration("wait_for_clock")
    task_id = LaunchConfiguration("task_id")
    target_cargo_name = LaunchConfiguration("target_cargo_name")
    command_rate_hz = LaunchConfiguration("command_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("wait_for_clock", default_value="true"),
            DeclareLaunchArgument("task_id", default_value="alfa_truth_pick_return_demo"),
            DeclareLaunchArgument("target_cargo_name", default_value=""),
            DeclareLaunchArgument("command_rate_hz", default_value="1.0"),
            Node(
                package="robot_state_machine",
                executable="alfa_truth_demo_state_machine_node",
                name="alfa_truth_demo_state_machine_node",
                parameters=[
                    {
                        "auto_start": auto_start,
                        "wait_for_clock": wait_for_clock,
                        "task_id": task_id,
                        "target_cargo_name": target_cargo_name,
                        "command_rate_hz": command_rate_hz,
                    }
                ],
                output="screen",
            ),
            Node(
                package="isaac_visualization_bridge",
                executable="validation_status_node",
                name="validation_status_node",
                output="screen",
            ),
        ]
    )
