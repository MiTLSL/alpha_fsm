from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration("auto_start")
    wait_for_clock = LaunchConfiguration("wait_for_clock")
    stage_time_scale = LaunchConfiguration("stage_time_scale")
    task_id = LaunchConfiguration("task_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("wait_for_clock", default_value="true"),
            DeclareLaunchArgument("stage_time_scale", default_value="1.0"),
            DeclareLaunchArgument("task_id", default_value="alfa_v7_nav_grasp_demo"),
            Node(
                package="robot_state_machine",
                executable="alfa_task_state_machine_node",
                name="alfa_task_state_machine_node",
                parameters=[
                    {
                        "auto_start": auto_start,
                        "wait_for_clock": wait_for_clock,
                        "stage_time_scale": stage_time_scale,
                        "task_id": task_id,
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
