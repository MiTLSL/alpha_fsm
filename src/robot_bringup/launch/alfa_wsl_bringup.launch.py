import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration("auto_start")
    write_command_file = LaunchConfiguration("write_command_file")
    enable_cmd_vel_bridge = LaunchConfiguration("enable_cmd_vel_bridge")
    enable_state_summary = LaunchConfiguration("enable_state_summary")
    enable_visualization = LaunchConfiguration("enable_visualization")
    use_sim_time = LaunchConfiguration("use_sim_time")

    fsm_launch = os.path.join(
        get_package_share_directory("fsm_config"),
        "launch",
        "bringup_wsl_to_windows_isaaclab.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("write_command_file", default_value="false"),
            DeclareLaunchArgument("enable_cmd_vel_bridge", default_value="false"),
            DeclareLaunchArgument("enable_state_summary", default_value="true"),
            DeclareLaunchArgument("enable_visualization", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fsm_launch),
                launch_arguments={
                    "auto_start": auto_start,
                    "write_command_file": write_command_file,
                }.items(),
            ),
            Node(
                package="robot_state_machine",
                executable="state_summary_node",
                name="state_summary_node",
                condition=IfCondition(enable_state_summary),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="isaac_visualization_bridge",
                executable="validation_status_node",
                name="validation_status_node",
                condition=IfCondition(enable_visualization),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_navigation",
                executable="cmd_vel_to_alfa_node",
                name="cmd_vel_to_alfa_node",
                condition=IfCondition(enable_cmd_vel_bridge),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
