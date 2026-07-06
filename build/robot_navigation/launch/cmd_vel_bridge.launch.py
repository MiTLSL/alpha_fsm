from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("command_topic", default_value="/alfa/command_json"),
            DeclareLaunchArgument("linear_scale", default_value="1.0"),
            DeclareLaunchArgument("yaw_scale", default_value="1.0"),
            Node(
                package="robot_navigation",
                executable="cmd_vel_to_alfa_node",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "linear_scale": LaunchConfiguration("linear_scale"),
                        "yaw_scale": LaunchConfiguration("yaw_scale"),
                    }
                ],
                output="screen",
            ),
        ]
    )
