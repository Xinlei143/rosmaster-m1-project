"""Launch only the independent physical command watchdog.

Keep this launch running in a dedicated terminal. Do not include it in the
planner launch: stopping the planner must not stop the process that protects
the hardware /cmd_vel topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("input_topic", default_value="/imperative/cmd_vel_raw"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("watchdog_timeout", default_value="0.60",
                              description="Maximum raw-command age [s]."),
        DeclareLaunchArgument("publish_rate", default_value="20.0",
                              description="Forward/zero publication rate [Hz]."),
    ]
    watchdog = Node(
        package="imperative_navigation",
        executable="imperative_cmd_watchdog",
        name="imperative_cmd_watchdog",
        output="screen",
        parameters=[{
            "input_topic": LaunchConfiguration("input_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
            "publish_rate": LaunchConfiguration("publish_rate"),
        }],
    )
    return LaunchDescription(arguments + [watchdog])
