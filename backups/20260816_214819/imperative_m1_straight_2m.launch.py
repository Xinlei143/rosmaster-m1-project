"""Run the full physical M1 planner toward a point two metres ahead."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("enabled", default_value="false",
                              description="Enable physical motion only after the safety check."),
        DeclareLaunchArgument("goal_distance", default_value="2.0",
                              description="Distance ahead of the first /odom pose [m]."),
        DeclareLaunchArgument("goal_tolerance", default_value="0.08",
                              description="Goal-arrival radius [m]."),
        DeclareLaunchArgument("max_speed", default_value="0.12",
                              description="Physical M1 speed cap [m/s]."),
        DeclareLaunchArgument("max_acceleration", default_value="0.20",
                              description="Physical M1 acceleration cap [m/s^2]."),
    ]
    controller = Node(
        package="imperative_navigation",
        executable="imperative_m1_controller",
        name="imperative_m1_controller",
        output="screen",
        parameters=[{
            "relative_goal_enabled": True,
            "goal_distance": LaunchConfiguration("goal_distance"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            "enabled": LaunchConfiguration("enabled"),
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "command_topic": "/imperative/cmd_vel_raw",
            "odom_frame": "odom",
        }],
    )
    return LaunchDescription(arguments + [controller])
