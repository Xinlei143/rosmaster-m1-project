"""Run the M1 controller with AMCL on real odometry and LaserScan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    localization_launch = os.path.join(bringup_share, "launch", "m1_localization.launch.py")
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": LaunchConfiguration("autostart"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "set_initial_pose": "false",
        }.items(),
    )
    controller = Node(
        package="imperative_navigation",
        executable="imperative_m1_controller",
        name="imperative_m1_controller",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_frame": "map",
            "global_frame": "map",
            "odom_frame": "odom",
            "global_tf_max_age": LaunchConfiguration("global_tf_max_age"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            "enabled": LaunchConfiguration("enabled"),
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "command_topic": "/imperative/cmd_vel_raw",
            "trajectory_planner_enabled": True,
        }],
    )
    watchdog = Node(
        package="m1_nav2_support",
        executable="m1_cmd_watchdog",
        name="imperative_localized_cmd_watchdog",
        output="screen",
        parameters=[{
            "input_topic": "/imperative/cmd_vel_raw",
            "output_topic": "/cmd_vel",
            "watchdog_timeout": 0.40,
            "publish_rate": 20.0,
        }],
    )
    arguments = [
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(
            bringup_share, "maps", "m1_baseline.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            bringup_share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("goal_x", default_value="1.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.08"),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("max_speed", default_value="0.18"),
        DeclareLaunchArgument("max_acceleration", default_value="0.25"),
        DeclareLaunchArgument("global_tf_max_age", default_value="0.5"),
    ]
    return LaunchDescription(arguments + [localization, controller, watchdog])
