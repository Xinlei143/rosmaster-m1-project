"""Run Imperative with AMCL and configurable simulated odometry slip."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    support_share = get_package_share_directory("m1_nav2_support")
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    support_launch = os.path.join(support_share, "launch", "m1_gazebo.launch.py")
    localization_launch = os.path.join(bringup_share, "launch", "m1_localization.launch.py")

    gazebo = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
            "software_lidar": LaunchConfiguration("software_lidar"),
            "dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
            "dynamic_seed": LaunchConfiguration("dynamic_seed"),
            "dynamic_motion_mode": LaunchConfiguration("dynamic_motion_mode"),
            "render_engine": LaunchConfiguration("render_engine"),
            "dual_gpu_lidar": LaunchConfiguration("dual_gpu_lidar"),
            "gpu_lidar_min_angle": LaunchConfiguration("gpu_lidar_min_angle"),
            "gpu_lidar_max_angle": LaunchConfiguration("gpu_lidar_max_angle"),
            "slip_enabled": LaunchConfiguration("slip_enabled"),
            "slip_profile": LaunchConfiguration("slip_profile"),
            "slip_seed": LaunchConfiguration("slip_seed"),
            "odom_x_scale": LaunchConfiguration("odom_x_scale"),
            "odom_y_scale": LaunchConfiguration("odom_y_scale"),
            "odom_yaw_scale": LaunchConfiguration("odom_yaw_scale"),
            "odom_x_bias": LaunchConfiguration("odom_x_bias"),
            "odom_y_bias": LaunchConfiguration("odom_y_bias"),
            "odom_yaw_bias": LaunchConfiguration("odom_yaw_bias"),
            "odom_x_noise_std": LaunchConfiguration("odom_x_noise_std"),
            "odom_y_noise_std": LaunchConfiguration("odom_y_noise_std"),
            "odom_yaw_noise_std": LaunchConfiguration("odom_yaw_noise_std"),
            "odom_random_walk_xy_std": LaunchConfiguration("odom_random_walk_xy_std"),
            "odom_random_walk_yaw_std": LaunchConfiguration("odom_random_walk_yaw_std"),
            "burst_enabled": LaunchConfiguration("burst_enabled"),
            "burst_start": LaunchConfiguration("burst_start"),
            "burst_duration": LaunchConfiguration("burst_duration"),
            "burst_x_scale": LaunchConfiguration("burst_x_scale"),
            "burst_y_scale": LaunchConfiguration("burst_y_scale"),
            "burst_yaw_scale": LaunchConfiguration("burst_yaw_scale"),
        }.items(),
    )])
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": LaunchConfiguration("autostart"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "set_initial_pose": "true",
            "initial_pose_x": "-2.5",
            "initial_pose_y": "-1.5",
            "initial_pose_yaw": "0.0",
        }.items(),
    )
    controller = Node(
        package="imperative_navigation",
        executable="imperative_m1_controller",
        name="imperative_m1_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_frame": "map",
            "global_frame": "map",
            "odom_frame": "odom",
            "global_tf_max_age": LaunchConfiguration("global_tf_max_age"),
            "global_tf_future_tolerance": LaunchConfiguration("global_tf_future_tolerance"),
            "enabled": LaunchConfiguration("enabled"),
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "command_topic": "/imperative/cmd_vel_raw",
            "trajectory_planner_enabled": True,
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
        }],
    )
    scan_relay = Node(
        package="m1_nav2_bringup",
        executable="scan_relay",
        name="imperative_localized_scan_relay",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
    )
    watchdog = Node(
        package="m1_nav2_support",
        executable="m1_cmd_watchdog",
        name="imperative_localized_cmd_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/imperative/cmd_vel_raw",
            "output_topic": "/cmd_vel",
            "watchdog_timeout": 0.40,
            "publish_rate": 20.0,
        }],
    )
    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("software_lidar", default_value="false"),
        DeclareLaunchArgument("render_engine", default_value="ogre"),
        DeclareLaunchArgument("dual_gpu_lidar", default_value="true"),
        DeclareLaunchArgument("gpu_lidar_min_angle", default_value="-3.14159265359"),
        DeclareLaunchArgument("gpu_lidar_max_angle", default_value="3.14159265359"),
        DeclareLaunchArgument("dynamic_obstacles", default_value="false"),
        DeclareLaunchArgument("dynamic_seed", default_value="20260814"),
        DeclareLaunchArgument("dynamic_motion_mode", default_value="continuous"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(
            bringup_share, "maps", "m1_baseline.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            bringup_share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("goal_y", default_value="1.5"),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("max_speed", default_value="0.18"),
        DeclareLaunchArgument("max_acceleration", default_value="0.25"),
        DeclareLaunchArgument("global_tf_max_age", default_value="0.5"),
        DeclareLaunchArgument("global_tf_future_tolerance", default_value="0.5"),
        DeclareLaunchArgument("slip_enabled", default_value="true"),
        DeclareLaunchArgument("slip_profile", default_value="none"),
        DeclareLaunchArgument("slip_seed", default_value="20260902"),
        DeclareLaunchArgument("odom_x_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_y_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_yaw_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_x_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_y_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_yaw_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_x_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_y_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_random_walk_xy_std", default_value="0.0"),
        DeclareLaunchArgument("odom_random_walk_yaw_std", default_value="0.0"),
        DeclareLaunchArgument("burst_enabled", default_value="false"),
        DeclareLaunchArgument("burst_start", default_value="0.0"),
        DeclareLaunchArgument("burst_duration", default_value="0.0"),
        DeclareLaunchArgument("burst_x_scale", default_value="1.0"),
        DeclareLaunchArgument("burst_y_scale", default_value="1.0"),
        DeclareLaunchArgument("burst_yaw_scale", default_value="1.0"),
    ]
    return LaunchDescription(arguments + [gazebo, localization, scan_relay, controller, watchdog])
