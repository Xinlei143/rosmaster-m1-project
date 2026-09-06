"""Run the restored Imperative controller on the current M1 Gazebo scene."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    support_share = get_package_share_directory("m1_nav2_support")
    support_launch = os.path.join(support_share, "launch", "m1_gazebo.launch.py")

    gazebo = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
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
            }.items(),
        )],
    )

    controller = Node(
        package="imperative_navigation",
        executable="imperative_controller",
        name="imperative_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "command_topic": "/cmd_vel",
            "scan_topic": PythonExpression([
                "'/sim_scan' if '", LaunchConfiguration("software_lidar"),
                "' == 'true' else '/scan'",
            ]),
            "odom_topic": "/odom",
            "goal_x": 2.5,
            "goal_y": 1.5,
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
            "trajectory_planner_enabled": LaunchConfiguration("trajectory_planner_enabled"),
            "trajectory_horizon": LaunchConfiguration("trajectory_horizon"),
            "trajectory_heading_samples": LaunchConfiguration("trajectory_heading_samples"),
            "trajectory_speed_samples": LaunchConfiguration("trajectory_speed_samples"),
            "dynamic_obstacle_radius": LaunchConfiguration("dynamic_obstacle_radius"),
            "dynamic_obstacles_topic": "/m1/dynamic_obstacles",
            "require_dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
            # These coordinates match the static cylinders in the current
            # m1_nav2_support/worlds/m1.sdf. They are used only if GPU LiDAR
            # produces a saturated frame.
            "static_obstacles": [0.0, 0.0, 0.3, -3.0, 2.2, 0.3],
        }],
    )

    # Keep RViz on the repository-wide /scan topic. The relay is only active
    # for the software-lidar fallback; the controller itself still consumes
    # /sim_scan in that mode as an explicit compatibility check.
    scan_relay = Node(
        package="m1_nav2_bringup",
        executable="scan_relay",
        name="imperative_scan_relay",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="imperative_rviz",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(
            get_package_share_directory("imperative_navigation"),
            "rviz", "imperative.rviz")],
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("software_lidar", default_value="false"),
        DeclareLaunchArgument("render_engine", default_value="ogre"),
        DeclareLaunchArgument("dual_gpu_lidar", default_value="true"),
        DeclareLaunchArgument("gpu_lidar_min_angle", default_value="-3.14159265359"),
        DeclareLaunchArgument("gpu_lidar_max_angle", default_value="3.14159265359"),
        DeclareLaunchArgument("dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("dynamic_seed", default_value="20260814"),
        DeclareLaunchArgument("dynamic_motion_mode", default_value="continuous"),
        DeclareLaunchArgument("max_speed", default_value="1.0"),
        DeclareLaunchArgument("max_acceleration", default_value="1.0"),
        DeclareLaunchArgument("robot_radius", default_value="0.15"),
        DeclareLaunchArgument("safety_margin", default_value="0.15"),
        DeclareLaunchArgument("trajectory_planner_enabled", default_value="true"),
        DeclareLaunchArgument(
            "trajectory_horizon", default_value="20",
            description="20 rollout steps at 0.1 s give a two-second preview."),
        DeclareLaunchArgument("trajectory_heading_samples", default_value="41"),
        DeclareLaunchArgument("trajectory_speed_samples", default_value="4"),
        DeclareLaunchArgument("dynamic_obstacle_radius", default_value="0.20"),
    ]

    return LaunchDescription(arguments + [gazebo, scan_relay, controller, rviz])
