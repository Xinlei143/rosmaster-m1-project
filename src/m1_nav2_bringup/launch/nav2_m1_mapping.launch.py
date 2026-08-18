"""Run the M1 Gazebo world and synchronous slam_toolbox mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    imperative_share = get_package_share_directory("imperative_navigation")
    gazebo_launch = os.path.join(
        imperative_share, "launch", "imperative_m1_gazebo.launch.py")
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch", "online_sync_launch.py")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_saver_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
            "software_lidar": LaunchConfiguration("software_lidar"),
            "dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params,
        }.items(),
    )
    scan_relay = Node(
        package="m1_nav2_bringup",
        executable="scan_relay",
        name="m1_scan_relay",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
    )
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[map_saver_params],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            "node_names": ["map_saver"],
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_mapping_rviz",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[
            "-d", os.path.join(bringup_share, "rviz", "m1_nav2_mapping.rviz")],
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("software_lidar", default_value="false"),
        DeclareLaunchArgument(
            "dynamic_obstacles", default_value="false",
            description=(
                "Keep false when saving the baseline static map; enable only "
                "for explicit dynamic-obstacle mapping experiments.")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]
    return LaunchDescription(arguments + [
        gazebo,
        scan_relay,
        slam,
        map_saver,
        lifecycle_manager,
        rviz,
    ])
