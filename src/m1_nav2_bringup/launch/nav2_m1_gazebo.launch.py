"""Run the M1 Gazebo world with the Nav2 localization and navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
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
    params_file = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "m1_baseline.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    namespace = LaunchConfiguration("namespace")
    params_path = LaunchConfiguration("params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    localization_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map"),
            },
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
            "record_performance": LaunchConfiguration("record_performance"),
            "start_imperative_controller": "false",
        }.items(),
    )

    scan_relay = Node(
        package="m1_nav2_bringup",
        executable="scan_relay",
        name="m1_scan_relay",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[localization_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[localization_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"],
        }],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    )
    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "controller_server", "smoother_server", "planner_server",
                "behavior_server", "bt_navigator", "waypoint_follower",
                "velocity_smoother", "collision_monitor",
            ],
        }],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[configured_params],
    )
    watchdog = Node(
        package="imperative_navigation",
        executable="imperative_cmd_watchdog",
        name="imperative_cmd_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/imperative/cmd_vel_raw",
            "output_topic": "/cmd_vel",
            "watchdog_timeout": 0.50,
            "publish_rate": 20.0,
        }],
    )
    initial_pose = Node(
        package="m1_nav2_bringup",
        executable="initial_pose_publisher",
        name="m1_initial_pose_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "x": LaunchConfiguration("initial_pose_x"),
            "y": LaunchConfiguration("initial_pose_y"),
            "yaw": LaunchConfiguration("initial_pose_yaw"),
            "publish_rate": 1.0,
            "publish_count": 3,
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(bringup_share, "rviz", "m1_nav2.rviz")],
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("software_lidar", default_value="false"),
        DeclareLaunchArgument(
            "dynamic_obstacles", default_value="true",
            description="Enable moving obstacles for navigation tests."),
        DeclareLaunchArgument("record_performance", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("initial_pose_x", default_value="-2.5"),
        DeclareLaunchArgument("initial_pose_y", default_value="-1.5"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
    ]

    return LaunchDescription(arguments + [
        gazebo,
        scan_relay,
        map_server,
        amcl,
        localization_lifecycle,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle,
        collision_monitor,
        watchdog,
        TimerAction(period=6.0, actions=[initial_pose]),
        rviz,
    ])
