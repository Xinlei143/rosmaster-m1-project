"""Run Nav2 against a physical Rosmaster M1 driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    description_share = get_package_share_directory("yahboomcar_description")
    params_file = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "m1_baseline.yaml")
    model_file = os.path.join(
        description_share, "urdf", "yahboomcar_M1.urdf.xacro")

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
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model_file]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        condition=IfCondition(
            LaunchConfiguration("publish_robot_description")),
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
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
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[configured_params],
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
        condition=IfCondition(LaunchConfiguration("publish_initial_pose")),
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
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "publish_robot_description", default_value="true",
            description=(
                "Publish the M1 URDF fixed transforms. Disable only if the "
                "vendor driver already publishes the same "
                "robot_description TF.")),
        DeclareLaunchArgument(
            "publish_initial_pose", default_value="false",
            description=(
                "Publish a known initial pose. Keep false for an unknown real "
                "robot pose and initialize with RViz 2D Pose Estimate.")),
        DeclareLaunchArgument(
            "set_initial_pose", default_value="false",
            description=(
                "Use the YAML initial_pose as AMCL's startup pose. Keep false "
                "unless the physical robot starts at that known pose.")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
    ]

    return LaunchDescription(arguments + [
        robot_state_publisher,
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
        collision_monitor,
        navigation_lifecycle,
        watchdog,
        TimerAction(period=3.0, actions=[initial_pose]),
        rviz,
    ])
