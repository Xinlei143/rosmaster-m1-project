"""Launch the M1 Gazebo Sim scene, planner, and selectable LiDAR backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from imperative_navigation.scene_profiles import get_scene_profile


def _float_override(context, name, default):
    value = LaunchConfiguration(name).perform(context).strip()
    return float(default) if value.lower() == "auto" else float(value)


def _resolve_world(package_share, configured, default_name):
    configured = configured.strip()
    if configured.lower() in {"", "auto"}:
        configured = default_name
    if os.path.isabs(configured):
        return configured
    return os.path.join(package_share, "worlds", configured)


def build_runtime_actions(context):
    package_share = get_package_share_directory("imperative_navigation")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    scene = LaunchConfiguration("scene").perform(context).strip().lower()
    profile = get_scene_profile(scene)
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    world_name = profile["world_name"] if world_name.lower() in {"", "auto"} else world_name
    world = _resolve_world(package_share,
                           LaunchConfiguration("world_file").perform(context),
                           profile["world_file"])
    render_engine = LaunchConfiguration("render_engine").perform(context).strip() or "ogre"
    server_only = "-s " if LaunchConfiguration("gui").perform(context).lower() == "false" else ""

    start_x = _float_override(context, "start_x", profile["start"][0])
    start_y = _float_override(context, "start_y", profile["start"][1])
    start_z = _float_override(context, "start_z", profile["start"][2])
    start_yaw = _float_override(context, "start_yaw", profile["start"][3])
    goal_x = _float_override(context, "goal_x", profile["goal"][0])
    goal_y = _float_override(context, "goal_y", profile["goal"][1])
    pose_service = f"/world/{world_name}/set_pose"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"{server_only}-r -v 4 --render-engine {render_engine} {world}"
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_m1",
        output="screen",
        arguments=[
            "-topic", "/robot_description", "-name", "m1",
            "-x", str(start_x), "-y", str(start_y), "-z", str(start_z),
            "-Y", str(start_yaw),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/moving_obstacle_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/moving_obstacle_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/moving_obstacle_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        remappings=[
            (f"/world/{world_name}/dynamic_pose/info", "/imperative/gazebo_dynamic_tf"),
        ],
        output="screen",
    )

    software_lidar = Node(
        package="imperative_navigation",
        executable="software_lidar",
        name="software_lidar",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scene": scene,
            "scan_topic": "/sim_scan",
        }],
    )

    controller_parameters = {
        "use_sim_time": True,
        "command_topic": "/cmd_vel",
        "scan_topic": PythonExpression([
            "'/sim_scan' if '", LaunchConfiguration("software_lidar"),
            "' == 'true' else '/scan'",
        ]),
        "odom_topic": "/odom",
        "goal_x": goal_x,
        "goal_y": goal_y,
        "control_period": LaunchConfiguration("control_period"),
        "max_speed": LaunchConfiguration("max_speed"),
        "max_acceleration": LaunchConfiguration("max_acceleration"),
        "robot_radius": LaunchConfiguration("robot_radius"),
        "safety_margin": LaunchConfiguration("safety_margin"),
        "trajectory_planner_enabled": LaunchConfiguration("trajectory_planner_enabled"),
        "trajectory_horizon": LaunchConfiguration("trajectory_horizon"),
        "trajectory_heading_samples": LaunchConfiguration("trajectory_heading_samples"),
        "trajectory_speed_samples": LaunchConfiguration("trajectory_speed_samples"),
        "dynamic_obstacle_radius": LaunchConfiguration("dynamic_obstacle_radius"),
        "dynamic_obstacle_radii": profile["dynamic_radii"],
        "moving_confirmation_age": LaunchConfiguration("moving_confirmation_age"),
        "require_dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
    }
    if profile["static_fallback"]:
        controller_parameters["static_obstacles"] = profile["static_fallback"]

    controller = Node(
        package="imperative_navigation",
        executable="imperative_controller",
        name="imperative_controller",
        output="screen",
        parameters=[controller_parameters],
    )

    dynamic_obstacle_mover = Node(
        package="imperative_navigation",
        executable="dynamic_obstacle_mover",
        name="dynamic_obstacle_mover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scene": scene,
            "enabled": LaunchConfiguration("dynamic_obstacles"),
            "random_seed": LaunchConfiguration("dynamic_seed"),
            "motion_mode": LaunchConfiguration("dynamic_motion_mode"),
            "control_period": LaunchConfiguration("control_period"),
            "pose_service": pose_service,
            "gazebo_pose_topic": "/imperative/gazebo_dynamic_tf",
        }],
    )

    performance_recorder = Node(
        package="imperative_navigation",
        executable="avoidance_performance_recorder",
        name="avoidance_performance_recorder",
        condition=IfCondition(LaunchConfiguration("record_performance")),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scenario": PythonExpression([
                "'dynamic' if '", LaunchConfiguration("dynamic_obstacles"),
                "' == 'true' else 'static'",
            ]),
            "scene": scene,
            "output_dir": LaunchConfiguration("performance_output_dir"),
            "timeout": LaunchConfiguration("performance_timeout"),
            "goal_x": goal_x,
            "goal_y": goal_y,
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
            "scan_topic": PythonExpression([
                "'/sim_scan' if '", LaunchConfiguration("software_lidar"),
                "' == 'true' else '/scan'",
            ]),
            "dynamic_obstacle_radii": profile["dynamic_radii"],
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(package_share, "rviz", "imperative.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return [
        gazebo,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        dynamic_obstacle_mover,
        software_lidar,
        controller,
        performance_recorder,
        rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=performance_recorder,
            on_exit=[Shutdown(reason="avoidance benchmark finished")],
        )),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("imperative_navigation")
    description_share = get_package_share_directory("yahboomcar_description")
    description_resource_root = os.path.dirname(description_share)
    model_file = os.path.join(description_share, "urdf", "yahboomcar_M1_gazebo.urdf.xacro")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", model_file,
            " enable_gpu_lidar:=",
            PythonExpression([
                "'false' if '", LaunchConfiguration("software_lidar"),
                "' == 'true' else 'true'",
            ]),
        ]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    resource_paths = os.pathsep.join(filter(None, [
        os.path.join(package_share, "models"),
        description_resource_root,
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]))
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="imperative_m1",
                              description="Scene profile: imperative_m1, cafe, or house."),
        DeclareLaunchArgument("world_file", default_value="auto",
                              description="World filename or absolute path; auto follows scene."),
        DeclareLaunchArgument("world_name", default_value="auto",
                              description="Gazebo world name; auto follows scene."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine", default_value="ogre",
                              description="Gazebo render engine; default is ogre."),
        DeclareLaunchArgument(
            "software_lidar", default_value="false",
            description="Use /sim_scan software fallback; Linux formal tests use GPU /scan."),
        DeclareLaunchArgument("dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("dynamic_seed", default_value="20260814"),
        DeclareLaunchArgument("dynamic_motion_mode", default_value="auto",
                              description="auto, route, continuous, or random_waypoint."),
        DeclareLaunchArgument("start_x", default_value="auto"),
        DeclareLaunchArgument("start_y", default_value="auto"),
        DeclareLaunchArgument("start_z", default_value="auto"),
        DeclareLaunchArgument("start_yaw", default_value="auto"),
        DeclareLaunchArgument("goal_x", default_value="auto"),
        DeclareLaunchArgument("goal_y", default_value="auto"),
        DeclareLaunchArgument("record_performance", default_value="false"),
        DeclareLaunchArgument("performance_output_dir",
                              default_value="/tmp/imperative_m1_performance"),
        DeclareLaunchArgument("performance_timeout", default_value="120.0"),
        DeclareLaunchArgument("control_period", default_value="0.1"),
        DeclareLaunchArgument("max_speed", default_value="0.8"),
        DeclareLaunchArgument("max_acceleration", default_value="1.0"),
        DeclareLaunchArgument("robot_radius", default_value="0.15"),
        DeclareLaunchArgument("safety_margin", default_value="0.15"),
        DeclareLaunchArgument("trajectory_planner_enabled", default_value="true"),
        DeclareLaunchArgument("trajectory_horizon", default_value="20",
                              description="Rollout steps; 20 steps at 0.1 s = 2 s."),
        DeclareLaunchArgument("trajectory_heading_samples", default_value="41"),
        DeclareLaunchArgument("trajectory_speed_samples", default_value="4"),
        DeclareLaunchArgument("dynamic_obstacle_radius", default_value="0.20"),
        DeclareLaunchArgument("moving_confirmation_age", default_value="3"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_paths),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths),
        robot_state_publisher,
        OpaqueFunction(function=build_runtime_actions),
    ])
