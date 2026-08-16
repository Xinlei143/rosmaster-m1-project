"""Launch the M1 Gazebo Sim scene, ROS bridges, planner, and RViz."""

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


def launch_gazebo(context):
    package_share = get_package_share_directory("imperative_navigation")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world = os.path.join(package_share, "worlds", "imperative_m1.sdf")
    server_only = "-s " if LaunchConfiguration("gui").perform(context).lower() == "false" else ""

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        # OGRE is the stable renderer for this Gazebo 6 + WSLg installation.
        # See the controller's static_obstacles fallback for its GPU-LiDAR
        # compatibility behavior on this renderer.
        launch_arguments={"gz_args": f"{server_only}-r -v 4 --render-engine ogre {world}"}.items(),
    )]


def generate_launch_description():
    package_share = get_package_share_directory("imperative_navigation")
    description_share = get_package_share_directory("yahboomcar_description")
    description_resource_root = os.path.dirname(description_share)
    model_file = os.path.join(
        description_share, "urdf", "yahboomcar_M1_gazebo.urdf.xacro")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", model_file,
            " enable_gpu_lidar:=",
            PythonExpression([
                "'false' if '", LaunchConfiguration("software_lidar"),
                "' == 'true' else 'true'"]),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_m1",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "m1",
            "-x", "-2.5",
            "-y", "-1.5",
            "-z", "0.01",
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
            "/world/imperative_m1/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/imperative_m1/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        remappings=[
            ("/world/imperative_m1/dynamic_pose/info", "/imperative/gazebo_dynamic_tf"),
        ],
        output="screen",
    )

    controller = Node(
        package="imperative_navigation",
        executable="imperative_controller",
        name="imperative_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "command_topic": "/cmd_vel",
            "scan_topic": PythonExpression(["'/sim_scan' if '", LaunchConfiguration("software_lidar"), "' == 'true' else '/scan'"]),
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
            "moving_confirmation_age": LaunchConfiguration("moving_confirmation_age"),
            # Matches the static cylinders in imperative_m1.sdf.  This is used
            # only when the WSLg/OGRE GPU-LiDAR bug saturates every beam.
            "static_obstacles": [-2.05, -1.18, 0.3, -3.0, 2.2, 0.3],
            "require_dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
        }],
    )

    dynamic_obstacle_mover = Node(
        package="imperative_navigation",
        executable="dynamic_obstacle_mover",
        name="dynamic_obstacle_mover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "enabled": LaunchConfiguration("dynamic_obstacles"),
            "random_seed": LaunchConfiguration("dynamic_seed"),
            "motion_mode": LaunchConfiguration("dynamic_motion_mode"),
        }],
    )

    software_lidar = Node(
        package="imperative_navigation",
        executable="software_lidar",
        name="software_lidar",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(package_share, "rviz", "imperative.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
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
            "output_dir": LaunchConfiguration("performance_output_dir"),
            "timeout": LaunchConfiguration("performance_timeout"),
            "goal_x": 2.5,
            "goal_y": 1.5,
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
        }],
    )

    # Gazebo Sim resolves package:// mesh URIs as model:// package paths.  The
    # parent of the ament share directory is therefore part of its resource
    # search path so the copied M1 STL meshes can be loaded after spawning.
    ignition_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]))
    gz_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo GUI."),
        DeclareLaunchArgument(
            "software_lidar", default_value="false",
            description=(
                "Use the software fallback LaserScan. Set true if the GPU lidar "
                "is unavailable on this WSLg installation.")),
        DeclareLaunchArgument(
            "dynamic_obstacles", default_value="true",
            description="Move dynamic obstacles; false parks them outside the room."),
        DeclareLaunchArgument(
            "dynamic_seed", default_value="20260814",
            description="Deterministic seed for the moving-obstacle scenario."),
        DeclareLaunchArgument(
            "dynamic_motion_mode", default_value="continuous",
            description=(
                "Obstacle motion: continuous smooth-random motion or "
                "random_waypoint for the legacy constant-speed patrol.")),
        DeclareLaunchArgument(
            "record_performance", default_value="false",
            description="Record command and odometry motion until goal or timeout."),
        DeclareLaunchArgument(
            "performance_output_dir", default_value="/tmp/imperative_m1_performance",
            description="Empty/new directory for benchmark CSV and summary files."),
        DeclareLaunchArgument(
            "performance_timeout", default_value="60.0",
            description="Benchmark duration measured in simulation seconds."),
        DeclareLaunchArgument("max_speed", default_value="1.0", description="Planner speed cap [m/s]."),
        DeclareLaunchArgument("max_acceleration", default_value="1.0",
                              description="Planner acceleration cap [m/s^2]."),
        DeclareLaunchArgument("robot_radius", default_value="0.15",
                              description="Robot radius used by the planner [m]."),
        DeclareLaunchArgument("safety_margin", default_value="0.15",
                              description="Additional planner clearance [m]."),
        DeclareLaunchArgument("trajectory_planner_enabled", default_value="true",
                              description="Use continuous receding-horizon holonomic trajectory planning."),
        DeclareLaunchArgument("trajectory_horizon", default_value="40",
                              description="Trajectory rollout length [steps]."),
        DeclareLaunchArgument("trajectory_heading_samples", default_value="41",
                              description="Number of sampled velocity headings per rollout."),
        DeclareLaunchArgument("trajectory_speed_samples", default_value="4",
                              description="Number of sampled speeds per rollout."),
        DeclareLaunchArgument("dynamic_obstacle_radius", default_value="0.20",
                              description="Radius assigned to each predicted moving-object track [m]."),
        DeclareLaunchArgument("moving_confirmation_age", default_value="3",
                              description="Consecutive moving observations required before dynamic prediction."),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ignition_resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        OpaqueFunction(function=launch_gazebo),
        robot_state_publisher,
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
    ])
