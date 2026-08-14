"""Launch the M1 Gazebo Sim scene, ROS bridges, planner, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_gazebo(context):
    package_share = get_package_share_directory("imperative_navigation")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_name = LaunchConfiguration("world").perform(context)
    world = os.path.join(package_share, "worlds", world_name)
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
            "/world/imperative_m1/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen",
    )

    controller = Node(
        package="imperative_navigation",
        executable="imperative_controller",
        name="imperative_controller",
        condition=IfCondition(LaunchConfiguration("planner")),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "command_topic": "/cmd_vel",
            "scan_topic": PythonExpression(["'/sim_scan' if '", LaunchConfiguration("software_lidar"), "' == 'true' else '/scan'"]),
            "odom_topic": "/odom",
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
            # Matches the static cylinders in imperative_m1.sdf.  This is used
            # only when the WSLg/OGRE GPU-LiDAR bug saturates every beam.
            "static_obstacles": [-2.05, -1.18, 0.3, -3.0, 2.2, 0.3],
            "require_dynamic_obstacles": True,
        }],
    )

    dynamic_obstacle_mover = Node(
        package="imperative_navigation",
        executable="dynamic_obstacle_mover",
        name="dynamic_obstacle_mover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "move_obstacles": LaunchConfiguration("move_obstacles"),
            "random_seed": LaunchConfiguration("random_seed"),
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

    experiment_logger = Node(
        package="imperative_navigation",
        executable="experiment_logger",
        name="experiment_logger",
        condition=IfCondition(LaunchConfiguration("record")),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "output_dir": LaunchConfiguration("log_dir"),
            "run_name": LaunchConfiguration("run_name"),
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "scan_topic": PythonExpression(["'/sim_scan' if '", LaunchConfiguration("software_lidar"), "' == 'true' else '/scan'"]),
            "command_topic": "/cmd_vel",
            "odom_topic": "/odom",
        }],
    )

    # Gazebo resolves package:// mesh URIs through its model resource path.
    ignition_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]))
    gz_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo GUI."),
        DeclareLaunchArgument("planner", default_value="true", description="Start the imperative planner."),
        DeclareLaunchArgument("world", default_value="imperative_m1.sdf",
                              description="SDF world filename under the package worlds directory."),
        DeclareLaunchArgument("software_lidar", default_value="true",
                              description="Use WSLg-safe software LaserScan instead of the broken GPU lidar."),
        DeclareLaunchArgument("record", default_value="false", description="Record experiment CSV files."),
        DeclareLaunchArgument("log_dir", default_value="/tmp/imperative_m1_experiment",
                              description="Experiment output directory."),
        DeclareLaunchArgument("run_name", default_value="gazebo", description="Experiment run label."),
        DeclareLaunchArgument("goal_x", default_value="2.5", description="Goal X in /odom [m]."),
        DeclareLaunchArgument("goal_y", default_value="1.5", description="Goal Y in /odom [m]."),
        DeclareLaunchArgument("move_obstacles", default_value="true",
                              description="Move the three test obstacles."),
        DeclareLaunchArgument("random_seed", default_value="-1",
                              description="Obstacle random seed; -1 uses a time-based seed."),
        DeclareLaunchArgument("max_speed", default_value="1.0", description="Planner speed cap [m/s]."),
        DeclareLaunchArgument("max_acceleration", default_value="1.0",
                              description="Planner acceleration cap [m/s^2]."),
        DeclareLaunchArgument("robot_radius", default_value="0.15",
                              description="Robot radius used by the planner [m]."),
        DeclareLaunchArgument("safety_margin", default_value="0.15",
                              description="Additional planner clearance [m]."),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ignition_resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        OpaqueFunction(function=launch_gazebo),
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        dynamic_obstacle_mover,
        software_lidar,
        controller,
        experiment_logger,
        rviz,
    ])
