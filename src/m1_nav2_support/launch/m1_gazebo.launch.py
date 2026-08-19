"""Launch M1 Gazebo Sim resources used by the Nav2 MPPI bringup."""

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
    package_share = get_package_share_directory("m1_nav2_support")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world = os.path.join(package_share, "worlds", "m1.sdf")
    server_only = "-s " if LaunchConfiguration("gui").perform(context).lower() == "false" else ""

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        # OGRE2 is required for the GPU LiDAR's full 360-degree field of view.
        launch_arguments={"gz_args": f"{server_only}-r -v 4 --render-engine ogre2 {world}"}.items(),
    )]


def generate_launch_description():
    package_share = get_package_share_directory("m1_nav2_support")
    description_share = get_package_share_directory("yahboomcar_description")
    description_resource_root = os.path.dirname(description_share)
    model_file = os.path.join(
        description_share, "urdf", "yahboomcar_M1_gazebo.urdf.xacro")
    # Feed Gazebo the expanded XML directly. This avoids the unreliable
    # transient-local /robot_description topic handshake in ros_gz_sim 0.244.
    robot_description_xml = Command([
        FindExecutable(name="xacro"), " ", model_file,
        " enable_gpu_lidar:=",
        PythonExpression([
            "'false' if '", LaunchConfiguration("software_lidar"),
            "' == 'true' else 'true'"]),
    ])
    robot_description = ParameterValue(robot_description_xml, value_type=str)

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
            "-string", robot_description_xml,
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
            "/world/m1/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/m1/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        remappings=[
            ("/world/m1/dynamic_pose/info", "/m1/gazebo_dynamic_tf"),
        ],
        output="screen",
    )

    dynamic_obstacle_mover = Node(
        package="m1_nav2_support",
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
        package="m1_nav2_support",
        executable="software_lidar",
        name="software_lidar",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
        parameters=[{"use_sim_time": True}],
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
        DeclareLaunchArgument("rviz", default_value="false", description="Unused adapter compatibility argument."),
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
                "random_waypoint for waypoint patrol.")),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ignition_resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        OpaqueFunction(function=launch_gazebo),
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        dynamic_obstacle_mover,
        software_lidar,
    ])
