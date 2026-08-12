"""Launch the M1 Gazebo Sim scene, ROS bridges, planner, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        parameters=[{"use_sim_time": True}],
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

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo GUI."),
        DeclareLaunchArgument("software_lidar", default_value="true",
                              description="Use WSLg-safe software LaserScan instead of the broken GPU lidar."),
        DeclareLaunchArgument("max_speed", default_value="1.0", description="Planner speed cap [m/s]."),
        DeclareLaunchArgument("max_acceleration", default_value="1.0",
                              description="Planner acceleration cap [m/s^2]."),
        DeclareLaunchArgument("robot_radius", default_value="0.15",
                              description="Robot radius used by the planner [m]."),
        DeclareLaunchArgument("safety_margin", default_value="0.15",
                              description="Additional planner clearance [m]."),
        OpaqueFunction(function=launch_gazebo),
        bridge,
        dynamic_obstacle_mover,
        software_lidar,
        controller,
        rviz,
    ])
