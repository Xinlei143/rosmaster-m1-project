"""Start only the M1 static map and AMCL localization lifecycle."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    params_file = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "m1_baseline.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    params_path = LaunchConfiguration("params_file")
    localization_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map"),
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                "initial_pose.x": LaunchConfiguration("initial_pose_x"),
                "initial_pose.y": LaunchConfiguration("initial_pose_y"),
                "initial_pose.yaw": LaunchConfiguration("initial_pose_yaw"),
            },
            convert_types=True,
        ),
        allow_substs=True,
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
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            "node_names": ["map_server", "amcl"],
        }],
    )

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("set_initial_pose", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="-2.5"),
        DeclareLaunchArgument("initial_pose_y", default_value="-1.5"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
    ]
    return LaunchDescription(arguments + [map_server, amcl, lifecycle_manager])

