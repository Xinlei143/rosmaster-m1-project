"""Launch imperative navigation against a running physical Rosmaster M1."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("goal_x", default_value="1.0", description="Goal X in /odom [m]."),
        DeclareLaunchArgument("goal_y", default_value="0.0", description="Goal Y in /odom [m]."),
        DeclareLaunchArgument("enabled", default_value="false",
                              description="Enable physical motion only after the safety check."),
        DeclareLaunchArgument("max_speed", default_value="0.18", description="M1 speed cap [m/s]."),
        DeclareLaunchArgument("max_acceleration", default_value="0.25", description="M1 acceleration cap [m/s^2]."),
        DeclareLaunchArgument("robot_radius", default_value="0.18",
                              description="Physical robot radius used by the planner [m]."),
        DeclareLaunchArgument("safety_margin", default_value="0.18",
                              description="Additional planner clearance around the robot [m]."),
        DeclareLaunchArgument("tf_max_age", default_value="0.30",
                              description="Maximum age of the latest odom<-laser TF [s]."),
        DeclareLaunchArgument("torch_num_threads", default_value="0",
                              description="PyTorch intra-op CPU threads; 0 keeps the platform default."),
        DeclareLaunchArgument("emergency_stop_distance", default_value="0.45",
                              description="Stop at or below this laser range [m]."),
    ]
    controller = Node(
        package="imperative_navigation",
        executable="imperative_m1_controller",
        name="imperative_m1_controller",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "enabled": LaunchConfiguration("enabled"),
            "max_speed": LaunchConfiguration("max_speed"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
            "tf_max_age": LaunchConfiguration("tf_max_age"),
            "torch_num_threads": LaunchConfiguration("torch_num_threads"),
            "emergency_stop_distance": LaunchConfiguration("emergency_stop_distance"),
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            # The watchdog is deliberately launched separately. This planner
            # launch only publishes raw commands, so Ctrl+C cannot terminate
            # the independent process that continually owns /cmd_vel.
            "command_topic": "/imperative/cmd_vel_raw",
            "odom_frame": "odom",
        }],
    )
    return LaunchDescription(arguments + [controller])
