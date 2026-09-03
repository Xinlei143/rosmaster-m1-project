"""Static checks for the simulation-only Mecanum contact profile."""

import ast
from pathlib import Path


ROOT = Path(__file__).parents[3]
URDF = ROOT / "src" / "yahboomcar_description" / "urdf" / "yahboomcar_M1_gazebo.urdf.xacro"
WORLD = ROOT / "src" / "m1_nav2_support" / "worlds" / "m1.sdf"
BRIDGE_LAUNCH = ROOT / "src" / "m1_nav2_support" / "launch" / "m1_gazebo.launch.py"


def test_all_mecanum_wheels_release_the_roller_direction():
    text = URDF.read_text()
    assert text.count("<mu2>0.0</mu2>") == 4
    assert "<mu2>0.1</mu2>" not in text


def test_x_pattern_friction_directions_are_preserved():
    text = URDF.read_text()
    assert text.count("<fdir1 ignition:expressed_in=\"base_footprint\">1 -1 0</fdir1>") == 2
    assert text.count("<fdir1 ignition:expressed_in=\"base_footprint\">1 1 0</fdir1>") == 2


def test_ground_has_explicit_high_friction():
    text = WORLD.read_text()
    assert "<mu>50.0</mu>" in text


def test_mecanum_drive_uses_the_requested_linear_acceleration_limits():
    text = URDF.read_text()
    assert "<min_acceleration>-0.8</min_acceleration>" in text
    assert "<max_acceleration>0.8</max_acceleration>" in text


def _parameter_bridge_arguments():
    module = ast.parse(BRIDGE_LAUNCH.read_text())
    bridges = {}
    for statement in ast.walk(module):
        if not isinstance(statement, ast.Assign) or not isinstance(statement.value, ast.Call):
            continue
        if not isinstance(statement.value.func, ast.Name) or statement.value.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in statement.value.keywords}
        if ast.literal_eval(keywords["package"]) != "ros_gz_bridge":
            continue
        if ast.literal_eval(keywords["executable"]) != "parameter_bridge":
            continue
        name = ast.literal_eval(keywords["name"])
        bridges[name] = ast.literal_eval(keywords["arguments"])
    return bridges


def test_gazebo_scan_bridge_isolated_from_core_bridge_topics():
    bridges = _parameter_bridge_arguments()

    assert set(bridges) == {
        "m1_gazebo_bridge_core",
        "m1_gazebo_bridge_scan",
    }
    core_arguments = bridges["m1_gazebo_bridge_core"]
    scan_arguments = bridges["m1_gazebo_bridge_scan"]

    for route in (
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/world/m1/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ):
        assert route in core_arguments
    assert not any(argument.startswith("/scan@") for argument in core_arguments)
    assert scan_arguments == [
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]


def test_gazebo_bridges_namespaced_world_clock_to_ros_clock():
    source = BRIDGE_LAUNCH.read_text()
    bridges = _parameter_bridge_arguments()

    assert "/world/m1/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" in bridges[
        "m1_gazebo_bridge_core"
    ]
    assert '("/world/m1/clock", "/clock")' in source


def test_gazebo_render_engine_is_explicit_and_defaults_to_ogre2():
    source = BRIDGE_LAUNCH.read_text()

    assert 'LaunchConfiguration("render_engine")' in source
    assert '"render_engine", default_value="ogre2"' in source


def test_gpu_lidar_horizontal_fov_is_configurable_with_a_360_degree_default():
    """The cubemap experiment must change only the real GPU LiDAR FOV."""
    text = URDF.read_text()
    source = BRIDGE_LAUNCH.read_text()

    assert '<xacro:arg name="gpu_lidar_min_angle" default="-3.14159265359"/>' in text
    assert '<xacro:arg name="gpu_lidar_max_angle" default="3.14159265359"/>' in text
    assert '<min_angle>$(arg gpu_lidar_min_angle)</min_angle>' in text
    assert '<max_angle>$(arg gpu_lidar_max_angle)</max_angle>' in text
    assert '" gpu_lidar_min_angle:=", LaunchConfiguration("gpu_lidar_min_angle")' in source
    assert '" gpu_lidar_max_angle:=", LaunchConfiguration("gpu_lidar_max_angle")' in source
    assert '"gpu_lidar_min_angle", default_value="-3.14159265359"' in source
    assert '"gpu_lidar_max_angle", default_value="3.14159265359"' in source
