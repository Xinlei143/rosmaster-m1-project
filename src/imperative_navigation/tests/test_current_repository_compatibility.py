"""Static compatibility checks for the restored parallel Imperative package."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SETUP = PACKAGE_ROOT / "setup.py"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"
GAZEBO_LAUNCH = PACKAGE_ROOT / "launch" / "imperative_m1_gazebo.launch.py"
REAL_LAUNCH = PACKAGE_ROOT / "launch" / "imperative_m1_real.launch.py"
WATCHDOG_LAUNCH = PACKAGE_ROOT / "launch" / "imperative_cmd_watchdog.launch.py"


def read(path):
    return path.read_text(encoding="utf-8")


def test_setup_registers_only_the_two_imperative_controllers():
    source = read(SETUP)
    assert "imperative_controller = imperative_navigation.controller_node:main" in source
    assert "imperative_m1_controller = imperative_navigation.m1_controller_node:main" in source
    assert "imperative_cmd_watchdog" not in source
    assert "dynamic_obstacle_mover" not in source
    assert "software_lidar" not in source
    assert "avoidance_performance_recorder" not in source
    assert '"share/" + package_name + "/algorithm"' in source
    assert '"share/" + package_name + "/launch"' in source


def test_package_declares_current_repository_adapters():
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {
        element.text for element in root.findall("depend") if element.text
    }
    assert {"m1_nav2_support", "m1_nav2_bringup", "yahboomcar_description"} <= dependencies
    assert {"geometry_msgs", "nav_msgs", "rclpy", "sensor_msgs", "tf2_ros",
            "visualization_msgs"} <= dependencies


def test_gazebo_launch_reuses_current_support_and_has_no_nav2_controller():
    source = read(GAZEBO_LAUNCH)
    assert 'get_package_share_directory("m1_nav2_support")' in source
    assert '"launch", "m1_gazebo.launch.py"' in source
    assert '"dynamic_obstacles_topic": "/m1/dynamic_obstacles"' in source
    assert '"static_obstacles": [0.0, 0.0, 0.3, -3.0, 2.2, 0.3]' in source
    assert "'/sim_scan'" in source and "'/scan'" in source
    assert '"trajectory_horizon", default_value="20"' in source
    assert "controller_server" not in source
    assert "velocity_smoother" not in source
    assert "nav2_controller" not in source


def test_gazebo_launch_keeps_current_scan_relay_only_for_software_lidar():
    source = read(GAZEBO_LAUNCH)
    relay_block = source[source.index("scan_relay = Node"):source.index("rviz = Node")]
    assert 'package="m1_nav2_bringup"' in relay_block
    assert 'executable="scan_relay"' in relay_block
    assert 'condition=IfCondition(LaunchConfiguration("software_lidar"))' in relay_block


def test_gazebo_launch_forwards_deferred_gpu_lidar_arguments():
    """The scoped support launch must retain LiDAR values until robot spawn."""
    source = read(GAZEBO_LAUNCH)
    for name, default in (
        ("render_engine", "ogre"),
        ("dual_gpu_lidar", "true"),
        ("gpu_lidar_min_angle", "-3.14159265359"),
        ("gpu_lidar_max_angle", "3.14159265359"),
    ):
        assert f'"{name}": LaunchConfiguration("{name}")' in source
        assert f'DeclareLaunchArgument("{name}", default_value="{default}"' in source


def test_real_launch_is_dry_run_by_default_and_publishes_raw_imperative_command():
    source = read(REAL_LAUNCH)
    assert 'DeclareLaunchArgument("enabled", default_value="false"' in source
    assert '"command_topic": "/imperative/cmd_vel_raw"' in source
    assert '"trajectory_horizon", default_value="20"' in source
    assert '"dynamic_obstacles_topic"' not in source


def test_watchdog_launch_reuses_current_watchdog_with_imperative_input():
    source = read(WATCHDOG_LAUNCH)
    assert 'package="m1_nav2_support"' in source
    assert 'executable="m1_cmd_watchdog"' in source
    assert 'DeclareLaunchArgument("input_topic", default_value="/imperative/cmd_vel_raw")' in source
    assert 'DeclareLaunchArgument("output_topic", default_value="/cmd_vel")' in source


def test_algorithm_loader_uses_installed_algorithm_data_directory():
    source = read(PACKAGE_ROOT / "imperative_navigation" / "algorithm_loader.py")
    assert "get_package_share_directory(\"imperative_navigation\")" in source
    assert '"algorithm" / "Imperative_learning_2D_moving.py"' in source


def test_current_support_files_are_not_duplicated_in_restored_package():
    names = {path.name for path in (PACKAGE_ROOT / "imperative_navigation").glob("*.py")}
    assert names.isdisjoint({
        "cmd_vel_watchdog_node.py", "dynamic_obstacle_mover.py",
        "software_lidar.py", "avoidance_performance_recorder.py",
    })
