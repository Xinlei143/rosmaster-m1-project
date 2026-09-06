"""Static contracts for the additive localized Imperative launch entries."""

from pathlib import Path


ROOT = Path(__file__).parents[3]
GAZEBO = ROOT / "src" / "imperative_navigation" / "launch" / "imperative_m1_localized_gazebo.launch.py"
REAL = ROOT / "src" / "imperative_navigation" / "launch" / "imperative_m1_localized_real.launch.py"
LOCALIZATION = ROOT / "src" / "m1_nav2_bringup" / "launch" / "m1_localization.launch.py"


def test_localization_only_launch_contains_only_localization_lifecycle_nodes():
    source = LOCALIZATION.read_text()
    assert 'executable="map_server"' in source
    assert 'executable="amcl"' in source
    assert 'node_names": ["map_server", "amcl"]' in source
    assert "controller_server" not in source
    assert "planner_server" not in source
    assert "bt_navigator" not in source
    assert "collision_monitor" not in source


def test_localized_gazebo_launch_wires_truth_lidar_and_map_goal():
    source = GAZEBO.read_text()
    assert '"slip_profile": LaunchConfiguration("slip_profile")' in source
    assert '"m1_gazebo.launch.py"' in source
    assert '"goal_frame": "map"' in source
    assert '"odom_topic": "/odom"' in source
    assert '"dynamic_obstacles_topic"' not in source


def test_localized_gazebo_launch_explicitly_allows_amcl_tf_lead_time():
    source = GAZEBO.read_text()
    assert '"global_tf_future_tolerance": LaunchConfiguration(' in source
    assert '"global_tf_future_tolerance", default_value="0.5"' in source


def test_localized_gazebo_launch_forwards_deferred_gpu_lidar_arguments():
    source = GAZEBO.read_text()
    for name, default in (
        ("render_engine", "ogre"),
        ("dual_gpu_lidar", "true"),
        ("gpu_lidar_min_angle", "-3.14159265359"),
        ("gpu_lidar_max_angle", "3.14159265359"),
    ):
        assert f'"{name}": LaunchConfiguration("{name}")' in source
        assert f'DeclareLaunchArgument("{name}", default_value="{default}"' in source


def test_localized_gazebo_launch_forwards_dynamic_scenario_controls():
    source = GAZEBO.read_text()
    for name, default in (("dynamic_seed", "20260814"), ("dynamic_motion_mode", "continuous")):
        assert f'"{name}": LaunchConfiguration("{name}")' in source
        assert f'DeclareLaunchArgument("{name}", default_value="{default}"' in source


def test_localized_real_launch_disables_fake_slip_and_initial_pose_by_default():
    source = REAL.read_text()
    assert '"goal_frame": "map"' in source
    assert '"set_initial_pose": "false"' in source
    assert '"slip_enabled"' not in source
    assert 'executable="m1_cmd_watchdog"' in source
