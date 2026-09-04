"""Unit and source contracts for the ground-truth/slip odometry adapter."""

import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[3]
MODULE_PATH = ROOT / "src" / "m1_nav2_support" / "m1_nav2_support" / "odom_slip_simulator.py"
URDF = ROOT / "src" / "yahboomcar_description" / "urdf" / "yahboomcar_M1_gazebo.urdf.xacro"
BRIDGE = ROOT / "src" / "m1_nav2_support" / "launch" / "m1_gazebo.launch.py"
SOFTWARE_LIDAR = ROOT / "src" / "m1_nav2_support" / "m1_nav2_support" / "software_lidar.py"


def load_module():
    spec = importlib.util.spec_from_file_location("odom_slip_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_integrate_increment_applies_scale_and_bias_in_body_frame():
    module = load_module()
    state = module.SlipState(x=1.0, y=2.0, yaw=math.pi / 2.0)

    module.integrate_increment(
        state,
        dx_world=1.0,
        dy_world=0.0,
        dyaw=0.2,
        dt=2.0,
        x_scale=1.0,
        y_scale=1.5,
        yaw_scale=2.0,
        x_bias_per_second=0.1,
        y_bias_per_second=-0.2,
        yaw_bias_per_second=0.05,
        noise=(0.0, 0.0, 0.0),
    )

    # The world x increment is local -y at yaw=pi/2; the scaled local motion
    # therefore maps to +1.9 m along the simulated world x axis.
    assert state.x == pytest.approx(2.9)
    assert state.y == pytest.approx(2.2)
    assert state.yaw == pytest.approx(math.pi / 2.0 + 0.50)


def test_profile_parameters_are_deterministic_and_registered():
    module = load_module()
    first = module.profile_parameters("lateral")
    second = module.profile_parameters("lateral")
    assert first == second
    assert first["x_scale"] == pytest.approx(1.0)
    assert first["y_scale"] > 1.0


def test_burst_profile_keeps_its_built_in_burst_scale():
    module = load_module()
    node = object.__new__(module.OdomSlipSimulator)
    values = {
        "enabled": True,
        "profile": "burst",
        "x_scale": 1.0,
        "y_scale": 1.0,
        "yaw_scale": 1.0,
        "burst_enabled": False,
        "burst_start": 0.0,
        "burst_duration": 0.0,
        "burst_x_scale": 1.0,
        "burst_y_scale": 1.0,
        "burst_yaw_scale": 1.0,
    }
    node._parameter = values.__getitem__
    assert node._active_scales(2.5)[1] == pytest.approx(2.0)

    setup = (ROOT / "src" / "m1_nav2_support" / "setup.py").read_text()
    assert "odom_slip_simulator = m1_nav2_support.odom_slip_simulator:main" in setup


def test_ground_truth_topics_do_not_publish_normal_odom_tf():
    urdf = URDF.read_text()
    assert "<odom_topic>/ground_truth/odom</odom_topic>" in urdf
    assert "<tf_topic>/ground_truth/tf</tf_topic>" in urdf

    bridge = BRIDGE.read_text()
    assert "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" in bridge
    assert '"/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"' not in bridge
    assert '"/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"' not in bridge


def test_software_lidar_pose_topic_is_configurable():
    source = SOFTWARE_LIDAR.read_text()
    assert 'declare_parameter("pose_topic", "/odom")' in source
    assert 'self.get_parameter("pose_topic").value' in source


def test_gazebo_support_starts_slip_adapter_and_uses_truth_for_software_lidar():
    source = BRIDGE.read_text()
    assert 'executable="odom_slip_simulator"' in source
    assert '"pose_topic": "/ground_truth/odom"' in source
    assert source.count('LaunchConfiguration("software_lidar")') >= 3
    assert '"== \'false\' and \'"' not in source
