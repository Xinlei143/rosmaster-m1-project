"""Unit tests for the independent physical command watchdog state machine."""

import importlib.util
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist


NODE_PATH = Path(__file__).parents[1] / "imperative_navigation" / "cmd_vel_watchdog_node.py"
SPEC = importlib.util.spec_from_file_location("cmd_vel_watchdog", NODE_PATH)
WATCHDOG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WATCHDOG)


def command(x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0):
    value = Twist()
    value.linear.x, value.linear.y, value.linear.z = x, y, z
    value.angular.x, value.angular.y, value.angular.z = ax, ay, az
    return value


def assert_twist(actual, expected):
    assert actual.linear.x == expected.linear.x
    assert actual.linear.y == expected.linear.y
    assert actual.linear.z == expected.linear.z
    assert actual.angular.x == expected.angular.x
    assert actual.angular.y == expected.angular.y
    assert actual.angular.z == expected.angular.z


def test_no_raw_command_continuously_selects_zero():
    state = WATCHDOG.CommandWatchdogState(0.60)
    for now_ns in (0, 50_000_000, 600_000_000, 5_000_000_000):
        assert state.is_stale(now_ns)
        assert_twist(state.command_for_time(now_ns), command())


def test_fresh_nonzero_command_is_forwarded_without_remapping():
    state = WATCHDOG.CommandWatchdogState(0.60)
    raw = command(0.13, -0.07, 0.02, 0.01, -0.01, 0.21)
    state.receive(raw, 1_000_000_000)
    assert not state.is_stale(1_599_000_000)
    assert_twist(state.command_for_time(1_599_000_000), raw)


def test_timeout_forces_zero_and_later_raw_command_restores_forwarding():
    state = WATCHDOG.CommandWatchdogState(0.60)
    raw = command(0.12, 0.05, 0.0, 0.0, 0.0, -0.15)
    state.receive(raw, 1_000_000_000)
    assert_twist(state.command_for_time(1_600_000_001), command())
    restored = command(-0.08, 0.04, 0.0, 0.0, 0.0, 0.10)
    state.receive(restored, 1_700_000_000)
    assert not state.is_stale(1_700_000_000)
    assert_twist(state.command_for_time(1_700_000_000), restored)


def test_zero_raw_command_is_forwarded_as_zero_while_fresh():
    state = WATCHDOG.CommandWatchdogState(0.60)
    state.receive(command(), 2_000_000_000)
    assert not state.is_stale(2_300_000_000)
    assert_twist(state.command_for_time(2_300_000_000), command())


def test_watchdog_node_starts_without_a_controller_node():
    """No raw publisher is needed for the watchdog node and timer to exist."""
    rclpy.init()
    node = WATCHDOG.ImperativeCmdWatchdog()
    try:
        assert node.state.is_stale(0)
        # Explicitly run one timer cycle with no controller/raw publisher.
        node.publish_timer_callback()
    finally:
        node.destroy_node()
        rclpy.shutdown()
