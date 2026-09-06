"""Tests for the localized controller's global-goal conversion contract."""

from types import SimpleNamespace
from pathlib import Path

import pytest
from imperative_navigation.localization_geometry import (
    inverse_transform_point_2d,
    resolve_goal_odom,
    transform_point_2d,
)


def transform(x, y, yaw, stamp_ns=10_000_000_000):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(
                x=0.0, y=0.0, z=__import__("math").sin(yaw / 2.0),
                w=__import__("math").cos(yaw / 2.0)),
        ),
    )


def test_map_goal_is_transformed_into_odom_using_translation_and_yaw():
    result = transform_point_2d((2.0, 1.0), transform(-0.3, 0.0, 0.0))
    assert result == pytest.approx((1.7, 1.0))


def test_repeated_transform_lookup_changes_the_odom_goal():
    goal = (2.0, 1.0)
    first = resolve_goal_odom(goal, transform(-0.3, 0.0, 0.0),
                              now_ns=10_100_000_000, max_age=0.5)
    second = resolve_goal_odom(goal, transform(-0.1, 0.2, 0.0),
                               now_ns=10_100_000_000, max_age=0.5)
    assert first == pytest.approx((1.7, 1.0))
    assert second == pytest.approx((1.9, 1.2))


def test_goal_conversion_rejects_missing_or_stale_localization_transform():
    goal = (2.0, 1.0)
    assert resolve_goal_odom(goal, None, now_ns=11_000_000_000, max_age=0.5) is None
    stale = transform(-0.3, 0.0, 0.0, stamp_ns=1_000_000_000)
    assert resolve_goal_odom(goal, stale, now_ns=2_000_000_000, max_age=0.5) is None


def test_goal_conversion_accepts_amcl_transform_tolerance_lead_time():
    goal = (2.0, 1.0)
    result = resolve_goal_odom(
        goal,
        transform(-0.3, 0.0, 0.0, stamp_ns=10_500_000_000),
        now_ns=10_000_000_000,
        max_age=0.5,
        future_tolerance=0.5,
    )
    assert result == pytest.approx((1.7, 1.0))


def test_goal_conversion_rejects_tf_beyond_configured_future_tolerance():
    goal = (2.0, 1.0)
    assert resolve_goal_odom(
        goal,
        transform(-0.3, 0.0, 0.0, stamp_ns=10_500_000_001),
        now_ns=10_000_000_000,
        max_age=0.5,
        future_tolerance=0.5,
    ) is None


def test_position_can_be_projected_back_to_map_without_mutating_tracks():
    tf = transform(-0.3, 0.0, 0.0)
    position = (1.7, 1.0)
    tracks = [[0.5, 0.2]]
    result = inverse_transform_point_2d(position, tf)
    assert result == pytest.approx((2.0, 1.0))
    assert tracks == [[0.5, 0.2]]


def test_controller_source_declares_localized_goal_contract():
    source = (Path(__file__).resolve().parents[1] /
              "imperative_navigation" / "m1_controller_node.py").read_text()
    assert '"goal_frame", "odom"' in source
    assert '"global_frame", "map"' in source
    assert '"global_tf_max_age", 0.5' in source
    assert '"global_tf_future_tolerance", 0.0' in source
    assert "resolve_goal_odom" in source
    assert "publishing stop" in source


def test_controller_handles_shutdown_publish_race():
    source = (Path(__file__).resolve().parents[1] /
              "imperative_navigation" / "m1_controller_node.py").read_text()
    assert "from rclpy._rclpy_pybind11 import RCLError" in source
    callback = source[source.index("    def control_callback"):source.index("    def publish_body_velocity")]
    assert callback.index("except RCLError:") < callback.index("except Exception as error:")
    publish_stop = source[source.index("    def publish_stop(self):"):source.index("    def publish_stop_burst")]
    assert "return self.publish_message(self.command_publisher, Twist())" in publish_stop


def test_controller_routes_all_ros_output_through_shutdown_safe_publish_boundary():
    source = (Path(__file__).resolve().parents[1] /
              "imperative_navigation" / "m1_controller_node.py").read_text()
    assert "def publish_message(self, publisher, message):" in source
    for publisher in ("command_publisher", "path_publisher", "track_publisher", "status_publisher"):
        assert f"self.{publisher}.publish(" not in source
    publish_message = source[source.index("    def publish_message"):source.index("    def publish_stop")]
    assert "if rclpy.ok():\n                raise" in publish_message


def test_controller_shutdown_handles_sigint_while_executor_waits_for_callbacks():
    source = (Path(__file__).resolve().parents[1] /
              "imperative_navigation" / "m1_controller_node.py").read_text()
    teardown = source[source.index("    finally:\n        # The separate watchdog") :]
    assert "try:\n            executor.shutdown()\n        except KeyboardInterrupt:" in teardown
    assert "executor_shutdown_interrupted = True" in teardown
    assert "if not executor_shutdown_interrupted:\n            node.destroy_node()" in teardown
