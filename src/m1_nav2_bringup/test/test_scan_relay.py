"""Unit tests for the diagnostic-only software scan dropout gate."""

from m1_nav2_bringup.scan_relay import dropout_active


def test_dropout_gate_is_disabled_by_default():
    assert dropout_active(10.0, 12.0, -1.0, 0.0) is False
    assert dropout_active(10.0, 12.0, 10.0, 0.0) is False


def test_dropout_gate_only_suppresses_the_requested_interval():
    assert dropout_active(10.0, 19.9, 10.0, 10.0) is False
    assert dropout_active(10.0, 20.0, 10.0, 10.0) is True
    assert dropout_active(10.0, 29.999, 10.0, 10.0) is True
    assert dropout_active(10.0, 30.0, 10.0, 10.0) is False


def test_dropout_gate_handles_clock_before_start():
    assert dropout_active(10.0, 9.0, 2.0, 3.0) is False
