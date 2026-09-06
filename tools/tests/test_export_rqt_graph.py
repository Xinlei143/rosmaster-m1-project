"""Contract tests for the reproducible rqt_graph exporter."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXPORTER = ROOT / "tools" / "export_rqt_graph.py"


def test_exporter_has_a_documented_node_topic_cli_contract():
    assert EXPORTER.is_file()
    source = EXPORTER.read_text()
    assert "--output" in source
    assert "NODE_TOPIC_GRAPH" in source
    assert "hide_dynamic_reconfigure=True" in source
    assert "quiet=True" in source
    assert "architecture_rqt_graph_exporter" in source


def test_exporter_uses_rqt_graphs_finite_refresh_cycle():
    """A zero graph-stale interval makes Graph.update() refresh forever."""
    source = EXPORTER.read_text()

    assert "graph.graph_stale = 0.0" not in source


def test_exporter_waits_for_dds_discovery_before_snapshotting():
    """A short-lived CLI node otherwise sees only a partial ROS graph."""
    source = EXPORTER.read_text()

    assert "--discovery-wait" in source
    assert "rclpy.spin_once" in source
    assert "args.discovery_wait" in source
