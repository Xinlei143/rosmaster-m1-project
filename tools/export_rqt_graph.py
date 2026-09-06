#!/usr/bin/env python3
"""Export the active ROS graph using rqt_graph's own DOT generator.

Run this only after sourcing the ROS workspace that owns the graph, for
example::

    python3 tools/export_rqt_graph.py --output docs/architecture/raw/nav2.dot

The result is the active Nodes/Topics rqt_graph view with standard debug and
parameter-event noise suppressed.  It deliberately does not filter project
nodes or project topics.
"""

import argparse
import time
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="Destination DOT file.")
    parser.add_argument("--node-filter", default="-/architecture_rqt_graph_exporter",
                        help="rqt_graph node filter; excludes the exporter by default.")
    parser.add_argument("--topic-filter", default="",
                        help="Optional rqt_graph topic filter.")
    parser.add_argument(
        "--discovery-wait", type=float, default=3.0,
        help="Seconds to spin the transient exporter node before snapshotting.")
    return parser.parse_args()


def main():
    args = parse_args()

    import rclpy
    from qt_dotgraph.pydotfactory import PydotFactory
    from rqt_graph.dotcode import NODE_TOPIC_GRAPH, RosGraphDotcodeGenerator
    from rqt_graph.rosgraph2_impl import Graph

    rclpy.init()
    node = rclpy.create_node("architecture_rqt_graph_exporter")
    try:
        deadline = time.monotonic() + max(args.discovery_wait, 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))

        graph = Graph(node)
        graph.set_node_stale(1.0)
        graph.update()

        dotcode = RosGraphDotcodeGenerator(node).generate_dotcode(
            rosgraphinst=graph,
            ns_filter=args.node_filter,
            topic_filter=args.topic_filter,
            graph_mode=NODE_TOPIC_GRAPH,
            dotcode_factory=PydotFactory(),
            hide_single_connection_topics=False,
            hide_dead_end_topics=False,
            cluster_namespaces_level=0,
            accumulate_actions=True,
            orientation="LR",
            quiet=True,
            unreachable=False,
            group_tf_nodes=True,
            hide_tf_nodes=False,
            group_image_nodes=False,
            hide_dynamic_reconfigure=True,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(dotcode)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
