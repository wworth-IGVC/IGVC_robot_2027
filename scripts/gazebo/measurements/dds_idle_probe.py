#!/usr/bin/env python3
"""A ROS 2 node that subscribes to nothing, publishes nothing, and spins.

The test for P0-1 (docs/GAZEBO_TODO.md). A Humble container on the Jazzy
simulator's graph printed "sequence size exceeds remaining buffer" three times
per process, and it did so with a node that subscribed to nothing, so the
message can only come from DDS discovery, not from any topic's data. This is
that node, kept so the test can be repeated exactly.

Run it in the container under test while a simulator is up on the same
ROS_DOMAIN_ID, capture everything it prints, and count the lines:

    python3 dds_idle_probe.py 60 > probe.log 2>&1
    grep -c "sequence size exceeds remaining buffer" probe.log

Fast DDS prints that line from its own C++ logging, not through rclpy, so
capture stderr and stdout both. It prints the count of other nodes it
discovered, so a run that saw no simulator at all cannot pass as a clean one:
a probe that discovered nobody proves nothing about the defect, and it
exits 3 in that case.

MEASURED 2026-09-29, 60 s each, the simulator in igvc_gazebo, all containers
on one compose network and ROS_DOMAIN_ID 0:

    container            simulator  DDS profile  defect lines  nodes seen
    Humble (retired)     none       yes          0             0   exit 3
    Humble (retired)     running    yes          3             0
    Humble (retired)     running    no           3             0
    igvc_dev_jazzy       running    no           0             4
    igvc_dev_jazzy       running    yes          0             4

So P0-1 is gone once perception runs on Jazzy, and the test still fails where
it should. The Humble probe discovering no nodes at all is the clue to the
cause: rmw_dds_common's Gid is char[24] on Humble and char[16] on Jazzy
(RMW_GID_STORAGE_SIZE 24u against 16u, read from both images), and node names
travel in /ros_discovery_info messages built from Gids. A Humble reader
misparses Jazzy's, printing this line and learning no node names, while topic
data still flows because DDS endpoint discovery does not use that topic. The
Gid sizes and the zero-node result are measured; the causal link is inferred.
"""
import sys
import time

import rclpy
from rclpy.node import Node


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = Node('dds_idle_probe')
    start = time.monotonic()
    seen = set()
    while time.monotonic() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.5)
        for name, ns in node.get_node_names_and_namespaces():
            if name != 'dds_idle_probe':
                seen.add(ns.rstrip('/') + '/' + name)
    print('IDLE_PROBE_DONE seconds=%.1f other_nodes_seen=%d' % (duration, len(seen)))
    sys.stdout.flush()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if seen else 3


if __name__ == '__main__':
    sys.exit(main())
