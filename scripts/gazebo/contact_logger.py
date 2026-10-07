#!/usr/bin/env python3
"""
contact_logger.py

Log every barrel contact the simulator reports, for autonomy_check.sh.

Each barrel in igvc_course.sdf carries a contact sensor on its own collision,
all publishing to /sim/barrel_contacts, bridged to ROS as
ros_gz_interfaces/msg/Contacts (gazebo_bridge.yaml). From the barrel side,
any part of the robot touching it is reported: the chassis, a camera body,
the lidar, a wheel. The IGVC rules penalise any touch ("Sideswipe/Obstacle
Touch") and end the run on a crash or a displaced barrel, so a touch is what
this records; the barrels are static (RQ-11, decided 2026-10-01), so what a
barrel would do next is never needed.

WHY THE PUBLISHER COUNT IS RECORDED

No contact is the normal result, and a broken chain (no bridge entry, a world
generated before the sensors existed) produces exactly the same silence. So
this also records the most publishers it ever saw on the topic. Zero means the
bridge never offered the topic and the run measured nothing about barrels;
the caller must report that as NOT MEASURED, never as "no contact".
measurements/barrel_contact_check.sh is the test that the chain can report a
touch at all: it drives the robot into a barrel and requires a contact.

Usage:
    contact_logger.py OUT_FILE

Runs until SIGINT or SIGTERM. Each line, one per barrel per message:
    sim_time barrel_name robot_collision_name max_depth_m
The last line is a comment: "# publishers_seen N".
"""

import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Contacts

TOPIC = '/sim/barrel_contacts'


def _model(scoped):
    """'barrel_3::link::collision' -> 'barrel_3'."""
    return scoped.split('::', 1)[0]


class ContactLogger(Node):
    """Write one line per barrel per Contacts message."""

    def __init__(self, fh):
        super().__init__('contact_logger')
        self.fh = fh
        self.messages = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(Contacts, TOPIC, self._on_contacts, qos)

    def _on_contacts(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        seen = {}
        for c in msg.contacts:
            a, b = c.collision1.name, c.collision2.name
            if _model(a).startswith('barrel_'):
                barrel, other = _model(a), b
            elif _model(b).startswith('barrel_'):
                barrel, other = _model(b), a
            else:
                continue
            depth = max(c.depths) if len(c.depths) else 0.0
            key = (barrel, other)
            seen[key] = max(seen.get(key, 0.0), depth)
        for (barrel, other), depth in sorted(seen.items()):
            self.fh.write('%.3f %s %s %.4f\n' % (t, barrel, other, depth))
        if seen:
            self.fh.flush()
            self.messages += 1


def main():
    out = sys.argv[1]
    stop = {'now': False}

    def _stop(*_):
        stop['now'] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # rclpy's own handler would turn the first SIGINT into an exception
    # inside spin_once; ours lets the loop finish and write the last line.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with open(out, 'w') as fh:
        node = ContactLogger(fh)
        publishers_seen = 0
        next_count = 0.0
        while not stop['now'] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now >= next_count:
                next_count = now + 1.0
                publishers_seen = max(publishers_seen,
                                      node.count_publishers(TOPIC))
        fh.write('# publishers_seen %d\n' % publishers_seen)
    print('contact_logger: %d contact messages, publishers seen %d'
          % (node.messages, publishers_seen))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
