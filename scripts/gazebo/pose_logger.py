#!/usr/bin/env python3
"""
pose_logger.py

Log the robot's WORLD pose to a file at a fixed rate, for autonomy_check.sh.

Why not `gz model -m igvc_robot -p`, which is the obvious answer and is what
this replaced: each call is a gz-transport service round trip that starts a new
process, and under a loaded simulator it took roughly 20 SECONDS to return. A
60-sample run would have taken 20 minutes, and the samples would have been
spaced by however long the simulator happened to be busy - which is exactly
when the robot is doing something interesting. One subscriber costs nothing and
samples on a real clock.

WORLD POSE FROM ODOMETRY, AND WHY THAT IS LEGITIMATE HERE

gazebo_odom_shim publishes odometry in a frame whose axes are the world's and
whose origin is the spawn point, so

    world = odom + spawn_position

exactly, by construction. That is not an assumption: bringup_smoke_test.sh
measures odometry against `gz model -p` ground truth every run and got a
heading disagreement of 0.13 degrees and a distance ratio of 0.997 across
three runs on 2026-09-15.

It does mean this logger cannot detect the robot falling off the ground slab,
because odometry keeps counting while the wheels spin in mid-air. The caller
still takes one `gz model -p` reading at the end for that.

Usage:
    pose_logger.py OUT_FILE DURATION_S SPAWN_X SPAWN_Y [RATE_HZ] [wall|sim]

With `sim` (what autonomy_check.sh passes since 2026-09-24) DURATION_S is
SIMULATION time, measured from the odometry stamps, so a slow machine watches
the same stretch of driving as a fast one. A wall-clock ceiling of ten times
the duration, at least ten minutes, stops a stalled simulator hanging it. The
default stays `wall` so any older caller behaves as before.

Each line: world_x world_y sim_time yaw_rad
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class PoseLogger(Node):
    """Subscribe to /odom and keep only the most recent message."""

    def __init__(self):
        super().__init__('pose_logger')
        self.last = None
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)

    def _on_odom(self, msg):
        self.last = msg


def main():
    out, dur = sys.argv[1], float(sys.argv[2])
    sx, sy = float(sys.argv[3]), float(sys.argv[4])
    rate = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
    clock = sys.argv[6] if len(sys.argv) > 6 else 'wall'
    wall_cap = max(600.0, 10.0 * dur) if clock == 'sim' else dur

    rclpy.init()
    node = PoseLogger()

    # Deliberately a wall-clock loop rather than create_timer().
    #
    # A ROS timer on this node runs on the node clock, and with
    # use_sim_time the first version of this file produced ZERO samples: the
    # timer waits on /clock, and setting use_sim_time through set_parameters
    # after construction did not reliably attach the time source. The sampling
    # CADENCE does not need to be simulation time - only the timestamp written
    # into each row does, and that is taken from the message header.
    written = 0
    deadline = time.time() + wall_cap
    next_write = time.time()
    first_stamp = None
    with open(out, 'w') as fh:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now < next_write or node.last is None:
                continue
            next_write = now + 1.0 / rate
            p = node.last.pose.pose.position
            t = (node.last.header.stamp.sec
                 + node.last.header.stamp.nanosec * 1e-9)
            # Yaw is a fourth column, appended so older logs still parse.
            # It is needed because the robot's lateral extent depends on its
            # heading relative to the lane: a 0.81 x 0.97 m chassis reaches
            # 0.405 m sideways when aligned and up to 0.632 m when skewed, so
            # a fixed half-width understates how close it came to the paint.
            # gazebo_odom_shim has already rotated /odom into world axes, so
            # this yaw is a world heading and needs no spawn offset, unlike
            # the position above.
            q = node.last.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            fh.write('%.6f %.6f %.3f %.6f\n' % (p.x + sx, p.y + sy, t, yaw))
            fh.flush()
            written += 1
            if first_stamp is None:
                first_stamp = t
            if clock == 'sim' and t - first_stamp >= dur:
                break

    if clock == 'sim' and first_stamp is not None and time.time() >= deadline:
        print('pose_logger: WALL CEILING of %.0f s hit before %.0f s of sim time'
              % (wall_cap, dur))
    print('pose_logger: %d samples' % written)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
