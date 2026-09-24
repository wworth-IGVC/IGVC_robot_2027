#!/usr/bin/env python3
"""PART 2 of bringup_smoke_test.sh: drive a short burst, measure the coast,
stop the robot, all in ONE process and all on SIMULATION time.

Why this exists
---------------
The smoke test used to publish the burst with `ros2 topic pub`, then take four
pose samples with `ros2 topic echo --once` and `gz model -p` before publishing
a stop. Each of those CLI calls spends seconds just starting up, and Gazebo's
DiffDrive holds the last command the whole time. So the robot kept driving for
as long as the SAMPLING took, which depends on how fast the machine starts a
Python process. Measured 2026-09-24 on the reference laptop: the "4 s" coast
window actually spanned about 11.7 s, the robot covered 9.5 m against a slab
edge 9.44 m away, and it drove off. On a slower machine, notably a Mac running
the image under emulation, it is worse, not better.

Here the timeline is fixed in simulation time, so the excursion is bounded by
arithmetic rather than by the machine:

    burst   BURST_S  at SPEED m/s      -> at most SPEED * BURST_S
    coast   COAST_S  publishing nothing -> at most SPEED * COAST_S more
    stop    zero twist, then SETTLE_S of rest

With the defaults that is at most 0.4 * (3 + 4) = 2.8 m, against 9.44 m to the
slab edge.

It also answers the /cmd_vel timeout question properly, which three earlier
attempts could not: the coast is measured between two odometry messages whose
stamps are recorded, starting at the last command actually sent, rather than
between two CLI samples an unknown number of seconds apart.

Output: human-readable lines, then KEY=VALUE lines for the shell to parse.
Exit 0 if the robot moved, 1 if it did not, 2 if the harness itself failed
(no /clock, no /odom, or nothing subscribed to /cmd_vel).
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED = float(os.environ.get("PROBE_SPEED", "0.4"))
BURST_S = float(os.environ.get("PROBE_BURST_S", "3.0"))
COAST_S = float(os.environ.get("PROBE_COAST_S", "4.0"))
SETTLE_S = float(os.environ.get("PROBE_SETTLE_S", "2.0"))
CMD_TOPIC = os.environ.get("PROBE_CMD_TOPIC", "/cmd_vel")
ODOM_TOPIC = os.environ.get("PROBE_ODOM_TOPIC", "/odom")
# Wall-clock ceiling on the whole probe. Generous, because an emulated machine
# may run simulation time at a fraction of real time.
WALL_LIMIT_S = float(os.environ.get("PROBE_WALL_LIMIT_S", "240"))
PUB_HZ = 10.0


class Probe(Node):
    def __init__(self):
        super().__init__(
            "smoke_drive_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.odom = None          # latest Odometry
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 50)
        self.wall0 = time.time()

    def _on_odom(self, m):
        self.odom = m

    # -- helpers --------------------------------------------------------------
    def wall_ok(self):
        return time.time() - self.wall0 < WALL_LIMIT_S

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for_sim(self, seconds, twist=None):
        """Advance `seconds` of SIMULATION time, optionally publishing `twist`
        at PUB_HZ of simulation time. Returns False on wall-clock timeout."""
        start = self.sim_now()
        next_pub = start
        while self.sim_now() - start < seconds:
            if not self.wall_ok():
                return False
            if twist is not None and self.sim_now() >= next_pub:
                self.pub.publish(twist)
                next_pub += 1.0 / PUB_HZ
            rclpy.spin_once(self, timeout_sec=0.01)
        return True


def xy(m):
    p = m.pose.pose.position
    return p.x, p.y


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    rclpy.init()
    n = Probe()

    # 1. /clock must be running, or "simulation time" means nothing.
    while n.sim_now() == 0.0 and n.wall_ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - n.wall0 > 30:
            break
    if n.sim_now() == 0.0:
        # Seen once on 2026-09-24 on the tier A laptop: /clock advanced 2 s,
        # then stopped, with every camera topic silent. gz-sim's sensors
        # system holds the physics step until a render completes, so a render
        # thread that never finishes freezes simulation time exactly like
        # this. Name it, so it is not read as a drive or odometry problem.
        print("  PROBE: no /clock for 30 s of wall time. The SIMULATION CLOCK IS NOT")
        print("         ADVANCING: gz sim has stalled or died. That is a simulator")
        print("         failure, not a drive failure. If the camera topics above also")
        print("         FAILED, suspect a render hang; restart the simulator and rerun.")
        print("PROBE_RESULT=HARNESS_FAIL")
        return 2

    # 2. Something must be listening on the command topic, or the burst is
    #    shouted into the void and the test would blame the robot.
    t0 = time.time()
    while n.pub.get_subscription_count() == 0 and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    subs = n.pub.get_subscription_count()
    if subs == 0:
        print("  PROBE: nothing subscribes to %s after 20 s; the simulator's"
              " DiffDrive is not listening" % CMD_TOPIC)
        print("PROBE_RESULT=HARNESS_FAIL")
        return 2

    # 3. A fresh odometry message before we start.
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None:
        print("  PROBE: no %s after 20 s" % ODOM_TOPIC)
        print("PROBE_RESULT=HARNESS_FAIL")
        return 2
    start_xy = xy(n.odom)

    go = Twist()
    go.linear.x = SPEED
    stop = Twist()

    # 4. Burst.
    burst_sim0 = n.sim_now()
    wall_burst0 = time.time()
    if not n.spin_for_sim(BURST_S, go):
        n.pub.publish(stop)
        print("  PROBE: wall-clock limit hit during the burst")
        print("PROBE_RESULT=HARNESS_FAIL")
        return 2
    last_cmd_sim = n.sim_now()
    rclpy.spin_once(n, timeout_sec=0.05)
    burst_end = n.odom
    burst_end_xy = xy(burst_end)

    # 5. Coast: publish NOTHING for COAST_S of simulation time. This is the
    #    /cmd_vel timeout measurement, and it is the only phase where the
    #    robot may move without a command.
    if not n.spin_for_sim(COAST_S, None):
        n.pub.publish(stop)
        print("  PROBE: wall-clock limit hit during the coast")
        print("PROBE_RESULT=HARNESS_FAIL")
        return 2
    rclpy.spin_once(n, timeout_sec=0.05)
    coast_end = n.odom
    coast_end_xy = xy(coast_end)
    coast_twist = coast_end.twist.twist.linear.x

    # 6. Stop deliberately, then let it settle so the caller samples at rest.
    n.spin_for_sim(1.0, stop)
    n.spin_for_sim(SETTLE_S, None)
    rclpy.spin_once(n, timeout_sec=0.05)
    rest_xy = xy(n.odom)
    wall_used = time.time() - wall_burst0
    sim_used = n.sim_now() - burst_sim0

    burst_d = math.hypot(burst_end_xy[0] - start_xy[0], burst_end_xy[1] - start_xy[1])
    coast_d = math.hypot(coast_end_xy[0] - burst_end_xy[0], coast_end_xy[1] - burst_end_xy[1])
    total_d = math.hypot(rest_xy[0] - start_xy[0], rest_xy[1] - start_xy[1])
    coast_window = stamp(coast_end) - stamp(burst_end)
    rtf = sim_used / wall_used if wall_used > 0 else 0.0
    bound = SPEED * (BURST_S + COAST_S) + SPEED * 1.0

    print("  burst  : %.2f m/s for %.1f s of sim time -> %.3f m" % (SPEED, BURST_S, burst_d))
    print("  coast  : %.3f m in %.2f s of sim time after the last command"
          " (odometry stamps), speed at the end %.3f m/s" % (coast_d, coast_window, coast_twist))
    print("  total  : %.3f m from start to rest, bounded by design at %.1f m" % (total_d, bound))
    print("  timing : %.1f s sim in %.1f s wall, real time factor %.2f" % (sim_used, wall_used, rtf))

    held = coast_d > 0.25 * SPEED * COAST_S
    print("PROBE_BURST_M=%.3f" % burst_d)
    print("PROBE_COAST_M=%.3f" % coast_d)
    print("PROBE_COAST_WINDOW_S=%.2f" % coast_window)
    print("PROBE_COAST_END_SPEED=%.3f" % coast_twist)
    print("PROBE_COAST_HELD=%s" % ("yes" if held else "no"))
    print("PROBE_TOTAL_M=%.3f" % total_d)
    print("PROBE_BOUND_M=%.1f" % bound)
    print("PROBE_RTF=%.2f" % rtf)
    moved = total_d > 0.25
    print("PROBE_RESULT=%s" % ("MOVED" if moved else "DID_NOT_MOVE"))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if moved else 1


if __name__ == "__main__":
    sys.exit(main())
