#!/usr/bin/env python3
"""Wait until the simulated stack is actually ready, instead of sleeping a
fixed number of seconds and hoping.

Why this exists
---------------
The check scripts used to `sleep 35` and `sleep 50` before looking at anything.
A fixed wall-clock wait is wrong in both directions: on a fast machine it
wastes time, and on a slow one (tier C, or a Mac running the image under
emulation) the stack is not up yet when the checks start. The fix is to wait
for the thing itself, with a generous ceiling, so the same script is correct
on every machine and nobody has to guess a multiplier.

Modes
-----
  odom LIMIT_S
      Ready when /clock has advanced at least 2 s of simulation time AND an
      /odom message has arrived. /odom is published by gazebo_odom_shim from
      bridged Gazebo odometry, so it proves Gazebo, the robot model, the
      DiffDrive plugin, the bridge and the shim are all running.

  moving LIMIT_S [METRES]
      Ready when the robot has moved METRES (default 0.3) from where it was
      when this started, on /odom. Used by autonomy_check.sh: the watch window
      should open when the robot is driving itself, not after a guess at how
      long Nav2's ten lifecycle nodes take to come up.

LIMIT_S is a WALL-clock ceiling. Exit 0 when ready, 1 on timeout. Either way
it prints how long it took in wall and simulation time, which is itself a
useful number on a new machine.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class Waiter(Node):
    def __init__(self):
        super().__init__(
            "wait_ready",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.clock0 = None
        self.clock = None
        self.odom0 = None
        self.odom = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock0 is None:
            self.clock0 = t
        self.clock = t

    def _on_odom(self, m):
        p = m.pose.pose.position
        if self.odom0 is None:
            self.odom0 = (p.x, p.y)
        self.odom = (p.x, p.y)


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("odom", "moving"):
        print("usage: wait_ready.py odom|moving LIMIT_S [METRES]")
        return 2
    mode, limit = sys.argv[1], float(sys.argv[2])
    metres = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3

    rclpy.init()
    n = Waiter()
    wall0 = time.time()
    ready = False
    last_note = wall0
    while time.time() - wall0 < limit:
        rclpy.spin_once(n, timeout_sec=0.1)
        sim_adv = (n.clock - n.clock0) if n.clock is not None else 0.0
        if mode == "odom":
            ready = sim_adv >= 2.0 and n.odom is not None
        else:
            if n.odom0 is not None and n.odom is not None:
                ready = math.hypot(n.odom[0] - n.odom0[0], n.odom[1] - n.odom0[1]) >= metres
        if ready:
            break
        if time.time() - last_note >= 15:
            last_note = time.time()
            print("  ...still waiting (%s): %.0f s wall, %.1f s sim, odom %s"
                  % (mode, time.time() - wall0, sim_adv,
                     "yes" if n.odom is not None else "no"), flush=True)

    wall = time.time() - wall0
    sim_adv = (n.clock - n.clock0) if n.clock is not None else 0.0
    if ready:
        print("  READY (%s) after %.1f s wall, %.1f s of sim time since /clock was first seen"
              % (mode, wall, sim_adv))
    else:
        print("  NOT READY (%s) after the %.0f s ceiling: /clock %s, /odom %s"
              % (mode, limit, "running" if n.clock is not None else "SILENT",
                 "publishing" if n.odom is not None else "SILENT"))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
