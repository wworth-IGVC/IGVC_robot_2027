#!/usr/bin/env python3
"""F17 check A: does Nav2's lifecycle manager abort when a node's get_state
reply takes longer than 2 s, and can a second STARTUP recover?

No simulator, no Gazebo, no GPU: run it inside the Gazebo container (or any
Jazzy environment with nav2_lifecycle_manager) in about 15 s per case.

    python3 scripts/gazebo/measurements/lifecycle_get_state_probe.py --delay 1.0
    python3 scripts/gazebo/measurements/lifecycle_get_state_probe.py --delay 2.6 --retry

WHAT IT DOES
The REAL nav2 lifecycle_manager runs as a subprocess, autostart on, bond check
off (bond_timeout 0, because the stand-ins below do not speak the bond
protocol), managing two stand-in lifecycle nodes, fake_a and fake_b. They
implement exactly the two services the manager uses, <name>/change_state and
<name>/get_state, with the real state machine's four transitions. fake_b holds
back the get_state reply that follows its configure by --delay seconds, once.

WHY
2026-10-01 and 2026-10-09: "Failed to change state for node: collision_monitor.
Exception: collision_monitor/get_state service client: async_send_request
failed." In the Jazzy sources, the manager calls change_state (no timeout) and
then get_state, and nav2_util's LifecycleServiceClient::get_state defaults to a
2 s timeout that no parameter reaches. This probe measures that threshold with
the real manager, so the claim can fail:

    --delay 1.0   expect NO abort            (inside the 2 s)
    --delay 1.8   expect NO abort
    --delay 2.3   expect ABORT, naming fake_b/get_state, about 2.0 s after
                  "Configuring fake_b"
    --delay 2.6   expect ABORT

and with --retry, after an abort it clears the delay and sends the manager a
second STARTUP (ManageLifecycleNodes command STARTUP), expecting "Managed
nodes are active". That is the claim a lifecycle-level retry would rest on:
the manager ignores the success flag of change_state and judges by get_state,
so re-configuring an already configured node is harmless. If this prints
RETRY_ACTIVE=NO, do not build a retry on it.

Exit 0 when the observation matches the expectation for --delay (below 1.9:
no abort; above 2.2: abort; in between: not judged), else 1. The last line is
a one-line machine-readable summary.

NOT RUN when written (no ROS on the authoring machine): py_compile only.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes

MANAGER = "lifecycle_manager_f17"
# transition id -> (state it needs, state it leads to)
TRANSITIONS = {
    Transition.TRANSITION_CONFIGURE: (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE),
    Transition.TRANSITION_CLEANUP: (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_UNCONFIGURED),
    Transition.TRANSITION_ACTIVATE: (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE),
    Transition.TRANSITION_DEACTIVATE: (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE),
}
LABELS = {State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
          State.PRIMARY_STATE_INACTIVE: "inactive",
          State.PRIMARY_STATE_ACTIVE: "active"}


class FakeLifecycle(Node):
    """Just enough of a lifecycle node for the manager: two services."""

    def __init__(self, name):
        super().__init__(name)
        self.state = State.PRIMARY_STATE_UNCONFIGURED
        self.delay_after_configure = 0.0   # seconds; applies to the next configure only
        self._pending_delay = 0.0
        grp = ReentrantCallbackGroup()
        self.create_service(ChangeState, name + "/change_state", self._change,
                            callback_group=grp)
        self.create_service(GetState, name + "/get_state", self._get,
                            callback_group=grp)

    def _change(self, req, res):
        need, to = TRANSITIONS.get(req.transition.id, (None, None))
        if self.state == need:
            self.state = to
            res.success = True
            if req.transition.id == Transition.TRANSITION_CONFIGURE:
                self._pending_delay = self.delay_after_configure
                self.delay_after_configure = 0.0
        else:
            res.success = False     # an invalid transition is refused, state unchanged
        return res

    def _get(self, req, res):
        hold, self._pending_delay = self._pending_delay, 0.0
        if hold > 0.0:
            time.sleep(hold)
        res.current_state.id = self.state
        res.current_state.label = LABELS.get(self.state, "unknown")
        return res


class Reader(threading.Thread):
    """Collects the manager's output lines."""

    def __init__(self, proc):
        super().__init__(daemon=True)
        self.proc, self.lines = proc, []

    def run(self):
        for ln in self.proc.stdout:
            self.lines.append(ln.rstrip("\n"))

    def find(self, text, start=0):
        for i in range(start, len(self.lines)):
            if text in self.lines[i]:
                return i
        return -1

    def wait_for(self, texts, timeout, start=0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for t in texts:
                i = self.find(t, start)
                if i >= 0:
                    return t, i
            time.sleep(0.1)
        return None, -1


def stamp(line):
    m = re.search(r"\[(\d{9,}\.\d+)\]", line)
    return float(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay", type=float, required=True,
                    help="seconds fake_b holds back the get_state reply after its configure")
    ap.add_argument("--retry", action="store_true",
                    help="after an abort, send a second STARTUP and expect active nodes")
    a = ap.parse_args()

    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=6)
    fa, fb = FakeLifecycle("fake_a"), FakeLifecycle("fake_b")
    fb.delay_after_configure = a.delay
    probe = Node("f17_probe")
    for n in (fa, fb, probe):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT="0")
    proc = subprocess.Popen(
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
         "-r", "__node:=" + MANAGER,
         "-p", "autostart:=true", "-p", "bond_timeout:=0.0",
         "-p", "node_names:=[fake_a, fake_b]"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
        start_new_session=True)
    rd = Reader(proc)
    rd.start()

    hit, idx = rd.wait_for(["Managed nodes are active", "Aborting bringup"], 40)
    aborted = hit is not None and "Aborting" in hit
    gap = None
    cause = ""
    if aborted:
        cfg = rd.find("Configuring fake_b")
        err = rd.find("Failed to change state for node: fake_b")
        if err >= 0:
            cause = rd.lines[err].split("]: ", 1)[-1]
            if cfg >= 0 and stamp(rd.lines[cfg]) and stamp(rd.lines[err]):
                gap = stamp(rd.lines[err]) - stamp(rd.lines[cfg])
    print("first attempt : %s" % ("ABORTED" if aborted else
                                   ("ACTIVE" if hit else "NO RESULT in 40 s")))
    if cause:
        print("  manager says: %s" % cause)
    if gap is not None:
        print("  \"Configuring fake_b\" to the error: %.2f s (2.0 s timeout plus the configure)" % gap)

    retry_active = None
    if aborted and a.retry:
        fb.delay_after_configure = 0.0
        cli = probe.create_client(ManageLifecycleNodes, "/%s/manage_nodes" % MANAGER)
        if not cli.wait_for_service(timeout_sec=10.0):
            print("  manage_nodes service not available")
            retry_active = False
        else:
            mark = len(rd.lines)
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.STARTUP
            fut = cli.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 40:
                time.sleep(0.1)
            ok = fut.done() and fut.result() is not None and fut.result().success
            seen, _ = rd.wait_for(["Managed nodes are active"], 5, start=mark)
            retry_active = bool(ok and seen)
            print("second STARTUP: service success=%s, 'Managed nodes are active' seen=%s, "
                  "fake_a=%s fake_b=%s" % (ok, bool(seen), LABELS.get(fa.state), LABELS.get(fb.state)))

    # tear down: the manager's whole process group
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=8)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
    rclpy.try_shutdown()

    if a.delay < 1.9:
        expected = "no abort"
        good = not aborted and hit is not None
    elif a.delay > 2.2:
        expected = "abort naming fake_b/get_state"
        good = aborted and "fake_b/get_state" in cause
    else:
        expected, good = "not judged (inside the boundary band)", True
    if a.retry and aborted:
        good = good and bool(retry_active)
    print("DELAY=%.1f ABORTED=%s GAP_S=%s RETRY_ACTIVE=%s EXPECTED=%s RESULT=%s" % (
        a.delay, "YES" if aborted else "NO",
        "%.2f" % gap if gap is not None else "-",
        {None: "-", True: "YES", False: "NO"}[retry_active], expected,
        "PASS" if good else "FAIL"))
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
