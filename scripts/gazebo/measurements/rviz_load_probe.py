#!/usr/bin/env python3
"""What RViz is being asked to draw: rate, size, bandwidth and staleness.

Written for the RViz views work (local-notes/RVIZ_VIEWS_PLAN_2026-10-20.md):
before and after changing gazebo_sim.rviz, measure the load each display
receives and how out of date its data is when it arrives. For every topic the
RViz config draws it reports, over a fixed window of WALL time:

  n          messages received in the window (0 if the topic never arrived;
             that is a result, not an error)
  Hz wall    n / wall seconds in the window
  Hz sim     n / simulation seconds advanced in the window
  mean kB    mean serialized size (the raw CDR buffer as delivered, which
             includes the 4-byte encapsulation header); kB = 1000 bytes
  MB/s       total bytes / wall seconds, MB = 1e6 bytes
  age        sim clock now at receipt minus header.stamp, median and 95th
             percentile, in sim seconds: how stale each display's data is
  grid       width x height @ resolution, for the OccupancyGrid topics
  frame      header.frame_id of the last message

and for the run as a whole:

  RTF        simulation seconds advanced / wall seconds, from the first and
             last /clock message received in the window
  sync       for every /lane_path message, its stamp minus the stamp of the
             newest /navigator/search_grid and of the newest /lane_costmap
             received so far (median, 95th percentile, 95th percentile of the
             absolute value). Positive means the grid is older than the path.
             RViz with "Use Timestamp" off draws a grid at the newest TF, not
             at its stamp, so this gap is how far the grid can slide against
             the path line.

Topics (label: topic, type):
  lane_costmap   /lane_costmap                                     OccupancyGrid
  lane_map       /lane_map                                         OccupancyGrid
  lidar_map      /lidar_obstacle_map                               OccupancyGrid
  search_grid    /navigator/search_grid                            OccupancyGrid
  lane_path      /lane_path                                        Path
  overlay        /lane_debug/cam0/overlay                          Image
  camera_rgb     /front_zed_camera_x/zed_node/rgb/color/rect/image Image
  scan           /scan                                             LaserScan
  odom           /odom                                             Odometry
  zed_odom       /front_zed_camera_x/zed_node/odom                 Odometry
  clock          /clock                                            Clock

Run it INSIDE the simulator's container, beside a running sim, ROS sourced:

    docker exec igvc_gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \\
        python3 /root/ros2_ws/src/IGVC_robot_2026/scripts/gazebo/measurements/rviz_load_probe.py \\
        --duration 40 --out /tmp/rviz_load_probe.json"

Options: --duration wall seconds to measure (default 40), --warmup wall
seconds before the window opens (default 2, so DDS discovery does not show up
as a low rate), --out the JSON path (default /tmp/rviz_load_probe.json),
--skip TOPIC (repeatable) to leave a topic out, --self-test to check the CDR
parsing below against rclpy's own deserializer with no ROS graph at all.
Ctrl+C or SIGTERM ends the window early; the table and the JSON are still
written. Exit 0 always, except 2 for a failed --self-test.

How it reads messages: every subscription is raw (the CDR bytes, never
deserialized into Python), BEST_EFFORT, VOLATILE, KEEP_LAST 10. A best-effort
volatile reader matches any publisher, reliable or transient-local, and gives
the publisher nothing to wait for, so it cannot slow a publisher down or
change its QoS. The header stamp, frame_id and grid metadata are read from
fixed CDR offsets (every topic here starts with a std_msgs/Header, and Clock
with a Time); the first message on each topic is also deserialized by rclpy
and compared, and a topic whose parse disagrees falls back to full
deserialization for the rest of the run (reported as "deser" in the JSON).

What can make it lie:
  * It measures what is PUBLISHED to RViz, not what RViz draws. RViz's own
    frame rate (bottom right of its window) and dropped frames are not seen
    here; nor is a "message filter dropping" warning in RViz's log.
  * It adds one more reader to every topic. That costs no publisher any
    waiting, but DDS still copies each image to it: with the camera already
    paced for DDS (docs/GAZEBO_SETUP.md 10.9), --skip the two image topics if
    the question is whether the probe itself changes the sim's speed.
  * A best-effort reader that falls behind drops messages silently, so a
    rate here is a lower bound. /clock is the fastest topic and the first to
    suffer; compare clock Hz sim with the bridge's clock rate. Python handles
    a few thousand callbacks per second on this machine at most.
  * Age uses the probe's own sim clock, which moves only when a /clock
    message arrives, so ages carry up to one clock period of jitter and can be
    slightly negative. Messages with a zero stamp are counted but left out of
    the age figures (reported as zero_stamps).
  * The window is wall time: at an RTF of 0.15, 40 wall seconds is only
    about 6 sim seconds, and a 1 Hz topic gives few samples. Raise
    --duration for slow topics.
  * If the sim restarts during the window, /clock jumps backwards; the jump
    count is reported and the RTF is then meaningless.
"""
import argparse
import json
import math
import signal
import struct
import sys
import time

SELF = "rviz_load_probe"
CAM = "/front_zed_camera_x/zed_node"
# (label, topic, type key)
TOPICS = [
    ("lane_costmap", "/lane_costmap", "grid"),
    ("lane_map", "/lane_map", "grid"),
    ("lidar_map", "/lidar_obstacle_map", "grid"),
    ("search_grid", "/navigator/search_grid", "grid"),
    ("lane_path", "/lane_path", "path"),
    ("overlay", "/lane_debug/cam0/overlay", "image"),
    ("camera_rgb", CAM + "/rgb/color/rect/image", "image"),
    ("scan", "/scan", "scan"),
    ("odom", "/odom", "odom"),
    ("zed_odom", CAM + "/odom", "odom"),
    ("clock", "/clock", "clock"),
]
SYNC_GRIDS = ("/navigator/search_grid", "/lane_costmap")


# -- pure logic, no ROS -------------------------------------------------------

def msg_class(kind):
    if kind == "grid":
        from nav_msgs.msg import OccupancyGrid
        return OccupancyGrid
    if kind == "path":
        from nav_msgs.msg import Path
        return Path
    if kind == "image":
        from sensor_msgs.msg import Image
        return Image
    if kind == "scan":
        from sensor_msgs.msg import LaserScan
        return LaserScan
    if kind == "odom":
        from nav_msgs.msg import Odometry
        return Odometry
    from rosgraph_msgs.msg import Clock
    return Clock


def _endian(raw):
    """'<' or '>' from the CDR encapsulation id, None if not plain CDR."""
    if len(raw) < 12:
        return None
    ident = (raw[0] << 8) | raw[1]
    if ident in (0x0001, 0x0007):   # CDR_LE, CDR2_LE
        return "<"
    if ident in (0x0000, 0x0006):   # CDR_BE, CDR2_BE
        return ">"
    return None


def parse_cdr(raw, kind):
    """{stamp, frame, [width, height, resolution]} from raw CDR, or None.

    Every field read here is 4-byte aligned, which is the same in XCDR1 and
    XCDR2, and alignment counts from the end of the 4-byte encapsulation.
    """
    e = _endian(raw)
    if e is None:
        return None
    sec, nsec = struct.unpack_from(e + "iI", raw, 4)
    out = {"stamp": sec + nsec * 1e-9}
    if kind == "clock":
        return out
    n = struct.unpack_from(e + "I", raw, 12)[0]
    if n < 1 or 16 + n > len(raw):
        return None
    out["frame"] = bytes(raw[16:16 + n - 1]).decode("utf-8", "replace")
    if kind != "grid":
        return out
    p = 16 + n
    p = 4 + ((p - 4 + 3) & ~3)          # align 4 from the payload start
    if p + 20 > len(raw):
        return None
    # MapMetaData: map_load_time (8 bytes), resolution f32, width u32, height u32
    res, w, h = struct.unpack_from(e + "fII", raw, p + 8)
    out.update(resolution=float(res), width=int(w), height=int(h))
    return out


def parse_deser(raw, kind, cls):
    """The same dict as parse_cdr, through rclpy's deserializer."""
    from rclpy.serialization import deserialize_message
    m = deserialize_message(bytes(raw), cls)
    if kind == "clock":
        return {"stamp": m.clock.sec + m.clock.nanosec * 1e-9}
    out = {"stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
           "frame": m.header.frame_id}
    if kind == "grid":
        out.update(resolution=float(m.info.resolution), width=int(m.info.width),
                   height=int(m.info.height))
    return out


def same(a, b):
    if a is None or b is None or set(a) != set(b):
        return False
    for k in a:
        if isinstance(a[k], float):
            if abs(a[k] - b[k]) > 1e-6 * max(1.0, abs(b[k])):
                return False
        elif a[k] != b[k]:
            return False
    return True


def pct(vals, q):
    """Linear-interpolated percentile, None for an empty list."""
    s = sorted(vals)
    if not s:
        return None
    k = (len(s) - 1) * q / 100.0
    f = int(math.floor(k))
    c = min(f + 1, len(s) - 1)
    return s[f] + (s[c] - s[f]) * (k - f)


def rnd(x, nd=4):
    return None if x is None else round(x, nd)


class TopicStats:
    def __init__(self, label, topic, kind):
        self.label, self.topic, self.kind = label, topic, kind
        self.n = 0
        self.bytes = 0
        self.ages = []
        self.zero_stamps = 0
        self.frame = None
        self.grid = None
        self.mode = "cdr"       # "deser" once the CDR parse disagreed
        self.checked = False
        self.parse_errors = 0

    def summary(self, wall, sim):
        d = {"label": self.label, "type": self.kind, "count": self.n,
             "hz_wall": rnd(self.n / wall) if wall > 0 else None,
             "hz_sim": rnd(self.n / sim) if sim and sim > 0 else None,
             "mean_bytes": rnd(self.bytes / self.n, 1) if self.n else None,
             "mb_per_s_wall": rnd(self.bytes / wall / 1e6, 5) if wall > 0 else None,
             "age_median_s": rnd(pct(self.ages, 50)),
             "age_p95_s": rnd(pct(self.ages, 95)),
             "age_n": len(self.ages), "zero_stamps": self.zero_stamps,
             "frame_id": self.frame, "parse": self.mode,
             "parse_errors": self.parse_errors}
        if self.kind == "grid":
            d["grid"] = self.grid
        return d


def sync_summary(diffs):
    return {"n": len(diffs), "median_s": rnd(pct(diffs, 50)),
            "p95_s": rnd(pct(diffs, 95)),
            "p95_abs_s": rnd(pct([abs(x) for x in diffs], 95)),
            "max_abs_s": rnd(max((abs(x) for x in diffs), default=None)
                             if diffs else None)}


def fmt(x, spec, width):
    return ("%" + spec) % x if x is not None else "-".rjust(width)


def table(report):
    r = report
    lines = ["rviz_load_probe: %.1f s wall window (after %.1f s warmup); sim advanced "
             "%s s; RTF %s; /clock jumps back %d"
             % (r["wall_s"], r["warmup_s"],
                "%.2f" % r["sim_advanced_s"] if r["sim_advanced_s"] is not None else "-",
                "%.3f" % r["rtf"] if r["rtf"] is not None else "-",
                r["clock_jumps_back"])]
    lines.append("%-13s %6s %8s %8s %9s %8s %8s %8s  %s"
                 % ("label", "n", "Hz wall", "Hz sim", "mean kB", "MB/s",
                    "age med", "age p95", "grid / frame"))
    for topic, d in r["topics"].items():
        if d.get("skipped"):
            lines.append("%-13s %6s  (skipped) %s" % (d["label"], "-", topic))
            continue
        g = d.get("grid")
        extra = ("%dx%d @%.3f " % (g["width"], g["height"], g["resolution"])
                 if g else "")
        extra += d["frame_id"] or ""
        lines.append("%-13s %6d %8s %8s %9s %8s %8s %8s  %s"
                     % (d["label"], d["count"], fmt(d["hz_wall"], "8.2f", 8),
                        fmt(d["hz_sim"], "8.2f", 8),
                        fmt(d["mean_bytes"] / 1000.0 if d["mean_bytes"] else None,
                            "9.1f", 9),
                        fmt(d["mb_per_s_wall"], "8.3f", 8),
                        fmt(d["age_median_s"], "8.3f", 8),
                        fmt(d["age_p95_s"], "8.3f", 8), extra))
    lines.append("sync: /lane_path stamp minus newest grid stamp (sim s; + = grid older)")
    for name, s in r["sync"].items():
        lines.append("  %-27s n %4d  median %s  p95 %s  p95|x| %s  max|x| %s"
                     % (name, s["n"], fmt(s["median_s"], ".3f", 1),
                        fmt(s["p95_s"], ".3f", 1), fmt(s["p95_abs_s"], ".3f", 1),
                        fmt(s["max_abs_s"], ".3f", 1)))
    return lines


def self_test():
    """Parse serialized sample messages and compare with rclpy's deserializer."""
    from rclpy.serialization import serialize_message
    ok = True
    for frame in ("", "m", "map", "base_link", "odom_x", "abcdefgh"):
        for label, _topic, kind in TOPICS:
            cls = msg_class(kind)
            m = cls()
            if kind == "clock":
                m.clock.sec, m.clock.nanosec = 1234, 567890000
            else:
                m.header.stamp.sec, m.header.stamp.nanosec = 77, 250000000
                m.header.frame_id = frame
            if kind == "grid":
                m.info.resolution = 0.05
                m.info.width, m.info.height = 321, 123
                m.data = [0] * (321 * 123)
            raw = serialize_message(m)
            a, b = parse_cdr(raw, kind), parse_deser(raw, kind, cls)
            if not same(a, b):
                ok = False
                print("SELF-TEST MISMATCH %s frame %r: cdr %s deser %s"
                      % (label, frame, a, b))
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 2


# -- ROS side -----------------------------------------------------------------

def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.signals import SignalHandlerOptions

    stop = {"now": False}

    def _stop(*_):
        stop["now"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(SELF, parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)

    stats = {}
    win = {"open": False, "clock_first": None, "clock_last": None,
           "jumps": 0, "last_clock": None}
    newest_grid = {t: None for t in SYNC_GRIDS}
    sync = {t: [] for t in SYNC_GRIDS}

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def make_cb(st, cls):
        def cb(raw):
            info = None
            if st.mode == "cdr":
                info = parse_cdr(raw, st.kind)
                if not st.checked:
                    st.checked = True
                    try:
                        ref = parse_deser(raw, st.kind, cls)
                    except Exception:  # noqa: BLE001 - counted, not fatal
                        ref = None
                    if not same(info, ref):
                        st.mode, info = "deser", ref
            else:
                try:
                    info = parse_deser(raw, st.kind, cls)
                except Exception:  # noqa: BLE001 - counted, not fatal
                    info = None
            if info is None:
                st.parse_errors += 1
                if win["open"]:
                    st.n += 1
                    st.bytes += len(raw)
                return
            stamp = info["stamp"]
            if st.kind == "clock":
                if win["last_clock"] is not None and stamp < win["last_clock"]:
                    win["jumps"] += 1
                win["last_clock"] = stamp
            if st.topic in newest_grid:
                newest_grid[st.topic] = stamp
            if not win["open"]:
                return
            st.n += 1
            st.bytes += len(raw)
            st.frame = info.get("frame", st.frame)
            if st.kind == "grid":
                st.grid = {"width": info["width"], "height": info["height"],
                           "resolution": rnd(info["resolution"], 5)}
            if st.kind == "clock":
                pair = (time.monotonic(), stamp)
                if win["clock_first"] is None:
                    win["clock_first"] = pair
                win["clock_last"] = pair
            if stamp == 0.0:
                st.zero_stamps += 1
            else:
                now = sim_now()
                if now > 0.0:
                    st.ages.append(now - stamp)
            if st.topic == "/lane_path" and stamp != 0.0:
                for g in SYNC_GRIDS:
                    if newest_grid[g] is not None:
                        sync[g].append(stamp - newest_grid[g])
        return cb

    for label, topic, kind in TOPICS:
        st = TopicStats(label, topic, kind)
        stats[topic] = st
        if topic in args.skip:
            continue
        cls = msg_class(kind)
        node.create_subscription(cls, topic, make_cb(st, cls), qos, raw=True)

    print("rviz_load_probe: %.1f s warmup, then %.1f s window; Ctrl+C ends early"
          % (args.warmup, args.duration), flush=True)
    t0 = time.monotonic()
    while not stop["now"] and time.monotonic() - t0 < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.05)
    win["open"] = True
    w0 = time.monotonic()
    while not stop["now"] and time.monotonic() - w0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    win["open"] = False
    wall = time.monotonic() - w0

    sim_adv = rtf = None
    cf, cl = win["clock_first"], win["clock_last"]
    if cf is not None and cl is not None and cl[0] > cf[0]:
        sim_adv = cl[1] - cf[1]
        rtf = sim_adv / (cl[0] - cf[0])
    topics = {}
    for label, topic, kind in TOPICS:
        if topic in args.skip:
            topics[topic] = {"label": label, "type": kind, "skipped": True}
        else:
            topics[topic] = stats[topic].summary(wall, sim_adv)
    report = {"probe": SELF,
              "wall_start_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                              time.gmtime(time.time() - wall)),
              "warmup_s": args.warmup, "wall_s": round(wall, 3),
              "stopped_early": wall < args.duration - 0.2,
              "sim_advanced_s": rnd(sim_adv), "rtf": rnd(rtf),
              "clock_jumps_back": win["jumps"],
              "qos": "best_effort volatile keep_last 10, raw",
              "topics": topics,
              "sync": {"lane_path_vs_" + g.rsplit("/", 1)[-1]: sync_summary(sync[g])
                       for g in SYNC_GRIDS}}
    node.destroy_node()
    rclpy.try_shutdown()
    return report


def main():
    ap = argparse.ArgumentParser(
        description="Measure the rate, size, bandwidth and staleness of what RViz draws.")
    ap.add_argument("--duration", type=float, default=40.0,
                    help="wall seconds to measure (default 40)")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="wall seconds of discovery before the window (default 2)")
    ap.add_argument("--out", default="/tmp/rviz_load_probe.json",
                    help="JSON output path (default /tmp/rviz_load_probe.json)")
    ap.add_argument("--skip", action="append", default=[], metavar="TOPIC",
                    help="leave this topic out (repeatable)")
    ap.add_argument("--self-test", action="store_true",
                    help="check the CDR parsing against rclpy, no ROS graph needed")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    report = run(args)
    print()
    for line in table(report):
        print(line)
    try:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1)
        print("wrote %s" % args.out)
    except OSError as exc:
        print("rviz_load_probe: cannot write %s: %s" % (args.out, exc))
    return 0


if __name__ == "__main__":
    sys.exit(main())
