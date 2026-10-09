#!/usr/bin/env python3
"""Who is really feeding the planner its lanes and barrels, and is it any good?

Used by the lane_source stage (docs/GAZEBO_SETUP.md 10.20, local-notes/
R1_lane_source_design.md section 4.3). It watches a running
gazebo_nav_test.launch.py and answers, from the ROS graph and the messages
themselves, tests W1 to W6:

  W1  exactly ONE publisher on /lane_costmap and on /lane_map, and it is the
      node lane_source names (gt_nav_bridge_node, lane_detection_node or
      lane_segmentation_node); /gt/lane_* published by gt_nav_bridge_node in
      camera modes and absent in gt mode; /lidar_obstacle_map published by
      gt_obstacle_map_node, lidar_obstacle_costmap_node or nobody.
  W2  the ground truth reaches no planner: nothing among igvc_navigator,
      controller_server, planner_server, local_costmap, global_costmap ever
      subscribed to /gt/lane_costmap or /gt/lane_map; and igvc_navigator does
      subscribe to /lane_costmap.
  W3  /lane_costmap content: frame base_link; the geometry the detector's config
      says (read from the config in use); only the values -1, 0, 100; the stamp
      under 1.0 s old in sim time for 95 percent of messages; at least 50
      lethal cells in at least 50 percent of the messages after the first one
      that had that many. The 1.0 s, 95, 50 and 50 are provisional, fixed
      before the first run (--max-age, --min-age-frac, --min-lethal,
      --min-evidence-frac).
  W4  (with --launch-log) the detector's log: no "falling back to pinhole
      projection" and no "No TF from" (Hough); the "YOLOPv2 ready: 1 instance(s)
      on cuda:0 (half=True)" line and no "falling back to CPU" (YOLOPv2).
  W5  no blind dash: the sim time of the first /lane_costmap with enough lethal
      cells is EARLIER than the sim time the robot first moved 0.3 m.
  W6  (gt_map only) /lidar_obstacle_map holds the barrels: its lethal cell count
      is within 15 percent of the area of the barrel discs (radius + inflation)
      at the grid resolution, and every disc's centre cell is lethal.

It uses a real rclpy node, not `ros2 topic hz`, because the questions are about
publisher and subscriber NAMES and message CONTENTS. The graph is sampled every
0.5 s of wall time for the whole window and everything seen is kept, so a
second publisher that exists for two seconds is still caught.

Run it INSIDE the container (ROS sourced), started before the check, and stop
it with SIGINT or SIGTERM afterwards (or give --duration, in simulation
seconds from the first /clock). The summary prints when it stops:

    python3 scripts/gazebo/measurements/lane_source_probe.py --lane-source hough \\
        --launch-log /tmp/autonomy.log --csv /tmp/probe.csv &
    ... run autonomy_check.sh ...
    kill -INT %1; wait %1

A python3 script is matched by neither teardown pattern of sim_preflight.sh
(they match /opt/ros/jazzy/lib/ and /root/ros2_ws/install/ command lines), so
it survives autonomy_check.sh's own teardown.

The arguments say what the launch was given, so the right answer is known:
    --lane-source gt|hough|yolopv2     (default gt)
    --obstacle-source ...              (default gt; promoted to gt_map when
                                        the lane source is a camera, as the
                                        launch does)
    --gt-outputs true|false            (default true)

Exit: 0 every test PASS or not applicable, 1 any FAIL, 3 none failed but some
could not be decided (no messages, no movement, no graph samples).
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_JSON = os.path.join(REPO, "IGVC_track_generator", "track_points.json")
CONFIG_DIR = os.path.join(REPO, "src", "igvc_test_bringup", "config")

SELF = "lane_source_probe"
LANE_NODE = {"gt": "gt_nav_bridge_node", "hough": "lane_detection_node",
             "yolopv2": "lane_segmentation_node"}
LANE_CONFIG = {"hough": ("lane_detection_config.yaml", "lane_detection_node"),
               "yolopv2": ("lane_segmentation_config.yaml", "lane_segmentation_node")}
TOPICS = ("/lane_costmap", "/lane_map", "/lidar_obstacle_map",
          "/gt/lane_costmap", "/gt/lane_map")
# Nodes that must never be fed the ground truth in a camera run.
FORBIDDEN_ON_GT = ("igvc_navigator", "controller_server", "planner_server",
                   "local_costmap", "global_costmap")
MOVE_M = 0.3   # wait_ready.py "moving" and the autonomy_check gate


# -- pure logic, no ROS: everything below is testable on its own --------------

def bare(name):
    """'/ns/node' -> 'node'."""
    return name.rsplit("/", 1)[-1]


def effective_obstacle(lane, obstacle):
    """The launch's own promotion rule: gt becomes gt_map under a camera."""
    return "gt_map" if lane != "gt" and obstacle == "gt" else obstacle


def expected_publishers(lane, obstacle_eff, gt_outputs):
    """topic -> the set of bare node names that must be its only publisher(s)."""
    exp = {"/lane_costmap": {LANE_NODE[lane]}, "/lane_map": {LANE_NODE[lane]}}
    gt = {"gt_nav_bridge_node"} if (lane != "gt" and gt_outputs) else set()
    exp["/gt/lane_costmap"] = gt
    exp["/gt/lane_map"] = gt
    exp["/lidar_obstacle_map"] = {
        "gt_map": {"gt_obstacle_map_node"},
        "lidar_map": {"lidar_obstacle_costmap_node"}}.get(obstacle_eff, set())
    return exp


def geometry_from_params(p):
    """Expected /lane_costmap geometry from a detector's grid_* parameters.

    width is the FORWARD cell count (grid_height), height the LATERAL one
    (grid_width), origin (0, -grid_width / 2): lane_detection.py:833-841 and
    lane_segmentation.py:3162-3217 build it that way.
    """
    res = float(p["grid_resolution"])
    gw, gh = float(p["grid_width"]), float(p["grid_height"])
    return {"width": int(round(gh / res)), "height": int(round(gw / res)),
            "resolution": res, "origin_x": 0.0, "origin_y": -gw / 2.0}


def geometry_from_bridge_params(p):
    """The same for gt_nav_bridge_node's local crop (_publish_local_costmap)."""
    res = float(p.get("local_costmap_resolution_m", 0.1))
    lat = float(p.get("local_costmap_width_m", 6.0))
    fwd = float(p.get("local_costmap_height_m", 6.0))
    return {"width": int(round(fwd / res)), "height": int(round(lat / res)),
            "resolution": res, "origin_x": 0.0, "origin_y": -lat / 2.0}


def expected_geometry(lane, config_dir):
    """The geometry for this lane source, read from the config in use."""
    import yaml
    if lane == "gt":
        path, key, fn = (os.path.join(config_dir, "isaac_nav_test.yaml"),
                         "gt_nav_bridge_node", geometry_from_bridge_params)
    else:
        name, key = LANE_CONFIG[lane]
        path, fn = os.path.join(config_dir, name), geometry_from_params
    with open(path, encoding="utf-8") as fh:
        params = yaml.safe_load(fh)[key]["ros__parameters"]
    return fn(params)


def count_cells(values):
    """(unknown, free, lethal >= 90, other) of an OccupancyGrid data array."""
    a = np.asarray(values, dtype=np.int8)
    unknown = int(np.count_nonzero(a == -1))
    free = int(np.count_nonzero(a == 0))
    lethal = int(np.count_nonzero(a >= 90))
    other = int(a.size - unknown - free - np.count_nonzero(a == 100))
    return unknown, free, lethal, other


class State:
    """Everything the probe saw."""

    def __init__(self):
        self.graph = {t: {"pubs": set(), "subs": set(), "max_pubs": 0,
                          "samples": 0, "with_pub": 0} for t in TOPICS}
        self.counts = {t: 0 for t in TOPICS}
        self.rows = []          # one dict per /lane_costmap message
        self.lidar_last = None  # summary of the last /lidar_obstacle_map
        self.lane_map_last = None
        self.t_first_move = None
        self.t_start = None
        self.t_end = None


UNRESOLVED = "_NODE_NAME_UNKNOWN_"   # rclpy's name for an endpoint DDS has found
                                     # but whose node has not announced itself yet


def record_graph(state, topic, pub_names, sub_names):
    g = state.graph[topic]
    g["samples"] += 1
    # An endpoint seen before its node name resolves still counts toward
    # max_pubs (two at once is a FAIL), but it is not a differently named
    # publisher: first contact with real rclpy (P3a, 2026-10-09) showed it on
    # the first sample or two of /lidar_obstacle_map, then the real name.
    resolved = [n for n in pub_names if bare(n) != UNRESOLVED]
    g["unresolved"] = g.get("unresolved", 0) + (len(pub_names) - len(resolved))
    g["pubs"].update(resolved)
    g["subs"].update(sub_names)
    g["max_pubs"] = max(g["max_pubs"], len(pub_names))
    if pub_names:
        g["with_pub"] += 1


def barrel_discs(track_json):
    """[(x_m, y_m, radius_m)] and the spawn (x, y), straight from the JSON."""
    with open(track_json, encoding="utf-8") as fh:
        d = json.load(fh)
    pos = d.get("robot_start_pose", {}).get("position_m", {})
    return ([(float(o["x_m"]), float(o["y_m"]), float(o["radius_m"]))
             for o in d.get("obstacles_m", [])],
            (float(pos.get("x", 0.0)), float(pos.get("y", 0.0))))


def summarise_overlay(msg_info, data, barrels, start, inflate):
    """Lethal count and the lethal state of every disc centre, for W6."""
    w, h = msg_info["width"], msg_info["height"]
    res, ox, oy = msg_info["resolution"], msg_info["origin_x"], msg_info["origin_y"]
    a = np.asarray(data, dtype=np.int8)
    lethal = int(np.count_nonzero(a >= 90))
    centres = []
    for x, y, _r in barrels:
        col = int(math.floor((x - start[0] - ox) / res))
        row = int(math.floor((y - start[1] - oy) / res))
        inside = 0 <= col < w and 0 <= row < h
        centres.append(bool(inside and a[row * w + col] >= 90))
    expect = sum(math.pi * (r + inflate) ** 2 for _x, _y, r in barrels) / (res * res)
    return {"lethal": lethal, "centres": centres, "expected_cells": expect,
            "res": res, "width": w, "height": h}


def pct(values, q):
    return float(np.percentile(values, q)) if len(values) else float("nan")


def evaluate(state, cfg):
    """List of (id, status, text). Status: PASS FAIL INCONCLUSIVE NA INFO."""
    out = []
    lane, obs = cfg["lane"], cfg["obstacle_eff"]
    exp = expected_publishers(lane, obs, cfg["gt_outputs"])

    # -- W1: publisher identity ----------------------------------------------
    for topic in ("/lane_costmap", "/lane_map", "/gt/lane_costmap", "/gt/lane_map",
                  "/lidar_obstacle_map"):
        g = state.graph[topic]
        seen = sorted({bare(n) for n in g["pubs"]})
        want = sorted(exp[topic])
        if g["samples"] == 0:
            out.append(("W1", "INCONCLUSIVE", "%s: the graph was never sampled" % topic))
            continue
        ok = seen == want and g["max_pubs"] <= 1
        unresolved = g.get("unresolved", 0)
        out.append(("W1", "PASS" if ok else "FAIL",
                    "%-20s publishers seen %s, expected %s; at most %d at once "
                    "(%d graph samples%s)"
                    % (topic, seen or "none", want or "none", g["max_pubs"],
                       g["samples"],
                       "; %d sample(s) held a publisher whose node name was not "
                       "yet resolved" % unresolved if unresolved else "")))

    # -- W2: the ground truth does not reach the planner ---------------------
    if exp["/gt/lane_costmap"]:
        bad = sorted({bare(n) for t in ("/gt/lane_costmap", "/gt/lane_map")
                      for n in state.graph[t]["subs"]} & set(FORBIDDEN_ON_GT))
        allsubs = sorted({bare(n) for t in ("/gt/lane_costmap", "/gt/lane_map")
                          for n in state.graph[t]["subs"]})
        out.append(("W2", "FAIL" if bad else "PASS",
                    "subscribers of /gt/lane_costmap and /gt/lane_map: %s%s"
                    % (allsubs or "none", "; FORBIDDEN: %s" % bad if bad else "")))
    else:
        out.append(("W2", "NA", "no /gt/lane_* topics are expected in this "
                                "configuration, so nothing to keep from the planner"))
    subs = {bare(n) for n in state.graph["/lane_costmap"]["subs"]}
    if cfg["expect_navigator"]:
        out.append(("W2", "PASS" if "igvc_navigator" in subs else "FAIL",
                    "igvc_navigator %s /lane_costmap (subscribers %s, publisher %s)"
                    % ("subscribes to" if "igvc_navigator" in subs
                       else "does NOT subscribe to", sorted(subs) or "none",
                       sorted({bare(n) for n in state.graph["/lane_costmap"]["pubs"]}) or "none")))

    # -- W3: /lane_costmap content and geometry ------------------------------
    rows = state.rows
    if not rows:
        out.append(("W3", "FAIL", "no /lane_costmap message arrived in the window"))
    else:
        geo = cfg.get("geometry")
        frames = sorted({r["frame"] for r in rows})
        out.append(("W3", "PASS" if frames == ["base_link"] else "FAIL",
                    "frame %s over %d message(s)" % (frames, len(rows))))
        if geo is None:
            out.append(("W3", "INCONCLUSIVE", "geometry not checked: %s"
                        % cfg.get("geometry_note", "no expectation")))
        else:
            bad = [r for r in rows if abs(r["width"] - geo["width"]) > 1
                   or abs(r["height"] - geo["height"]) > 1
                   or abs(r["resolution"] - geo["resolution"]) > 1e-6
                   or abs(r["origin_x"] - geo["origin_x"]) > 1e-3
                   or abs(r["origin_y"] - geo["origin_y"]) > 1e-3]
            r0 = rows[0]
            out.append(("W3", "FAIL" if bad else "PASS",
                        "geometry %dx%d at %.3f origin (%.2f, %.2f); config says "
                        "%dx%d at %.3f origin (%.2f, %.2f); %d message(s) differ"
                        % (r0["width"], r0["height"], r0["resolution"],
                           r0["origin_x"], r0["origin_y"], geo["width"],
                           geo["height"], geo["resolution"], geo["origin_x"],
                           geo["origin_y"], len(bad))))
        other = sum(1 for r in rows if r["other"] > 0)
        out.append(("W3", "FAIL" if other else "PASS",
                    "values only -1, 0, 100: %d message(s) held other values" % other))
        ages = [r["age"] for r in rows]
        fresh = sum(1 for a in ages if a <= cfg["max_age"]) / len(ages)
        out.append(("W3", "PASS" if fresh >= cfg["min_age_frac"] else "FAIL",
                    "stamp age (sim) <= %.1f s in %.1f%% of messages (need %.0f%%); "
                    "median %.2f s, 95th %.2f s, max %.2f s"
                    % (cfg["max_age"], 100 * fresh, 100 * cfg["min_age_frac"],
                       pct(ages, 50), pct(ages, 95), max(ages))))
        ev = [i for i, r in enumerate(rows) if r["lethal"] >= cfg["min_lethal"]]
        if not ev:
            out.append(("W3", "FAIL", "no message ever had %d lethal cells "
                                      "(most: %d)" % (cfg["min_lethal"],
                                                      max(r["lethal"] for r in rows))))
        else:
            after = rows[ev[0]:]
            frac = sum(1 for r in after if r["lethal"] >= cfg["min_lethal"]) / len(after)
            out.append(("W3", "PASS" if frac >= cfg["min_evidence_frac"] else "FAIL",
                        ">= %d lethal cells in %.1f%% of the %d message(s) from the "
                        "first one that had them (need %.0f%%)"
                        % (cfg["min_lethal"], 100 * frac, len(after),
                           100 * cfg["min_evidence_frac"])))

    # -- W4: the detector is what it claims (needs the launch log) -----------
    log = cfg.get("launch_log_text")
    if lane == "gt":
        out.append(("W4", "NA", "no detector in gt mode"))
    elif log is None:
        out.append(("W4", "INCONCLUSIVE", "pass --launch-log /tmp/autonomy.log to check"))
    elif lane == "hough":
        n1 = log.count("falling back to pinhole projection")
        n2 = log.count("No TF from")
        out.append(("W4", "PASS" if n1 == 0 and n2 == 0 else "FAIL",
                    "Hough log: %d 'falling back to pinhole projection', %d 'No TF from'"
                    % (n1, n2)))
    else:
        ready = "YOLOPv2 ready: 1 instance(s) on cuda:0 (half=True)" in log
        cpu = "falling back to CPU" in log
        out.append(("W4", "PASS" if ready and not cpu else "FAIL",
                    "YOLOPv2 log: ready line on cuda:0 %s, CPU-fallback warning %s"
                    % ("present" if ready else "ABSENT", "PRESENT" if cpu else "absent")))

    # -- W5: no blind dash ---------------------------------------------------
    t_ev = next((r["sim"] for r in rows if r["lethal"] >= cfg["min_lethal"]), None)
    t_mv = state.t_first_move
    if t_ev is None:
        out.append(("W5", "FAIL", "the planner's grid never had %d lethal cells, so "
                                  "any movement was blind" % cfg["min_lethal"]))
    elif t_mv is None:
        out.append(("W5", "INCONCLUSIVE", "first evidence at sim %.1f s, but the "
                                          "robot never moved %.1f m in the window"
                                          % (t_ev, MOVE_M)))
    else:
        out.append(("W5", "PASS" if t_ev < t_mv else "FAIL",
                    "first /lane_costmap with >= %d lethal cells at sim %.1f s; "
                    "robot first moved %.1f m at sim %.1f s (%s by %.1f s)"
                    % (cfg["min_lethal"], t_ev, MOVE_M, t_mv,
                       "evidence first" if t_ev < t_mv else "MOVED FIRST",
                       abs(t_mv - t_ev))))

    # -- W6: the barrel overlay ----------------------------------------------
    if obs != "gt_map":
        out.append(("W6", "NA", "obstacle source %s has no barrel overlay node" % obs))
    elif state.lidar_last is None:
        out.append(("W6", "FAIL", "no /lidar_obstacle_map message arrived"))
    else:
        s = state.lidar_last
        lo, hi = 0.85 * s["expected_cells"], 1.15 * s["expected_cells"]
        ok = lo <= s["lethal"] <= hi and all(s["centres"])
        out.append(("W6", "PASS" if ok else "FAIL",
                    "%d lethal cells against %.0f expected from %d disc(s) (allowed "
                    "%.0f to %.0f); disc centres lethal %d of %d"
                    % (s["lethal"], s["expected_cells"], len(s["centres"]), lo, hi,
                       sum(s["centres"]), len(s["centres"]))))
    return out


def info_lines(state):
    rows = state.rows
    span = (state.t_end - state.t_start) if state.t_start is not None and state.t_end else 0.0
    lines = []
    for t in TOPICS:
        lines.append("  %-20s %5d message(s)" % (t, state.counts[t]))
    if rows:
        lethal = [r["lethal"] for r in rows]
        known = [r["free"] + r["lethal"] for r in rows]
        frac = [l / k for l, k in zip(lethal, known) if k]
        lines.append("  /lane_costmap        %.2f msg per sim second over %.0f s; first "
                     "message at sim %.1f s" % (len(rows) / span if span > 0 else float("nan"),
                                              span, rows[0]["sim"]))
        lines.append("  lethal cells         median %d, min %d, max %d; median lethal "
                     "fraction of known cells %.3f"
                     % (pct(lethal, 50), min(lethal), max(lethal),
                        pct(frac, 50) if frac else float("nan")))
    lines.append("  robot first moved    %s" % (
        "sim %.1f s" % state.t_first_move if state.t_first_move is not None
        else "never in the window"))
    return lines


# -- ROS side -----------------------------------------------------------------

def run_probe(args, cfg, barrels, start):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.signals import SignalHandlerOptions

    state = State()
    stop = {"now": False}

    def _stop(*_):
        stop["now"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    # rclpy's own handler would raise inside spin_once; ours lets the loop end
    # and print the summary.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("sim_time,stamp,age,frame,width,height,resolution,origin_x,"
                  "origin_y,unknown,free,lethal,other\n")

    class Probe(Node):
        def __init__(self):
            super().__init__(SELF, parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.odom0 = None
            latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)
            plain = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(OccupancyGrid, "/lane_costmap", self._costmap, latched)
            self.create_subscription(OccupancyGrid, "/lane_map",
                                     lambda m: self._count("/lane_map", m), latched)
            self.create_subscription(OccupancyGrid, "/gt/lane_costmap",
                                     lambda m: self._count("/gt/lane_costmap", m), latched)
            self.create_subscription(OccupancyGrid, "/gt/lane_map",
                                     lambda m: self._count("/gt/lane_map", m), latched)
            self.create_subscription(OccupancyGrid, "/lidar_obstacle_map",
                                     self._overlay, latched)
            self.create_subscription(Odometry, args.odom_topic, self._odom, plain)

        def sim_now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _count(self, topic, _msg):
            state.counts[topic] += 1

        def _info(self, m):
            return {"frame": m.header.frame_id, "width": int(m.info.width),
                    "height": int(m.info.height),
                    "resolution": float(m.info.resolution),
                    "origin_x": float(m.info.origin.position.x),
                    "origin_y": float(m.info.origin.position.y)}

        def _costmap(self, m):
            state.counts["/lane_costmap"] += 1
            now = self.sim_now()
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            unknown, free, lethal, other = count_cells(m.data)
            row = dict(self._info(m), sim=now, stamp=stamp, age=now - stamp,
                       unknown=unknown, free=free, lethal=lethal, other=other)
            state.rows.append(row)
            if csv:
                csv.write("%.3f,%.3f,%.3f,%s,%d,%d,%.4f,%.3f,%.3f,%d,%d,%d,%d\n" % (
                    now, stamp, now - stamp, row["frame"], row["width"],
                    row["height"], row["resolution"], row["origin_x"],
                    row["origin_y"], unknown, free, lethal, other))

        def _overlay(self, m):
            state.counts["/lidar_obstacle_map"] += 1
            if cfg["obstacle_eff"] == "gt_map":
                state.lidar_last = summarise_overlay(
                    self._info(m), m.data, barrels, start, cfg["inflate"])

        def _odom(self, m):
            p = m.pose.pose.position
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.odom0 is None:
                self.odom0 = (p.x, p.y)
            elif (state.t_first_move is None
                  and math.hypot(p.x - self.odom0[0], p.y - self.odom0[1]) >= MOVE_M):
                state.t_first_move = t

    node = Probe()

    def sample_graph():
        for topic in TOPICS:
            def full(i):
                return (i.node_namespace or "/").rstrip("/") + "/" + i.node_name
            pubs = [full(i) for i in node.get_publishers_info_by_topic(topic)]
            subs = [full(i) for i in node.get_subscriptions_info_by_topic(topic)
                    if i.node_name != SELF]
            record_graph(state, topic, pubs, subs)

    wall0 = time.time()
    wall_cap = max(600.0, 10.0 * args.duration) if args.duration > 0 else None
    next_sample = 0.0
    print("lane_source_probe: watching (lane_source %s, obstacle_source %s%s); "
          "SIGINT or SIGTERM prints the summary"
          % (args.lane_source, cfg["obstacle_eff"],
             "" if args.obstacle_source == cfg["obstacle_eff"]
             else ", promoted from %s" % args.obstacle_source), flush=True)
    while not stop["now"] and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now_wall = time.time()
        if now_wall >= next_sample:
            next_sample = now_wall + 0.5
            sample_graph()
        sim = node.sim_now()
        if state.t_start is None and sim > 0.0:
            state.t_start = sim
        if sim > 0.0:
            state.t_end = sim
        if (args.duration > 0 and state.t_start is not None
                and sim - state.t_start >= args.duration):
            break
        if wall_cap is not None and now_wall - wall0 > wall_cap:
            print("lane_source_probe: wall ceiling of %.0f s hit" % wall_cap)
            break
    sample_graph()
    if csv:
        csv.close()
    node.destroy_node()
    rclpy.try_shutdown()
    return state


def main():
    ap = argparse.ArgumentParser(
        description="Watch gazebo_nav_test.launch.py and judge tests W1 to W6.")
    ap.add_argument("--lane-source", choices=sorted(LANE_NODE), default="gt")
    ap.add_argument("--obstacle-source", default="gt",
                    choices=("gt", "gt_map", "none", "nav2_scan", "lidar_map"))
    ap.add_argument("--gt-outputs", choices=("true", "false"), default="true")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="simulation seconds to watch from the first /clock "
                         "(default: until SIGINT or SIGTERM)")
    ap.add_argument("--odom-topic", default="/odom",
                    help="where to see the robot move (default /odom, as "
                         "pose_logger.py and wait_ready.py)")
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--config-dir", default=CONFIG_DIR,
                    help="directory holding the lane configs and isaac_nav_test.yaml")
    ap.add_argument("--geometry", default=None, metavar="W,H,RES,OX,OY",
                    help="override the expected /lane_costmap geometry")
    ap.add_argument("--inflate", type=float, default=None,
                    help="barrel inflation for W6 (default: the shared YAML)")
    ap.add_argument("--launch-log", default=None,
                    help="the launch log for W4, e.g. /tmp/autonomy.log")
    ap.add_argument("--no-navigator", action="store_true",
                    help="nav2:=false run: do not expect igvc_navigator")
    ap.add_argument("--csv", default=None, help="write one row per /lane_costmap message")
    ap.add_argument("--json-out", default=None, help="write the verdicts as JSON")
    ap.add_argument("--max-age", type=float, default=1.0)
    ap.add_argument("--min-age-frac", type=float, default=0.95)
    ap.add_argument("--min-lethal", type=int, default=50)
    ap.add_argument("--min-evidence-frac", type=float, default=0.5)
    args = ap.parse_args()

    obs_eff = effective_obstacle(args.lane_source, args.obstacle_source)
    cfg = {"lane": args.lane_source, "obstacle_eff": obs_eff,
           "gt_outputs": args.gt_outputs == "true",
           "expect_navigator": not args.no_navigator,
           "max_age": args.max_age, "min_age_frac": args.min_age_frac,
           "min_lethal": args.min_lethal, "min_evidence_frac": args.min_evidence_frac}

    # Expected geometry: from the config in use, so a config change cannot
    # silently break the test.
    if args.geometry:
        w, h, res, ox, oy = [float(v) for v in args.geometry.split(",")]
        cfg["geometry"] = {"width": int(w), "height": int(h), "resolution": res,
                           "origin_x": ox, "origin_y": oy}
    else:
        try:
            cfg["geometry"] = expected_geometry(args.lane_source, args.config_dir)
        except Exception as exc:  # noqa: BLE001 - reported as INCONCLUSIVE
            cfg["geometry"], cfg["geometry_note"] = None, "cannot read the config (%s)" % exc

    barrels, start = barrel_discs(args.json)
    inflate = args.inflate
    if inflate is None:
        inflate = 0.30
        try:
            import yaml
            with open(os.path.join(args.config_dir, "isaac_nav_test.yaml"),
                      encoding="utf-8") as fh:
                inflate = float(yaml.safe_load(fh)["gt_nav_bridge_node"]
                                ["ros__parameters"]["obstacle_inflate_radius_m"])
        except Exception:  # noqa: BLE001 - the bridge's own code default
            pass
    cfg["inflate"] = inflate

    state = run_probe(args, cfg, barrels, start)

    if args.launch_log:
        try:
            with open(args.launch_log, encoding="utf-8", errors="replace") as fh:
                cfg["launch_log_text"] = fh.read()
        except OSError as exc:
            print("lane_source_probe: cannot read %s: %s" % (args.launch_log, exc))
    results = evaluate(state, cfg)

    print()
    print("lane_source_probe summary: lane_source %s, obstacle_source %s, "
          "gt_outputs %s" % (args.lane_source, obs_eff, args.gt_outputs))
    for line in info_lines(state):
        print(line)
    print()
    for tid, status, text in results:
        print("  %-3s %-12s %s" % (tid, status, text))
    fails = [r for r in results if r[1] == "FAIL"]
    undecided = [r for r in results if r[1] == "INCONCLUSIVE"]
    print()
    print("  %d FAIL, %d INCONCLUSIVE, %d PASS"
          % (len(fails), len(undecided), sum(1 for r in results if r[1] == "PASS")))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump({"lane_source": args.lane_source, "obstacle_source": obs_eff,
                       "gt_outputs": cfg["gt_outputs"],
                       "results": [{"id": r[0], "status": r[1], "text": r[2]}
                                   for r in results]}, fh, indent=1)
    return 1 if fails else (3 if undecided else 0)


if __name__ == "__main__":
    sys.exit(main())
