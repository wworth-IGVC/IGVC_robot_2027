#!/usr/bin/env python3
"""Score a lane detector against the lane lines the simulator actually paints.

WHY THIS EXISTS RATHER THAN lane_eval_node

`lane_eval_node` scores `/lane_map` against `/lane_ground_truth` and has
returned identically zero since 2026-05-14 11:12. Two independent reasons:

1. ENCODING. It thresholds both grids at `occupied_threshold: 50`, but
   `track_ground_truth_node` writes only 0 (free corridor) and -1 (unknown) and
   never anything >= 50. Commit 37832a1 changed the corridor from 100 to 0 five
   hours after the file was written, so Nav2's `lethal_cost_threshold >= 90` and
   `_extract_centreline`'s `data == 0` would consume it. The evaluator was never
   updated. With an empty ground-truth set the metrics degenerate: IoU 0,
   precision 0, recall 1.0 by the zero-division convention at
   `lane_eval_node.py:158`.

2. SEMANTICS, which is the deeper problem. `/lane_ground_truth` is a FILLED
   CORRIDOR. `/lane_map` is PAINTED LANE LINES. The corridor's interior is
   exactly where the lines are not, so the two are near-disjoint by
   construction and even a perfect detector scores near zero. Fixing the
   threshold alone would not fix that.

`lane_eval_node.py` lives under `src/igvc_lane_detection/`, which this task may
not modify, so this is a sim-side replacement rather than a patch.

WHAT THE GROUND TRUTH IS HERE

The lane LINES, generated from the same `track_points.json` centreline and the
same variable 10-to-20 ft width profile that `generate_igvc_world.py` paints
into the world. So the reference and the thing the camera sees are the same
geometry by construction, not two files that happen to agree.

TWO SCORES, DELIBERATELY

Cell IoU is reported because it is the conventional number, but it is a poor
one here: a painted line is under one cell wide at 0.125 m resolution, so a
single-cell lateral offset can drive IoU to zero while the detection is
perfectly usable. The distance-tolerant scores are the honest ones:

    hit rate      fraction of PREDICTED lane cells within TOL of real paint
                  (precision-like: how much of what it drew is really there)
    coverage      fraction of REAL paint within TOL of a predicted cell
                  (recall-like: how much of the paint it found)

TOL defaults to 0.25 m, two grid cells, which is about three lane-line widths.
State it with every result; the numbers are meaningless without it.

Unknown cells (-1) are never counted as either positive or negative.

Usage, inside the Gazebo container with the simulator running:
    lane_eval_sim.py [--topic /lane_map] [--tol 0.25] [--duration 60]
"""

import argparse
import io
import json
import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from generate_igvc_world import (LINE_WIDTH_M, offset_polyline,  # noqa: E402
                                 resample, track_inner_edge_m)

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
DEFAULT_JSON = os.path.join(REPO, "IGVC_track_generator", "track_points.json")


def lane_line_points(track_json, step_m=0.10):
    """Both painted lane lines, in the ODOM frame, as an (N, 2) array.

    Mirrors generate_igvc_world.main() exactly: same resample, same per-point
    inner edge, same half-line-width offset to the line centre. Then shifted so
    the robot's spawn is the origin, because gazebo_odom_shim starts /odom at
    zero there and `/lane_map` is published in a frame that is identity to odom.
    """
    with io.open(track_json, encoding="utf-8") as fh:
        d = json.load(fh)
    centre = [(p["x"], p["y"]) for p in d["centerline_m"]]
    start = d["robot_start_pose"]["position_m"]
    gap = math.hypot(centre[0][0] - centre[-1][0], centre[0][1] - centre[-1][1])
    closed = gap < 1.0
    if closed:
        centre = centre[:-1]
    coarse, prog = resample(centre, step_m)
    offs = [track_inner_edge_m(p) + LINE_WIDTH_M / 2.0 for p in prog]
    left = offset_polyline(coarse, offs, closed)
    right = offset_polyline(coarse, [-o for o in offs], closed)
    pts = np.array(left + right, dtype=np.float64)
    return pts - np.array([start["x"], start["y"]])


def grid_occupied_xy(msg, thresh=50):
    """World-frame (x, y) centres of every occupied cell in an OccupancyGrid."""
    data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                      msg.info.width)
    ys, xs = np.nonzero(data >= thresh)
    res = msg.info.resolution
    ox = msg.info.origin.position.x
    oy = msg.info.origin.position.y
    return (np.stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res], axis=-1),
            data)


def nearest_dists(a, b, chunk=4000):
    """Min distance from every point in a to any point in b, chunked."""
    if len(a) == 0 or len(b) == 0:
        return np.array([])
    out = np.empty(len(a))
    for i in range(0, len(a), chunk):
        blk = a[i:i + chunk]
        d = np.linalg.norm(blk[:, None, :] - b[None, :, :], axis=2)
        out[i:i + chunk] = d.min(axis=1)
    return out


class Eval(Node):
    def __init__(self, topic):
        super().__init__("lane_eval_sim")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.msg = None
        self.create_subscription(OccupancyGrid, topic,
                                 self._cb, qos)

    def _cb(self, m):
        self.msg = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/lane_map")
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--tol", type=float, default=0.25,
                    help="distance tolerance in metres (default 0.25, two cells)")
    ap.add_argument("--obs-radius", type=float, default=1.5,
                    help="a reference point counts as observed if it is within "
                         "this distance of a cell the detector marked known "
                         "(default 1.5 m)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="how long to wait for a grid with content")
    args = ap.parse_args()

    ref = lane_line_points(args.json)
    print("  reference paint      : %d points from track_points.json, "
          "same geometry the world is painted from" % len(ref))

    rclpy.init()
    n = Eval(args.topic)
    import time
    t0 = time.time()
    best = None
    while time.time() - t0 < args.duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.msg is None:
            continue
        pred, _ = grid_occupied_xy(n.msg)
        if len(pred) > 0:
            best = n.msg
    if best is None:
        print("  FAIL: no grid on %s within %.0f s" % (args.topic, args.duration))
        return 2

    pred, data = grid_occupied_xy(best)
    known = int(np.sum(np.array(best.data, dtype=np.int16) >= 0))
    print("  %-20s : %dx%d at %.3f m, origin (%.2f, %.2f), frame %s"
          % (args.topic, best.info.width, best.info.height,
             best.info.resolution, best.info.origin.position.x,
             best.info.origin.position.y, best.header.frame_id))
    print("  occupied cells       : %d   (known cells %d, rest unknown and "
          "counted as neither)" % (len(pred), known))
    if len(pred) == 0:
        print("  FAIL: grid has no occupied cells; nothing to score")
        return 1

    # Restrict the reference to the area the grid actually covers, otherwise
    # coverage is scored against paint the detector was never shown.
    res = best.info.resolution
    x0 = best.info.origin.position.x
    y0 = best.info.origin.position.y
    x1 = x0 + best.info.width * res
    y1 = y0 + best.info.height * res
    inside = ((ref[:, 0] >= x0) & (ref[:, 0] < x1)
              & (ref[:, 1] >= y0) & (ref[:, 1] < y1))
    ref_in = ref[inside]
    print("  reference in window  : %d of %d points" % (len(ref_in), len(ref)))
    if len(ref_in) == 0:
        print("  FAIL: no reference paint inside the grid window")
        return 1

    # Cell IoU, reported for continuity with lane_eval_node even though it is
    # the weaker of the two scores at this resolution.
    gt_cells = set()
    for x, y in ref_in:
        gt_cells.add((int((x - x0) / res), int((y - y0) / res)))
    pr_cells = set()
    for x, y in pred:
        pr_cells.add((int((x - x0) / res), int((y - y0) / res)))
    inter = len(gt_cells & pr_cells)
    union = len(gt_cells | pr_cells)
    iou = inter / union if union else float("nan")

    d_pred = nearest_dists(pred, ref_in)
    d_ref = nearest_dists(ref_in, pred)
    hit = float(np.mean(d_pred <= args.tol))
    cov = float(np.mean(d_ref <= args.tol))

    # Coverage over the whole window is unfair to a camera: a persistent map
    # only marks cells the robot has actually observed, and in one run it sees
    # a fraction of the lap. Scoring it against paint that was never in view
    # measures how far the robot drove, not how well the detector works.
    # So also report coverage restricted to reference paint that lies near a
    # cell the detector marked KNOWN (>= 0, i.e. free or occupied rather than
    # unexplored). That is the region it had an opinion about.
    known_xy, _ = grid_occupied_xy(best, thresh=0)
    cov_obs = float("nan")
    n_obs = 0
    if len(known_xy):
        d_known = nearest_dists(ref_in, known_xy)
        seen = d_known <= args.obs_radius
        n_obs = int(seen.sum())
        if n_obs:
            cov_obs = float(np.mean(d_ref[seen] <= args.tol))

    print()
    print("  cell IoU             : %.4f   (%d intersect / %d union)"
          % (iou, inter, union))
    print("  hit rate  @ %.2f m   : %.3f   of predicted paint is really paint"
          % (args.tol, hit))
    print("  coverage, whole window: %.3f   of ALL paint in the grid"
          % cov)
    print("  coverage, observed    : %.3f   of the %d reference points within "
          "%.1f m of an observed cell" % (cov_obs, n_obs, args.obs_radius))
    print("  predicted->paint dist: median %.3f m, 90th pct %.3f m"
          % (float(np.median(d_pred)), float(np.percentile(d_pred, 90))))
    print("  paint->predicted dist: median %.3f m, 90th pct %.3f m"
          % (float(np.median(d_ref)), float(np.percentile(d_ref, 90))))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
