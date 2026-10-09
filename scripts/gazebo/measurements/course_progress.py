#!/usr/bin/env python3
"""How far along the course did the robot really get?

autonomy_check.sh grades PROGRESS as "the two furthest-apart poses are at least
5 m apart". That cannot tell a lap from a straight dash: a robot that drives
blind in a line scores the same as one that follows the course. This script
scores the thing the stage cares about, the ARC LENGTH ADVANCED ALONG THE
CENTRELINE, in the course direction, from the pose log autonomy_check.sh leaves
in /tmp/track_log.txt. It never talks to ROS and runs anywhere with Python 3.

Inputs
    LOG ...    pose logs, one line per sample: "x y sim_time [yaw]". The x, y
               are WORLD coordinates (odom plus the spawn point, which
               pose_logger.py adds), the same frame as track_points.json.
               Lines that do not parse are skipped and counted.
    --json     track_points.json (default: the repo's)
    --log-frame world|odom
               world (default) is what pose_logger.py writes. odom means the
               poses are relative to the spawn point (a log taken straight from
               /odom); the centreline is then shifted by minus the spawn
               position, the same shift lane_eval_sim.py applies to the paint.

The centreline is centerline_m, in order. Its direction is the course
direction: the robot is spawned at index 0 facing along it. The path is a LOOP
if its first and last points are within --closed-gap (1.0 m, as lane_eval_sim.py
decides); the closing segment is then part of the line, and arc length can
exceed one lap. Otherwise it is an OPEN path, and progress stops at its end.
track_points.json today: 1000 points, closed (gap 0.028 m), 78.83 m.

THE RULE (what counts as progress, and what does not)
    1. Every pose is projected onto the centreline (nearest point on the
       polyline). The first pose is projected onto the whole line; every later
       one only onto the stretch within --max-jump (3.0 m) of arc either side
       of where the tracker is, so a pose near a different part of the course
       cannot pull the tracker across.
    2. OFF-COURSE: a pose farther than --max-offset (3.5 m) from the centreline
       is not credited and the tracker does not move. The lane half-width is
       1.52 to 3.05 m, so such a pose is off the paint everywhere.
    3. JUMP: a pose whose projection is more than --max-jump (3.0 m) of arc from
       the tracker is ignored the same way. A real robot moves a few tenths of a
       metre between 10 Hz samples; 3 m is a log gap, a teleport or a reset.
    4. Otherwise the tracker moves to the projection, forward or backward.
    5. Credit = the furthest the tracker ever got minus where it started. Going
       backwards lowers the tracker but never the credit, and ground covered
       again adds nothing until the old maximum is passed. Driving the course
       the wrong way credits nothing.
    Consequence, on purpose: after an off-course excursion or a jump the tracker
    stays where it was, so a robot that skips ahead of the 3 m window by cutting
    a corner gets no credit for the skip or for anything later. Conservative.
    The count of rejected samples is printed; a large one means the log or the
    limits are wrong, not that the rule is.

The credit is measured from the FIRST LOGGED POSE, which autonomy_check.sh
takes after the robot has already moved 0.3 m, so about 0.3 to 1 m of the lap is
never counted, the same for every run being compared.

Gate
    --min-fraction-of REF_METRES --fraction F
        exit 1 unless arc advanced >= F x REF_METRES. REF_METRES should be the
        arc advanced by a ground-truth run of the same session, as this script
        prints it (use --arc-only on those logs; with several logs it prints
        their median), NOT the odometric distance travelled, which is longer
        than the centreline because the robot weaves. F defaults to 0.8. The
        gate also fails if the first pose is not at the start of the course
        (within --start-tol, 3.0 m, along the line and --max-offset across it),
        because a run that did not start there cannot be compared.
    Exit codes: 0 pass or no gate, 1 gate failed, 2 no usable input.

    --arc-only    print one number, the arc advanced in metres (the median over
                  the logs if there are several), and nothing else.

Examples
    python3 course_progress.py /tmp/track_log.txt
    REF=$(python3 course_progress.py --arc-only gt_run1.txt gt_run2.txt gt_run3.txt)
    python3 course_progress.py /tmp/track_log.txt --min-fraction-of $REF --fraction 0.8
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_JSON = os.path.join(REPO, "IGVC_track_generator", "track_points.json")
DEFAULT_LOG = "/tmp/track_log.txt"


class Centreline:
    """The course centreline as a polyline with cumulative arc length.

    Vertex i is at arc[i]. Segment i joins vertex i to vertex (i + 1) % n, so a
    closed loop has n segments (the last one closes the loop) and an open path
    n - 1. total is the length of the line, closing segment included.
    """

    def __init__(self, points, closed):
        self.pts = points
        self.n = len(points)
        self.closed = closed
        self.nseg = self.n if closed else self.n - 1
        self.arc = [0.0]
        self.seglen = []
        for i in range(self.nseg):
            ax, ay = points[i]
            bx, by = points[(i + 1) % self.n]
            length = math.hypot(bx - ax, by - ay)
            self.seglen.append(length)
            self.arc.append(self.arc[-1] + length)
        self.total = self.arc[-1]

    def wrap(self, delta):
        """delta in arc, folded into [-total/2, total/2) on a loop."""
        if self.closed and self.total > 0.0:
            return (delta + self.total / 2.0) % self.total - self.total / 2.0
        return delta

    def project(self, x, y, centre=None, window=None):
        """(distance, arc position) of the nearest point on the line.

        With centre and window, only segments whose midpoint is within
        window + half a segment of arc from `centre` are considered.
        """
        best_d, best_s = float("inf"), 0.0
        for i in range(self.nseg):
            length = self.seglen[i]
            if centre is not None:
                mid = self.arc[i] + length / 2.0
                if abs(self.wrap(mid - centre)) > window + length / 2.0:
                    continue
            ax, ay = self.pts[i]
            bx, by = self.pts[(i + 1) % self.n]
            dx, dy = bx - ax, by - ay
            if length == 0.0:
                t = 0.0
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy)
                                 / (length * length)))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d < best_d:
                best_d, best_s = d, self.arc[i] + t * length
        return best_d, best_s

    def index_at(self, arc_pos):
        """Index of the last vertex at or before arc_pos (mod total on a loop)."""
        s = arc_pos % self.total if self.closed and self.total > 0.0 else arc_pos
        lo, hi = 0, self.nseg
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.arc[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return min(lo, self.n - 1)


def load_centreline(track_json, log_frame, closed_gap):
    with open(track_json, encoding="utf-8") as fh:
        d = json.load(fh)
    pts = [(float(p["x"]), float(p["y"])) for p in d["centerline_m"]]
    if len(pts) < 2:
        raise ValueError("centerline_m has %d point(s)" % len(pts))
    gap = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
    if log_frame == "odom":
        s = d["robot_start_pose"]["position_m"]
        pts = [(x - float(s["x"]), y - float(s["y"])) for x, y in pts]
    return Centreline(pts, gap < closed_gap), gap


def read_log(path):
    """[(x, y, t)] of the usable poses, and how many lines were skipped."""
    rows, skipped = [], 0
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            try:
                v = [float(s) for s in line.split()]
            except ValueError:
                skipped += 1
                continue
            if len(v) < 3 or not all(math.isfinite(a) for a in v[:3]):
                skipped += 1
                continue
            rows.append((v[0], v[1], v[2]))
    return rows, skipped


def score(line, rows, max_jump, max_offset, start_tol):
    """Run the rule over the poses. Returns a dict of everything it measured."""
    x0, y0, _ = rows[0]
    d0, s0 = line.project(x0, y0)
    # Where the first pose sits relative to the start of the course (arc 0).
    from_start = min(s0, line.total - s0) if line.closed else s0
    start_ok = d0 <= max_offset and from_start <= start_tol

    u_start = s0          # tracker position, unwrapped arc
    u_cur = s0
    u_max = s0
    u_min = s0
    off_course = jumps = used = 0
    for x, y, _t in rows[1:]:
        centre = u_cur % line.total if line.closed else u_cur
        d, s = line.project(x, y, centre, max_jump)
        if d > max_offset:
            off_course += 1
            continue
        step = line.wrap(s - centre)
        if abs(step) > max_jump:
            jumps += 1
            continue
        u_cur += step
        used += 1
        u_max = max(u_max, u_cur)
        u_min = min(u_min, u_cur)
    arc = u_max - u_start
    return {
        "arc_m": arc,
        "fraction": arc / line.total if line.total > 0.0 else float("nan"),
        "laps": arc / line.total if line.closed and line.total > 0.0 else None,
        "index": line.index_at(u_max),
        "furthest_arc_m": u_max if not line.closed else u_max % line.total,
        "net_m": u_cur - u_start,
        "lowest_m": u_min - u_start,
        "off_course": off_course,
        "jumps": jumps,
        "used": used,
        "samples": len(rows),
        "start_arc_m": from_start,
        "start_offset_m": d0,
        "start_ok": start_ok,
    }


def median(values):
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def main():
    ap = argparse.ArgumentParser(
        description="Arc length advanced along the course centreline, from a "
                    "pose log (see the module docstring for the rule).")
    ap.add_argument("logs", nargs="*", default=[DEFAULT_LOG],
                    help="pose log(s) (default %s)" % DEFAULT_LOG)
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--log-frame", choices=("world", "odom"), default="world")
    ap.add_argument("--closed-gap", type=float, default=1.0,
                    help="first and last centreline points closer than this "
                         "make it a loop (metres, default 1.0)")
    ap.add_argument("--max-jump", type=float, default=3.0,
                    help="largest arc step between samples that is believed "
                         "(metres, default 3.0)")
    ap.add_argument("--max-offset", type=float, default=3.5,
                    help="farther than this from the centreline is off-course "
                         "(metres, default 3.5)")
    ap.add_argument("--start-tol", type=float, default=3.0,
                    help="the first pose must be this close to the course "
                         "start, along the line (metres, default 3.0)")
    ap.add_argument("--min-fraction-of", type=float, default=None,
                    metavar="REF_METRES",
                    help="gate: arc advanced must reach --fraction of this")
    ap.add_argument("--fraction", type=float, default=0.8)
    ap.add_argument("--arc-only", action="store_true",
                    help="print only the arc advanced (median over the logs)")
    args = ap.parse_args()

    try:
        line, gap = load_centreline(args.json, args.log_frame, args.closed_gap)
    except (OSError, ValueError, KeyError) as exc:
        print("course_progress: cannot read the centreline from %s: %s"
              % (args.json, exc), file=sys.stderr)
        return 2

    results = []
    for path in args.logs:
        try:
            rows, skipped = read_log(path)
        except OSError as exc:
            print("course_progress: cannot read %s: %s" % (path, exc),
                  file=sys.stderr)
            return 2
        if len(rows) < 2:
            print("course_progress: %s has %d usable pose(s), need at least 2"
                  % (path, len(rows)), file=sys.stderr)
            return 2
        r = score(line, rows, args.max_jump, args.max_offset, args.start_tol)
        r["path"], r["skipped"] = path, skipped
        results.append(r)

    arcs = [r["arc_m"] for r in results]
    if args.arc_only:
        print("%.3f" % median(arcs))
        return 0

    for r in results:
        print("course_progress: %s" % r["path"])
        print("  centreline            : %d points, %s, length %.2f m"
              % (line.n, "closed loop (gap %.3f m)" % gap if line.closed
                 else "open path (gap %.2f m)" % gap, line.total))
        print("  poses                 : %d usable, %d line(s) skipped"
              % (r["samples"], r["skipped"]))
        print("  start anchor          : first pose %.2f m from the course "
              "start along the line, %.2f m off it (%s)" % (
                  r["start_arc_m"], r["start_offset_m"],
                  "ok" if r["start_ok"] else "NOT AT THE START OF THE COURSE"))
        print("  rule                  : off-course beyond %.1f m, jumps beyond "
              "%.1f m of arc, neither credited" % (args.max_offset, args.max_jump))
        print("  ARC ADVANCED          : %.2f m in the course direction "
              "(%.3f of the centreline%s)" % (
                  r["arc_m"], r["fraction"],
                  ", %.2f lap(s)" % r["laps"] if r["laps"] is not None else ""))
        print("  furthest index        : %d of %d (%.1f m from the course "
              "start)" % (r["index"], line.n, r["furthest_arc_m"]))
        print("  net along the course  : %+.2f m (negative = ended behind where "
              "it began); lowest %+.2f m" % (r["net_m"], r["lowest_m"]))
        print("  samples not credited  : %d off-course, %d jump(s), of %d"
              % (r["off_course"], r["jumps"], r["samples"]))
        rejected = r["off_course"] + r["jumps"]
        if rejected > 0.05 * r["samples"]:
            print("  WARNING: more than 5%% of the samples were not credited; "
                  "the robot left the course, or the log is sparse (try "
                  "--max-jump) or in the other frame (--log-frame)")
        print("COURSE_PROGRESS arc_m=%.3f fraction=%.4f index=%d samples=%d "
              "rejected=%d start_ok=%d" % (
                  r["arc_m"], r["fraction"], r["index"], r["samples"],
                  rejected, 1 if r["start_ok"] else 0))
        print()

    if len(results) > 1:
        print("median arc advanced over %d logs: %.2f m" % (len(results), median(arcs)))

    if args.min_fraction_of is None:
        return 0
    need = args.fraction * args.min_fraction_of
    passed = 0
    for r in results:
        ok = r["arc_m"] >= need and r["start_ok"]
        passed += ok
        why = ""
        if not r["start_ok"]:
            why = " (the run did not start at the course start)"
        print("  PROGRESS GATE : %s  %.2f m advanced against %.2f m required "
              "(%.2f x %.2f m)%s  [%s]" % (
                  "PASS" if ok else "FAIL", r["arc_m"], need, args.fraction,
                  args.min_fraction_of, why, r["path"]))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
