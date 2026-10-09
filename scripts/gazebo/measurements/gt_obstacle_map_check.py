#!/usr/bin/env python3
"""U1: does gt_obstacle_map_node mark the same barrel cells as gt_nav_bridge_node?

Question: with lane_source:=hough or yolopv2 the barrels reach the navigator on
/lidar_obstacle_map instead of inside /lane_costmap. If the new node's cells
differ from the bridge's, camera runs would avoid barrels differently from the
ground-truth runs they are compared with, and any difference in BARRELS or LANE
could be the overlay rather than the detector.

Method: from the real track_points.json,
  1. the bridge's own _load_obstacles parses the barrels (a stand-in object
     carries the attributes, so no node is built and nothing is published);
  2. the bridge's own _stamp_obstacles stamps them onto a blank 800 x 800 grid
     at 0.1 m with origin (-40, -40), the grid track_ground_truth_node makes;
  3. gt_obstacle_map.rasterise_barrels draws the same barrels on the same grid.
The two lethal cell sets must be IDENTICAL, cell for cell. The parsed barrel
lists must be identical too.

Controls, because a comparison of a function with itself passes whatever it
does. Each must produce a DIFFERENT cell set from the bridge's, or the test
itself fails:
  CONTROL A  the node's inflation is raised by 0.10 m
  CONTROL B  the node ignores the spawn offset (start taken as 0, 0)

Resolution: the bridge reads it from a message, where it is a float32
(0.10000000149011612), so the bridge is run with that value, as it is in the
running graph. A run with the plain 0.1 is printed for information.

Run it where rclpy and nav_msgs import (the container, ROS sourced), from the
repo root. The two modules are read from the source tree, so it needs no build:
    cd /root/ros2_ws/src/IGVC_robot_2026
    python3 scripts/gazebo/measurements/gt_obstacle_map_check.py

Exit 0 on PASS, 1 on FAIL, 2 if the modules cannot be imported.
Options: --json PATH, --inflate METRES (default: obstacle_inflate_radius_m from
config/isaac_nav_test.yaml, else 0.30).
"""
import argparse
import array
import os
import sys
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_JSON = os.path.join(REPO, "IGVC_track_generator", "track_points.json")
SHARED_CFG = os.path.join(REPO, "src", "igvc_test_bringup", "config",
                          "isaac_nav_test.yaml")

# The grid track_ground_truth_node publishes and gt_nav_bridge_node stamps.
GRID_CELLS = 800
GRID_RES = 0.10
GRID_ORIGIN = -40.0


def shared_inflate():
    """obstacle_inflate_radius_m from the YAML the launch hands the bridge."""
    try:
        import yaml
        with open(SHARED_CFG, encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        return float(data["gt_nav_bridge_node"]["ros__parameters"]
                     ["obstacle_inflate_radius_m"]), "read from %s" % SHARED_CFG
    except Exception as exc:  # noqa: BLE001 - reported, then the code default
        return 0.30, "code default, could not read %s (%s)" % (SHARED_CFG, exc)


class _Quiet:
    """Stands in for rclpy's logger inside the bridge's loader."""

    def info(self, *a, **k):
        pass

    warn = error = debug = info


def bridge_cells(bridge_cls, track_json, inflate, resolution):
    """Lethal cell mask from the bridge's own parser and stamper, plus what it parsed."""
    from nav_msgs.msg import OccupancyGrid
    me = SimpleNamespace(_track_file=track_json, _obstacles=[],
                         _origin_offset_x=0.0, _origin_offset_y=0.0,
                         _origin_yaw=0.0, _inflate_r=inflate,
                         get_logger=lambda: _Quiet())
    bridge_cls._load_obstacles(me)

    grid = OccupancyGrid()
    grid.info.resolution = resolution
    grid.info.width = GRID_CELLS
    grid.info.height = GRID_CELLS
    grid.info.origin.position.x = GRID_ORIGIN
    grid.info.origin.position.y = GRID_ORIGIN
    grid.data = array.array('b', b'\xff' * (GRID_CELLS * GRID_CELLS))
    bridge_cls._stamp_obstacles(me, grid)
    data = np.array(grid.data, dtype=np.int8).reshape((GRID_CELLS, GRID_CELLS))
    return data == 100, me._obstacles, (me._origin_offset_x, me._origin_offset_y)


def node_cells(mod, barrels, start, inflate, resolution=GRID_RES):
    data = mod.rasterise_barrels(barrels, start, GRID_CELLS, GRID_CELLS,
                                 resolution, GRID_ORIGIN, GRID_ORIGIN, inflate)
    if not set(np.unique(data).tolist()) <= {-1, 100}:
        raise AssertionError("node grid holds values other than -1 and 100")
    return data == 100


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--inflate", type=float, default=None,
                    help="barrel inflation in metres (default: the shared YAML)")
    args = ap.parse_args()

    # Source tree first, so the check tests the files in the repo whether or not
    # they have been built and installed. Both are plain modules.
    for sub in ("src/igvc_lane_detection", "src/igvc_test_bringup"):
        sys.path.insert(0, os.path.join(REPO, *sub.split("/")))
    try:
        from igvc_lane_detection.gt_nav_bridge_node import GTNavBridgeNode
        from igvc_test_bringup import gt_obstacle_map as mod
    except ImportError as exc:
        print("FAIL: cannot import the two modules (%s). Source the ROS "
              "environment first: . /opt/ros/jazzy/setup.bash" % exc)
        return 2

    if args.inflate is None:
        inflate, where = shared_inflate()
    else:
        inflate, where = args.inflate, "from --inflate"
    print("U1: gt_obstacle_map_node against gt_nav_bridge_node")
    print("  track file          : %s" % args.json)
    print("  inflation           : %.2f m (%s)" % (inflate, where))
    print("  grid                : %d x %d at %.2f m, origin (%.1f, %.1f)"
          % (GRID_CELLS, GRID_CELLS, GRID_RES, GRID_ORIGIN, GRID_ORIGIN))
    print("  bridge from         : %s" % sys.modules[GTNavBridgeNode.__module__].__file__)
    print("  node from           : %s" % mod.__file__)

    res_wire = float(np.float32(GRID_RES))
    ref, b_barrels, b_start = bridge_cells(GTNavBridgeNode, args.json, inflate, res_wire)
    ref_lit, _, _ = bridge_cells(GTNavBridgeNode, args.json, inflate, GRID_RES)
    n_barrels, n_start, problems = mod.load_barrels(args.json)

    ok = True
    print()

    same_parse = (b_barrels == n_barrels) and (b_start == n_start)
    print("  barrels parsed      : bridge %d, node %d, %s"
          % (len(b_barrels), len(n_barrels),
             "identical" if same_parse else "DIFFERENT"))
    if problems or not same_parse or len(n_barrels) == 0:
        print("  FAIL: the two parsers disagree, or there are no barrels %s" % problems)
        ok = False

    got = node_cells(mod, n_barrels, n_start, inflate)
    n_ref, n_got = int(ref.sum()), int(got.sum())
    diff = int(np.count_nonzero(ref != got))
    print("  lethal cells        : bridge %d, node %d, differing cells %d"
          % (n_ref, n_got, diff))
    lit_diff = int(np.count_nonzero(ref_lit != ref))
    print("  (information)       : bridge with the Python literal 0.1 instead of "
          "the float32 value differs from itself in %d cell(s)" % lit_diff)
    if n_ref == 0:
        print("  FAIL: the bridge marked no cells, so the comparison is empty")
        ok = False
    if diff != 0:
        print("  FAIL: the cell sets are not identical")
        ok = False
    else:
        print("  PASS: identical, cell for cell")

    # Every barrel centre must be lethal in the node's grid, a check of the
    # arithmetic that does not use the bridge at all.
    missing = []
    for k, (x, y, _r) in enumerate(n_barrels):
        col = int((x - n_start[0] - GRID_ORIGIN) / GRID_RES)
        row = int((y - n_start[1] - GRID_ORIGIN) / GRID_RES)
        if not (0 <= row < GRID_CELLS and 0 <= col < GRID_CELLS) or not got[row, col]:
            missing.append(k)
    print("  barrel centres      : %d of %d lethal in the node's grid"
          % (len(n_barrels) - len(missing), len(n_barrels)))
    if missing:
        print("  FAIL: barrel(s) %s have a non-lethal centre cell" % missing)
        ok = False

    print()
    for label, what, cells in (
            ("CONTROL A", "inflation +0.10 m",
             node_cells(mod, n_barrels, n_start, inflate + 0.10)),
            ("CONTROL B", "spawn offset ignored",
             node_cells(mod, n_barrels, (0.0, 0.0), inflate))):
        d = int(np.count_nonzero(ref != cells))
        good = d > 0
        print("  %s           : %-22s differs from the bridge in %5d cell(s) "
              "-> %s" % (label, what, d, "ok, it differs" if good else "NO DIFFERENCE"))
        if not good:
            print("  FAIL: %s produced the same cells, so this test cannot "
                  "tell a wrong overlay from a right one" % label)
            ok = False

    print()
    print("  U1: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
