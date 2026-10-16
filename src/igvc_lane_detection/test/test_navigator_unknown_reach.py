"""Unknown lane-grid space must not read as open road when the navigator is told so.

The navigator treats unknown (-1) cells as traversable at a small cost
(`traversable = roi < 90`). With ground-truth lanes that is harmless: the
corridor is dense, the paint is lethal and everything outside is unknown but
cut off. With a camera detector it is not. A blind camera (an all-unknown
grid) reads as open road and the robot drives straight on; a robot whose centre
is past the paint plans on through the unknown space beyond it, for ever
(measured in the simulator: every "-22 m" and "-34 m" run, and the
`lane_source:=hough` blind control).

`unknown_reach_m` (default -1.0, the behaviour above, unchanged) limits how far
unknown space may be planned through: only within that distance of a
known-free cell, reached without crossing a lethal cell. 0.0 means never.

Grids are in the navigator's own convention (`info.width` = forward cells,
`data[row=lateral][col=forward]`, base_link, origin (0, -width/2)); the older
costmap_fixtures.py is transposed and is not used. Tests that set
`_unknown_reach_m` on the shipped navigator, which ignores it, are the ones that
fail there.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest

from nav_msgs.msg import OccupancyGrid

from igvc_lane_detection import navigator as navigator_module
from igvc_lane_detection.navigator import IGVCNavigatorNode

RES = 0.05
FWD_M = 8.0
LAT_M = 3.0
REACH = 1.5     # what the simulator uses in the camera modes


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Canvas:
    """A lane grid, robot at the origin of the forward axis, +lat to the left."""

    def __init__(self):
        self.w = int(round(FWD_M / RES))
        self.h = int(round(2 * LAT_M / RES))
        self.data = np.full((self.h, self.w), -1, dtype=np.int8)

    def col(self, fwd):
        return int(round(fwd / RES))

    def row(self, lat):
        return int(round((lat + LAT_M) / RES))

    def rows(self, lat0, lat1):
        a, b = sorted((self.row(lat0), self.row(lat1)))
        return slice(a, b + 1)

    def free_box(self, f0, f1, l0, l1):
        self.data[self.rows(l0, l1), self.col(f0):self.col(f1)] = 0
        return self

    def lethal_box(self, f0, f1, l0, l1):
        self.data[self.rows(l0, l1), self.col(f0):self.col(f1)] = 100
        return self

    def corridor(self, half_width, f0=0.0, f1=FWD_M):
        """Ground-truth shape: free between two one-cell lethal lines, unknown outside."""
        self.free_box(f0, f1, -half_width + RES, half_width - RES)
        self.lethal_box(f0, f1, half_width - RES, half_width)
        self.lethal_box(f0, f1, -half_width, -half_width + RES)
        return self

    def grid(self):
        g = OccupancyGrid()
        g.header.frame_id = 'base_link'
        g.info.resolution = RES
        g.info.width = self.w
        g.info.height = self.h
        g.info.origin.position.x = 0.0
        g.info.origin.position.y = -LAT_M
        g.info.origin.orientation.w = 1.0
        g.data = self.data.reshape(-1).tolist()
        return g


def _navigator(grid, reach=None, **overrides):
    node = object.__new__(IGVCNavigatorNode)
    node._grid = grid
    node._obstacle_grid = None
    node._robot_xy = None
    node._robot_yaw = None
    node._lookahead = 4.0
    node._grid_res = RES
    node._min_follow_path_poses = 5
    node._min_follow_path_length_m = 1.5
    node._path_sample_spacing_m = 0.10
    node._path_smooth_window = 5
    node._max_path_lateral_jump_m = 0.60
    node._max_path_heading_rad = 1.57
    node._prev_path_bias_weight = 0.0
    node._prev_path_bias_radius_m = 0.30
    node._prev_centerline_odom = None
    node._base_frame = 'base_link'
    node._last_lane_path_reason = ''
    node.get_logger = lambda: _Logger()
    if reach is not None:
        node._unknown_reach_m = reach
    for key, value in overrides.items():
        setattr(node, key, value)
    return node


def _raw(canvas, reach=None, **overrides):
    return _navigator(canvas.grid(), reach, **overrides)._extract_centerline()


def _cells(canvas, raw):
    """Grid value under every raw path point."""
    return [int(canvas.data[canvas.row(lat), canvas.col(fwd)]) for fwd, lat in raw]


# The failures: the shipped navigator ignores the reach and plans on regardless.

def test_a_blind_grid_gives_no_path_when_unknown_is_limited():
    """No lane evidence at all: nothing to follow, so no path (the caller holds)."""
    assert _raw(_Canvas(), REACH) == []


def test_a_blind_grid_gives_no_path_even_at_a_long_reach():
    """The limit is measured from free cells, so no free cell means no reach."""
    assert _raw(_Canvas(), 50.0) == []


def test_a_robot_past_the_paint_does_not_plan_on_through_unknown():
    """The robot is 1.5 m past a lethal line; the lane is on its right, unknown ahead."""
    canvas = _Canvas()
    canvas.free_box(0.0, FWD_M, -LAT_M, -1.55)       # the lane, to the right
    canvas.lethal_box(0.0, FWD_M, -1.55, -1.50)      # the paint
    # the robot's own side is unknown
    assert _raw(canvas, REACH) == []


def test_a_small_free_pocket_does_not_extend_through_unknown():
    """A 0.5 m x 0.5 m patch of known road, unknown all round: with a reach of 0.3 m
    nothing farther than 1 m from the robot is traversable, so no path of 1.5 m."""
    canvas = _Canvas().free_box(0.0, 0.5, -0.25, 0.25)
    assert _raw(canvas, 0.3) == []


def test_unknown_behind_a_diagonal_lethal_line_is_not_reachable():
    """A one-cell lethal line at 45 degrees across a small pocket, unknown behind it.

    The search's own diagonal step passes between two diagonally touching lethal
    cells, so without the limit the path leaks through into the unknown corridor
    behind and carries on; the 4-connected reach does not, however far it
    reaches."""
    canvas = _Canvas()
    canvas.free_box(0.0, FWD_M, -0.5, 0.5)
    canvas.lethal_box(0.0, FWD_M, 0.5, 0.55)
    canvas.lethal_box(0.0, FWD_M, -0.55, -0.5)
    # the diagonal: forward 0.3 at +0.5 to forward 1.3 at -0.5, one cell per column
    r0, c0 = canvas.row(0.5), canvas.col(0.3)
    for k in range(int(round(1.0 / RES)) + 1):
        canvas.data[r0 - k, c0 + k] = 100
    beyond = np.add.outer(np.arange(canvas.h), np.arange(canvas.w)) > r0 + c0
    canvas.data[beyond & (canvas.data == 0)] = -1
    raw = _raw(canvas, 6.0)
    assert not raw or all(v == 0 for v in _cells(canvas, raw))


def test_reach_zero_plans_on_known_free_cells_only():
    """0.0 means unknown is never traversable: every path cell is free."""
    canvas = _Canvas().corridor(1.5)
    raw = _raw(canvas, 0.0)
    assert len(raw) >= 5
    assert all(v == 0 for v in _cells(canvas, raw))


def test_the_declared_default_leaves_unknown_open():
    """unknown_reach_m exists and its default is negative (off)."""
    source = Path(navigator_module.__file__).read_text(encoding='utf-8')
    match = re.search(r"\('unknown_reach_m',\s*(-?[0-9.]+)\)", source)
    assert match is not None, 'unknown_reach_m is not declared'
    assert float(match.group(1)) < 0.0


# The controls: the default, ground truth, and sparse camera grids are unchanged.

def test_an_all_unknown_grid_is_still_a_straight_path_by_default():
    """Default (reach absent, or negative): the original behaviour, both ways."""
    for node in (_navigator(_Canvas().grid()), _navigator(_Canvas().grid(), -1.0)):
        raw = node._extract_centerline()
        assert len(raw) >= 5
        assert all(abs(lat) < 0.5 for _, lat in raw)


def _ground_truth(canvas):
    canvas.corridor(1.5)
    # a barrel, inflated, off the centreline
    canvas.lethal_box(2.4, 3.0, -0.3, 0.5)
    return canvas


@pytest.mark.parametrize('reach', [None, -1.0, REACH, 0.5, 4.0])
def test_a_ground_truth_corridor_gives_the_same_path_at_any_reach(reach):
    """Ground-truth default is cell for cell and path for path unchanged."""
    expected = _raw(_ground_truth(_Canvas()))
    assert expected, 'control grid must have a path'
    assert _raw(_ground_truth(_Canvas()), reach) == expected


@pytest.mark.parametrize('reach', [None, REACH])
def test_a_hough_like_band_gives_the_same_path(reach):
    """Both fitted lines lethal, free between, unknown outside: no change."""
    canvas = _Canvas().corridor(0.8)
    expected = _raw(_Canvas().corridor(0.8))
    assert _raw(canvas, reach) == expected


def _sparse_free_grid(gap_cells):
    """YOLOPv2-like: free dashes with unknown gaps of `gap_cells` cells between them."""
    canvas = _Canvas()
    canvas.lethal_box(0.0, FWD_M, 1.0, 1.25)
    canvas.lethal_box(0.0, FWD_M, -1.25, -1.0)
    for start in range(canvas.col(0.1), canvas.col(FWD_M), 2 * gap_cells):
        canvas.data[canvas.rows(-0.95, 0.95), start:start + gap_cells] = 0
    return canvas


@pytest.mark.parametrize('gap_cells', [4, 10, 16])
def test_a_sparse_camera_grid_still_gives_a_long_path(gap_cells):
    """Gaps between free dashes are bridged: the path is as long as the open-road one."""
    canvas = _sparse_free_grid(gap_cells)
    default_raw = _raw(canvas)
    limited_raw = _raw(canvas, REACH)
    assert default_raw and limited_raw
    assert max(f for f, _ in limited_raw) >= max(f for f, _ in default_raw) - 0.2
    assert max(f for f, _ in limited_raw) >= 2.0


def test_a_free_start_with_the_lane_ahead_gives_a_path_at_the_reach():
    """The camera's near blind zone (0.3 to 1.3 m unknown, free beyond) is bridged."""
    canvas = _Canvas()
    canvas.lethal_box(0.0, FWD_M, 1.0, 1.05)
    canvas.lethal_box(0.0, FWD_M, -1.05, -1.0)
    canvas.free_box(1.3, FWD_M, -0.95, 0.95)
    raw = _raw(canvas, REACH)
    assert raw and max(f for f, _ in raw) >= 3.0


# The helper itself.

def test_limit_unknown_reach_counts_steps_and_stops_at_lethal_cells():
    roi = np.full((7, 9), -1, dtype=np.int8)
    roi[3, 0] = 0
    roi[:, 4] = 100                      # a lethal column, complete
    out = IGVCNavigatorNode._limit_unknown_reach(roi, 2.0)
    assert out[3, 0] and out[3, 1] and out[3, 2]
    assert not out[3, 3]                 # three steps away
    assert out[2, 1] and not out[0, 0]   # two steps up and over; three steps up
    assert not out[:, 4].any()           # lethal is never traversable
    assert not out[:, 5:].any()          # and nothing is reached across it


def test_limit_unknown_reach_does_not_step_between_diagonal_lethal_cells():
    roi = np.full((6, 6), -1, dtype=np.int8)
    roi[2, 2] = roi[3, 3] = 100          # a diagonal pair
    roi[3, 2] = 0                        # free below the pair...
    out = IGVCNavigatorNode._limit_unknown_reach(roi, 4.0)
    assert not out[2, 3]                 # ...is not stepped to the far side diagonally
    assert out[3, 2] and out[4, 2] and out[3, 1]
