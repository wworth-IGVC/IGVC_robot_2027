"""The grid the Hough node builds when the robot faces a painted line obliquely.

In every camera lap of 2026-10-16 the robot left the lane over the LEFT paint
just before barrel_3 (centreline index 146 to 156), facing the paint at 40 to
55 degrees. The front camera then sees that one line crossing its heading.
`_fill_missing` repartitions its points by the SIGN of their lateral offset, so
the one line becomes a 'left' piece and a 'right' piece, and `_build_grid`
holds each piece's end lateral to the 20 m grid edge (`_interp` clamps). The
free band between the two pieces runs straight ahead, through the paint.

`lane_boundary_model: whole_line` keeps the line whole, sides it by where its
straight-line fit passes the robot, continues it along its own direction and
paints it as a 4-connected lethal run. The default, `legacy`, is what the node
has always done, cell for cell.

The two scenes are the fused points the shipped node computed from the frames
grabbed live at sim 19.87 s and 20.92 s (P5b, grab_c2 t020 and t021).

Grids are in the navigator's convention (`info.width` forward cells,
`data[row=lateral][col=forward]`, base_link, origin (0, -width/2)).
"""

from __future__ import annotations

import random

import numpy as np
import pytest

from builtin_interfaces.msg import Time

from igvc_lane_detection.lane_detection import LaneDetectionNode
from igvc_lane_detection.navigator import IGVCNavigatorNode

STAMP = Time(sec=1)
RES = 0.05

# grab_c2 t021 (rgb 20.92 s): one fitted line, 20 points, from 0.77 m left at
# 1.23 m ahead to 1.90 m right at 3.64 m ahead. The real left paint crosses the
# heading 1.78 m ahead.
T021 = [(1.23, 0.766), (1.268, 0.734), (1.303, 0.691), (1.332, 0.639), (1.381, 0.597),
        (1.427, 0.542), (1.477, 0.483), (1.533, 0.421), (1.605, 0.35), (1.675, 0.268),
        (1.754, 0.181), (1.843, 0.076), (1.963, -0.044), (2.084, -0.176), (2.224, -0.338),
        (2.39, -0.529), (2.649, -0.773), (2.904, -1.062), (3.225, -1.426), (3.64, -1.897)]
# grab_c2 t020 (rgb 19.87 s): the same paint, 13 points. Real paint crosses the
# heading 2.48 m ahead.
T020 = [(1.544, 1.303), (1.618, 1.23), (1.688, 1.136), (1.754, 1.015), (1.843, 0.897),
        (1.964, 0.769), (2.084, 0.606), (2.225, 0.421), (2.391, 0.203), (2.627, -0.069),
        (2.906, -0.401), (3.226, -0.817), (3.642, -1.368)]


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def _detector(**attrs):
    node = object.__new__(LaneDetectionNode)
    node.base_frame = 'base_link'
    node.occupancy_grid_frame = 'base_link'
    node.assumed_lane_width = 3.0
    node.grid_res = RES
    node.grid_width_m = 20.0
    node.grid_height_m = 20.0
    node.min_projected_line_points = 3
    node.synthetic_boundary = 'unknown'          # as the simulator runs it (P5a)
    node.get_logger = lambda: _Logger()
    for name, value in attrs.items():
        setattr(node, name, value)
    return node


def _whole(**attrs):
    return _detector(lane_boundary_model='whole_line', **attrs)


def _grid_from(node, left, right):
    """What `_on_images` does after fusion: fill the missing side, build the grid."""
    left, right = node._fill_missing(left, right)
    if left is None or right is None:
        return None
    return node._build_grid(left, right, STAMP,
                            synthetic_side=getattr(node, '_last_synthetic_side', None))


def _array(grid):
    return np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)


def _row(lat):
    return int((lat + 10.0) / RES)


def _col(fwd):
    return int(fwd / RES)


def _line_lat(pts, fwd):
    """The fitted line through the scene's points, at fwd (the paint as the camera saw it)."""
    f = np.array([p[0] for p in pts])
    lat = np.array([p[1] for p in pts])
    b, a = np.polyfit(f, lat, 1)
    return a + b * fwd


def _line(f0, l0, f1, l1, n=20):
    return [(f0 + (f1 - f0) * i / (n - 1), l0 + (l1 - l0) * i / (n - 1)) for i in range(n)]


def _navigator(grid, unknown_reach_m=1.5):
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
    node._unknown_reach_m = unknown_reach_m
    node.get_logger = lambda: _Logger()
    return node


def _far_side_cells(grid, pts, fmax, margin=0.10):
    """(rows, cols) of cells at least `margin` beyond the seen line, on the side away from
    the robot, up to fmax ahead and within the navigator's +-3 m window."""
    a = _array(grid)
    rows, cols = [], []
    for c in range(_col(0.0), _col(fmax)):
        edge = _line_lat(pts, c * RES + RES / 2) + margin
        for r in range(max(0, _row(edge) + 1), min(a.shape[0], _row(3.0) + 1)):
            rows.append(r)
            cols.append(c)
    return np.array(rows), np.array(cols)


# the bug, pinned (pass on both)


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_default_opens_a_corridor_straight_through_a_crossing_line(scene):
    """Control, and the bug: today the cells straight ahead are free far past the paint."""
    a = _array(_grid_from(_detector(), list(scene), []))
    crossing = next(f for f, lat in scene if lat < 0.0)
    assert a[_row(0.0), _col(crossing + 1.5)] == 0
    assert a[_row(0.0), _col(6.0)] == 0


def test_default_splits_one_crossing_line_into_two_sides():
    """Control: the legacy repartition puts the near part left and the far part right."""
    node = _detector()
    left, right = node._fill_missing(list(T021), [])
    assert max(lat for _, lat in right) < 0.0 < min(lat for _, lat in left)
    assert len(left) + len(right) < len(T021), 'points within 0.08 m of the axis are dropped'
    assert node._last_synthetic_side is None


# the fix (fail on the shipped code, pass with it)


def test_boundary_models_are_declared():
    assert LaneDetectionNode._BOUNDARY_MODELS == ('legacy', 'whole_line')


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_whole_line_keeps_a_crossing_line_on_one_side(scene):
    node = _whole()
    left, right = node._fill_missing(list(scene), [])
    assert left == list(scene), 'the whole line is the left boundary'
    assert node._last_synthetic_side == 'right'
    assert right == [(f, lat - 3.0) for f, lat in scene]


def test_whole_line_sides_a_line_by_where_it_passes_the_robot():
    """Mean lateral -1.25 m, but at the robot the line is 1.67 m to the left."""
    scene = _line(1.0, 0.5, 4.0, -3.0)
    node = _whole()
    left, right = node._fill_missing(list(scene), [])
    assert left == scene
    assert node._last_synthetic_side == 'right'


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_whole_line_is_lethal_where_the_line_crosses_the_heading(scene):
    a = _array(_grid_from(_whole(), list(scene), []))
    heading = a[_row(0.0), :_col(8.0)]
    lethal = np.nonzero(heading == 100)[0]
    assert len(lethal), 'the heading meets a lethal cell'
    cross = _line_lat(scene, 0.0) / -np.polyfit([p[0] for p in scene], [p[1] for p in scene], 1)[0]
    assert abs(lethal[0] * RES - cross) < 0.15
    assert not (heading[lethal[0]:] == 0).any(), 'nothing free on the heading past the line'


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_whole_line_marks_nothing_free_beyond_the_line(scene):
    grid = _grid_from(_whole(), list(scene), [])
    rows, cols = _far_side_cells(grid, scene, 8.0)
    assert len(rows)
    assert not (_array(grid)[rows, cols] == 0).any()


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_whole_line_unknown_beyond_the_line_is_out_of_the_planners_reach(scene):
    """F12: unknown cells within 1.5 m of free are searchable, 4-connected, not across a
    lethal cell. A steep one-cell-per-column line leaks; the 4-connected run does not."""
    grid = _grid_from(_whole(), list(scene), [])
    a = _array(grid)
    r0, r1, c1 = _row(-3.0), _row(3.0) + 1, _col(4.0) + 1
    reach = IGVCNavigatorNode._limit_unknown_reach(a[r0:r1, :c1].astype(np.int8), 1.5 / RES)
    rows, cols = _far_side_cells(grid, scene, 4.0)
    assert not reach[rows - r0, cols].any()


@pytest.mark.parametrize('scene', [T020, T021], ids=['t020', 't021'])
def test_whole_line_path_stays_on_the_robots_side_of_the_line(scene):
    grid = _grid_from(_whole(), list(scene), [])
    points = _navigator(grid)._extract_centerline()
    assert len(points) >= 5
    assert all(lat < _line_lat(scene, f) for f, lat in points)
    assert points[-1][0] > 3.0


def test_whole_line_continues_along_the_line_beyond_the_points():
    node = _whole()
    grid = _grid_from(node, list(T021), [])
    lethal = np.nonzero(_array(grid)[:, _col(5.0)] == 100)[0]
    expect = _row(T021[-1][1] + np.polyfit([p[0] for p in T021], [p[1] for p in T021], 1)[0]
                  * (5.0 - T021[-1][0]))
    assert any(abs(r - expect) <= 1 for r in lethal)


# controls (pass on both)


def test_default_path_runs_through_the_line():
    """Control, and the bug: the planned path crosses the seen line (t021)."""
    grid = _grid_from(_detector(), list(T021), [])
    points = _navigator(grid)._extract_centerline()
    assert len(points) >= 5
    assert any(lat > _line_lat(T021, f) for f, lat in points)


def test_legacy_named_explicitly_is_the_default():
    for scene in (T020, T021, _line(0.6, 1.5, 6.0, 1.5)):
        a = _grid_from(_detector(), list(scene), [])
        b = _grid_from(_detector(lane_boundary_model='legacy'), list(scene), [])
        assert list(a.data) == list(b.data)


@pytest.mark.parametrize('sb', ['lethal', 'unknown'])
def test_whole_line_changes_nothing_on_a_straight_lane(sb):
    """Control: two parallel lines, and one parallel line, give the legacy grid exactly."""
    left = _line(0.6, 1.5, 6.0, 1.5)
    right = _line(0.6, -1.5, 6.0, -1.5)
    for l, r in ((left, right), (left, []), ([], right)):
        a = _grid_from(_detector(synthetic_boundary=sb), list(l), list(r))
        b = _grid_from(_whole(synthetic_boundary=sb), list(l), list(r))
        assert list(a.data) == list(b.data)


def _shipped_build_grid(node, left_pts, right_pts):
    """The shipped `_build_grid` (lethal synthetic boundary), transcribed."""
    lpts = sorted(left_pts, key=lambda p: p[0])
    rpts = sorted(right_pts, key=lambda p: p[0])
    nx = int(node.grid_height_m / node.grid_res)
    ny = int(node.grid_width_m / node.grid_res)
    data = [-1] * (nx * ny)
    for col in range(nx):
        fwd = col * node.grid_res
        ll = LaneDetectionNode._interp(lpts, fwd)
        rl = LaneDetectionNode._interp(rpts, fwd)
        left_row = max(0, min(ny - 1, int((ll + node.grid_width_m / 2) / node.grid_res)))
        right_row = max(0, min(ny - 1, int((rl + node.grid_width_m / 2) / node.grid_res)))
        lo, hi = min(left_row, right_row), max(left_row, right_row)
        for row in range(ny):
            if lo < row < hi:
                data[row * nx + col] = 0
            elif row == lo or row == hi:
                data[row * nx + col] = 100
    return data


@pytest.mark.parametrize('seed', [1, 2, 3, 4])
def test_default_grid_is_cell_for_cell_the_shipped_grid(seed):
    """Control: with the defaults the new code paints what the old code painted,
    including on crossing lines."""
    rng = random.Random(seed)
    node = _detector(synthetic_boundary='lethal')
    f0 = rng.uniform(0.5, 2.5)
    left = _line(f0, rng.uniform(0.3, 2.5), f0 + rng.uniform(0.8, 3.0), rng.uniform(-2.5, 2.5))
    right = [(f, lat - rng.uniform(2.5, 4.0)) for f, lat in left]
    grid = node._build_grid(left, right, STAMP)
    assert list(grid.data) == _shipped_build_grid(node, left, right)
