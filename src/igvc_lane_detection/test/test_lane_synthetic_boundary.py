"""The grid the Hough node publishes when it sees only one lane line.

`lane_detection_node` invents the missing boundary one `assumed_lane_width_m`
(3.0 m) from the line it saw (`_fill_missing`) and `_build_grid` paints it
lethal. On a wide lane the navigator then centres itself between the seen line
and a wall nobody saw, which pulls the robot toward the seen line. In the P3b
simulator runs the course turns right within 3 m of the start, only the left
paint is in view, and the robot drifted left into it in all six runs.

`synthetic_boundary: unknown` leaves the invented boundary unpainted. The
default, `lethal`, is what the node has always done.

Grids are in the navigator's convention (`info.width` forward cells,
`data[row=lateral][col=forward]`, base_link, origin (0, -width/2)); the older
costmap_fixtures.py is transposed and is not used.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

from builtin_interfaces.msg import Time

from igvc_lane_detection.lane_detection import LaneDetectionNode
from igvc_lane_detection.navigator import IGVCNavigatorNode

STAMP = Time(sec=1)
RES = 0.05
# The left paint as the P3b camera saw it at sim 8.0 s, robot at the start:
# a straight fitted line in view from 2.1 to 3.5 m ahead (map_compare.py).
START_SCENE_LEFT = [(2.1 + 1.4 * i / 19.0, 2.05 - 1.10 * i / 19.0) for i in range(20)]


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
    node.get_logger = lambda: _Logger()
    for name, value in attrs.items():
        setattr(node, name, value)
    return node


def _grid_from(node, left, right):
    """What `_on_images` does after fusion: fill the missing side, build the grid."""
    left, right = node._fill_missing(left, right)
    side = getattr(node, '_last_synthetic_side', None)
    kwargs = {} if side is None else {'synthetic_side': side}
    return node._build_grid(left, right, STAMP, **kwargs)


def _array(grid):
    return np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)


def _row(grid, lat):
    return int((lat - grid.info.origin.position.y) / grid.info.resolution)


def _col(fwd):
    return int(fwd / RES)


def _line(f0, l0, f1, l1, n=20):
    return [(f0 + (f1 - f0) * i / (n - 1), l0 + (l1 - l0) * i / (n - 1)) for i in range(n)]


# the grid


def _lethal_rows(grid, fwd):
    a = _array(grid)
    return np.nonzero(a[:, _col(fwd)] == 100)[0].tolist()


def test_default_paints_the_invented_boundary_lethal():
    """Control: today's grid. The seen line and the invented one 3 m away, both 100."""
    grid = _grid_from(_detector(), list(START_SCENE_LEFT), None)
    rows = _lethal_rows(grid, 1.0)
    assert len(rows) == 2
    assert abs((rows[1] - rows[0]) * RES - 3.0) < 0.1
    a = _array(grid)
    assert a[(rows[0] + rows[1]) // 2, _col(1.0)] == 0


def test_unknown_leaves_the_invented_boundary_unpainted():
    invented = _lethal_rows(_grid_from(_detector(), list(START_SCENE_LEFT), None), 1.0)[0]
    grid = _grid_from(_detector(synthetic_boundary='unknown'), list(START_SCENE_LEFT), None)
    a = _array(grid)
    col = _col(1.0)
    rows = _lethal_rows(grid, 1.0)
    assert len(rows) == 1, 'only the line that was seen stays lethal'
    assert abs(rows[0] - (invented + round(3.0 / RES))) <= 1
    assert a[invented, col] != 100, 'the line nobody saw must not be a wall'
    assert a[invented, col] == -1
    assert a[invented + 5, col] == 0, 'the free band is kept'


def test_unknown_paints_exactly_one_lethal_cell_per_column_with_one_line_seen():
    grid = _grid_from(_detector(synthetic_boundary='unknown'), list(START_SCENE_LEFT), None)
    lethal_per_column = (_array(grid) == 100).sum(axis=0)
    assert set(lethal_per_column.tolist()) == {1}


def test_unknown_mirrors_when_only_the_right_line_is_seen():
    right = [(f, -lat) for f, lat in START_SCENE_LEFT]        # right line at -2.05 .. -0.95
    default = _grid_from(_detector(), None, list(right))
    invented = _lethal_rows(default, 1.0)[1]                  # the upper row is the invented left
    grid = _grid_from(_detector(synthetic_boundary='unknown'), None, list(right))
    a = _array(grid)
    col = _col(1.0)
    rows = _lethal_rows(grid, 1.0)
    assert len(rows) == 1
    assert abs(rows[0] - (invented - round(3.0 / RES))) <= 1
    assert a[invented, col] == -1
    assert a[invented - 5, col] == 0


def test_unknown_changes_nothing_when_both_lines_are_seen():
    """Control: two real lines, so there is no invented boundary to leave out."""
    left = _line(0.6, 1.5, 6.0, 1.5)
    right = _line(0.6, -1.5, 6.0, -1.5)
    default = _grid_from(_detector(), list(left), list(right))
    unknown = _grid_from(_detector(synthetic_boundary='unknown'), list(left), list(right))
    assert list(default.data) == list(unknown.data)
    assert (_array(default) == 100).sum(axis=0).max() == 2


def test_the_two_modes_differ_only_in_the_invented_boundary_cells():
    """Control: free cells and the seen line are the same in both modes."""
    default = _array(_grid_from(_detector(), list(START_SCENE_LEFT), None))
    unknown = _array(_grid_from(_detector(synthetic_boundary='unknown'),
                                list(START_SCENE_LEFT), None))
    differ = default != unknown
    assert set(default[differ].tolist()) <= {100}
    assert set(unknown[differ].tolist()) <= {-1}
    assert ((default == 0) == (unknown == 0)).all()


def test_synthetic_side_is_recorded_by_fill_missing():
    node = _detector()
    node._fill_missing(list(START_SCENE_LEFT), None)
    assert getattr(node, '_last_synthetic_side', 'missing') == 'right'
    node._fill_missing(None, [(f, -l) for f, l in START_SCENE_LEFT])
    assert getattr(node, '_last_synthetic_side', 'missing') == 'left'
    node._fill_missing(_line(0.6, 1.5, 6.0, 1.5), _line(0.6, -1.5, 6.0, -1.5))
    assert getattr(node, '_last_synthetic_side', 'missing') is None


def test_a_collapsed_pair_records_the_replaced_side():
    """Two lines closer than min_valid_sep: the weaker one is replaced."""
    node = _detector()
    left = _line(0.6, 0.30, 6.0, 0.30, n=20)                  # 20 points
    right = _line(0.6, -0.10, 6.0, -0.10, n=10)               # 10 points
    node._fill_missing(left, right)
    assert getattr(node, '_last_synthetic_side', 'missing') == 'right'


def test_blind_camera_grid_is_all_unknown():
    """Control (N1): no lines at all is an all-unknown grid in both modes."""
    for node in (_detector(), _detector(synthetic_boundary='unknown')):
        grid = node._empty_grid(STAMP)
        assert set(grid.data) == {-1}
        assert grid.info.width == 400 and grid.info.height == 400
        # and a failed build gives the same grid back
        assert list(node._build_grid(None, None, STAMP).data) == list(grid.data)


def _shipped_build_grid(node, left_pts, right_pts):
    """The shipped `_build_grid`, transcribed, as the reference for the default."""
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


@pytest.mark.parametrize('seed', [1, 2, 3])
def test_default_grid_is_cell_for_cell_the_shipped_grid(seed):
    """Control: with the defaults the new code paints what the old code painted."""
    rng = random.Random(seed)
    node = _detector()
    f0 = rng.uniform(0.5, 2.5)
    left = _line(f0, rng.uniform(0.8, 2.5), f0 + rng.uniform(0.8, 3.0), rng.uniform(-0.5, 2.5))
    right = [(f, l - rng.uniform(2.5, 4.0)) for f, l in left]
    grid = node._build_grid(left, right, STAMP)
    assert list(grid.data) == _shipped_build_grid(node, left, right)


# the path the real navigator plans on it


def _navigator(grid):
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
    return node


def _lateral_at(points, fwd):
    return min(points, key=lambda p: abs(p[0] - fwd))[1]


def test_default_start_scene_path_drifts_toward_the_seen_line():
    """Control, and the bug: today the centerline goes LEFT at the P3b start.

    The invented wall at -0.95 m makes the free band [-0.95, +2.05] and the
    planner steers for its middle, +0.28 m, while the course turns right."""
    grid = _grid_from(_detector(), list(START_SCENE_LEFT), None)
    points = _navigator(grid)._extract_centerline()
    assert len(points) >= 5
    assert _lateral_at(points, 3.0) > 0.0


def test_unknown_start_scene_path_turns_right_with_the_course():
    """With the invented wall gone the centerline bears right, as the true course does.

    Measured offline: +0.28 m at 3 m ahead with the wall, -1.57 m without it
    (the ground-truth grid gives -1.91 m)."""
    grid = _grid_from(_detector(synthetic_boundary='unknown'), list(START_SCENE_LEFT), None)
    points = _navigator(grid)._extract_centerline()
    assert len(points) >= 5
    assert _lateral_at(points, 3.0) < -0.5
    assert math.hypot(*points[-1]) > 3.0
