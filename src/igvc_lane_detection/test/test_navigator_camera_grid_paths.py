"""Camera-shaped lane grids must still give a valid path through the navigator.

The heading fixes (test_navigator_path_conditioning.py) change which paths the
validator accepts, so they are checked against the grids the camera detectors
publish, not only the ground-truth one:

* Hough (`lane_detection_node`): free (0) only between two fitted straight
  lines, the lines lethal (100), unknown (-1) everywhere else, 0.05 m cells. The
  lines are straight even where the lane bends, so the band can run at a steep
  angle to the robot.
* YOLOPv2 (`lane_segmentation_node`): sparse free cells with unknown gaps
  between them, lane lines dilated to a few cells, 0.05 m cells.

Each test runs the whole chain, grid to centerline to conditioned path to
`_path_invalid_reason`, at the simulator's heading limit (1.57) and at the
shipped default (0.75). Grids are in the navigator's convention (`info.width`
forward cells, `data[row=lateral][col=forward]`, origin (0, -width/2)); the
older costmap_fixtures.py is transposed and is not used.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid

from igvc_lane_detection.navigator import IGVCNavigatorNode

FWD_M = 8.0
LAT_M = 3.0


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def _navigator(grid, limit):
    node = object.__new__(IGVCNavigatorNode)
    node._grid = grid
    node._obstacle_grid = None
    node._robot_xy = None
    node._robot_yaw = None
    node._lookahead = 4.0
    node._grid_res = 0.05
    node._min_follow_path_poses = 5
    node._min_follow_path_length_m = 1.5
    node._path_sample_spacing_m = 0.10
    node._path_smooth_window = 5
    node._max_path_lateral_jump_m = 0.60
    node._max_path_heading_rad = limit
    node._prev_path_bias_weight = 0.0
    node._prev_path_bias_radius_m = 0.30
    node._prev_centerline_odom = None
    node._base_frame = 'base_link'
    node._last_lane_path_reason = ''
    node.get_logger = lambda: _Logger()
    return node


class _Canvas:
    def __init__(self, res=0.05):
        self.res = res
        self.w = int(round(FWD_M / res))
        self.h = int(round(2 * LAT_M / res))
        self.data = np.full((self.h, self.w), -1, dtype=np.int8)
        cols = (np.arange(self.w) + 0.5) * res
        rows = -LAT_M + (np.arange(self.h) + 0.5) * res
        self.fwd, self.lat = np.meshgrid(cols, rows)

    def band(self, angle_deg, width_m, centre_lat=0.0, start=0.2):
        """A straight band at an angle to the robot, lines lethal, inside free."""
        a = math.radians(angle_deg)
        along = self.fwd * math.cos(a) + (self.lat - centre_lat) * math.sin(a)
        across = np.abs(-self.fwd * math.sin(a) + (self.lat - centre_lat) * math.cos(a))
        ahead = along > start
        self.data[ahead & (across < width_m / 2.0)] = 0
        line = ahead & (across >= width_m / 2.0) & (across < width_m / 2.0 + self.res)
        self.data[line] = 100
        return self

    def grid(self):
        g = OccupancyGrid()
        g.header.frame_id = 'base_link'
        g.info.resolution = self.res
        g.info.width = self.w
        g.info.height = self.h
        g.info.origin.position.x = 0.0
        g.info.origin.position.y = -LAT_M
        g.info.origin.orientation.w = 1.0
        g.data = self.data.reshape(-1).tolist()
        return g


def _chain(canvas, limit):
    """Grid to validator reason, as `_lane_path_from_costmap` runs it."""
    node = _navigator(canvas.grid(), limit)
    raw = node._extract_centerline()
    assert len(raw) >= node._min_follow_path_poses, 'no centerline found'
    points = node._condition_path_points(raw)
    path = node._build_path(points, Time(sec=1))
    return node._path_invalid_reason(path), points


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_hough_like_straight_band_gives_a_valid_path(limit):
    """Control: a 1.6 m band straight ahead."""
    reason, points = _chain(_Canvas().band(0.0, 1.6), limit)
    assert reason is None
    assert max(f for f, _ in points) >= 3.0


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_hough_like_band_narrower_than_the_chassis_gives_a_valid_path(limit):
    """Control: a 0.7 m band, narrower than the 0.81 m chassis, still has a path."""
    reason, points = _chain(_Canvas().band(0.0, 0.7), limit)
    assert reason is None
    assert max(f for f, _ in points) >= 3.0


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_hough_like_band_at_a_steep_angle_gives_a_valid_path(limit):
    """Straight fitted lines through a bend: the band leaves at 55 degrees.

    The limiter saturates on this path, so at the shipped 0.75 rad the validator
    used to refuse it on a 2e-15 rad excess."""
    reason, points = _chain(_Canvas().band(55.0, 1.6), limit)
    assert reason is None
    assert max(f for f, _ in points) >= 2.0


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_hough_like_band_with_the_robot_beside_a_line_gives_a_valid_path(limit):
    """Robot 0.3 m from one line of a 1.6 m band."""
    reason, points = _chain(_Canvas().band(0.0, 1.6, centre_lat=0.5), limit)
    assert reason is None
    assert max(f for f, _ in points) >= 3.0


def _yolopv2_like():
    """Half the free cells dropped to unknown, lines 0.25 m thick."""
    canvas = _Canvas()
    inside = np.abs(canvas.lat) < 0.8
    ahead = canvas.fwd > 0.2
    canvas.data[inside & ahead] = 0
    thin = np.zeros_like(canvas.data, dtype=bool)
    thin[:, 1::2] = True                      # every other forward column unknown
    canvas.data[(canvas.data == 0) & thin] = -1
    line = ahead & (np.abs(canvas.lat) >= 0.8) & (np.abs(canvas.lat) < 1.05)
    canvas.data[line] = 100
    return canvas


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_yolopv2_like_sparse_grid_gives_a_valid_path(limit):
    """Control: scattered free cells between thick lines."""
    reason, points = _chain(_yolopv2_like(), limit)
    assert reason is None
    assert max(f for f, _ in points) >= 3.0


@pytest.mark.parametrize('limit', [1.57, 0.75])
def test_all_unknown_grid_gives_a_valid_straight_path(limit):
    """Control: no lane evidence at all is read as open road (R1 fact 3)."""
    reason, points = _chain(_Canvas(), limit)
    assert reason is None
    assert all(abs(lat) < 0.5 for _, lat in points)
