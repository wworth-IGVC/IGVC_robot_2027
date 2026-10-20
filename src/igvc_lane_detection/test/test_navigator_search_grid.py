"""The navigator's search grid, published for RViz (/navigator/search_grid).

`_extract_centerline` plans over a window of /lane_costmap ahead of the robot,
after three things RViz never saw: the lidar obstacle map is stamped into it,
gap choice (F21) drops cells too narrow for the robot, and unknown_reach_m
(F12) drops unknown cells far from known-free ones. /lane_costmap alone does
not show the cells the path was actually chosen over.

`publish_search_grid` (default False) publishes that window as an
OccupancyGrid in the lane grid's frame and stamp, every planning cycle:

    100       blocked (paint, a stamped barrel, a lidar obstacle map cell)
    99        passable but not searched (gap choice or unknown reach)
    1 to 60   searched, known free: the search's clearance penalty
              (0.45 m / clearance, capped at 6) scaled to 1 to 60, so low
              mid-lane (about 5 on this course) rising to 60 at the paint
    -1        searched, unknown

These tests use test_navigator_gap_choice.py's scene: barrel_3 on the course,
a 0.88 m gap between it and the left paint.
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
import numpy as np
from test_navigator_gap_choice import (_navigator, _Scene, BARREL_3,
                                       CHASSIS_HALF_M, HALF_LANE_M, RES,
                                       STAMPED_R)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _scene():
    return _Scene(1.5, 25.0).corridor(HALF_LANE_M).barrel(*BARREL_3).back_strip()


def _with_pub(scene, **attrs):
    node = _navigator(scene, attrs.pop('gap', None))
    node._grid.header.stamp = Time(sec=7, nanosec=500)
    node._search_grid_pub = _Pub()
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


def _array(msg):
    return np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)


def _value_at(msg, fwd, lat):
    c = int((fwd - msg.info.origin.position.x) / msg.info.resolution)
    r = int((lat - msg.info.origin.position.y) / msg.info.resolution)
    return int(_array(msg)[r, c])


def _barrel_in_base_link(scene, barrel=BARREL_3):
    dx, dy = barrel[0], barrel[1] - scene.y0
    c, s = math.cos(scene.psi), math.sin(scene.psi)
    return c * dx + s * dy, -s * dx + c * dy


# the fix

def test_one_grid_per_plan_in_the_lane_grids_frame_and_stamp():
    """One grid per planning cycle, in the lane grid's frame and stamp."""
    node = _with_pub(_scene())
    node._extract_centerline()
    assert len(node._search_grid_pub.msgs) == 1
    msg = node._search_grid_pub.msgs[0]
    assert isinstance(msg, OccupancyGrid)
    assert msg.header.frame_id == 'base_link'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (7, 500)
    assert msg.info.resolution == RES
    assert msg.info.origin.orientation.w == 1.0
    assert len(msg.data) == msg.info.width * msg.info.height


def test_the_window_is_the_lookahead_ahead_of_the_robot():
    """The window runs from the robot to the 4 m lookahead, +-3 m wide."""
    node = _with_pub(_scene())
    node._extract_centerline()
    msg = node._search_grid_pub.msgs[0]
    near_x = msg.info.origin.position.x
    far_x = near_x + msg.info.width * msg.info.resolution
    assert -0.15 <= near_x <= 0.0
    assert 3.9 <= far_x <= node._lookahead + 0.2
    assert msg.info.origin.position.y <= -2.9
    assert msg.info.origin.position.y + msg.info.height * RES >= 2.9


def test_every_path_point_is_on_a_searched_cell():
    """The path RViz draws lies on cells the grid shows as searched."""
    node = _with_pub(_scene(), gap=True)
    pts = node._extract_centerline()
    msg = node._search_grid_pub.msgs[0]
    assert pts
    for fwd, lat in pts:
        assert _value_at(msg, fwd, lat) < 99, (fwd, lat)


def test_the_barrel_is_blocked_and_the_free_lane_is_low_cost():
    """Barrel cells are 100; free cells cost little mid-lane, more by paint."""
    scene = _scene()
    node = _with_pub(scene)
    node._extract_centerline()
    msg = node._search_grid_pub.msgs[0]
    bf, bl = _barrel_in_base_link(scene)
    assert _value_at(msg, bf, bl) == 100
    values = _array(msg)
    free_searched = values[(values >= 1) & (values <= 60)]
    assert free_searched.size > 0
    assert int(free_searched.min()) <= 10
    assert int(values.max()) == 100
    # Rising toward the paint: a searched free cell beside a blocked one is
    # 0.1 m from it, penalty 4.5, so 45 or more.
    free = (values >= 1) & (values <= 60)
    beside = np.zeros_like(free)
    blocked = values == 100
    beside[1:, :] |= blocked[:-1, :]
    beside[:-1, :] |= blocked[1:, :]
    beside[:, 1:] |= blocked[:, :-1]
    beside[:, :-1] |= blocked[:, 1:]
    assert (free & beside).any()
    assert int(values[free & beside].min()) >= 45


def test_gap_choice_shows_the_narrow_gap_as_not_searched():
    """Show the 0.88 m gap left of barrel_3 as not searched with gap choice.

    It is passable paint-free ground the robot does not fit through: 99 with
    gap choice on, searched without it.
    """
    scene = _scene()
    bf, bl = _barrel_in_base_link(scene)
    c, s = math.cos(scene.psi), math.sin(scene.psi)
    # the middle of the gap, between the barrel and the left paint, in world
    gap_y = (BARREL_3[1] + STAMPED_R + HALF_LANE_M) / 2.0
    dx, dy = BARREL_3[0], gap_y - scene.y0
    gf, gl = c * dx + s * dy, -s * dx + c * dy

    on = _with_pub(scene, gap=True)
    on._extract_centerline()
    assert _value_at(on._search_grid_pub.msgs[0], gf, gl) == 99

    off = _with_pub(_scene(), gap=False)
    off._extract_centerline()
    assert 1 <= _value_at(off._search_grid_pub.msgs[0], gf, gl) <= 60
    assert 2 * (CHASSIS_HALF_M + 0.15) > (HALF_LANE_M - BARREL_3[1] - STAMPED_R)


def test_unknown_beyond_the_reach_is_not_searched_and_within_it_is():
    """Unknown within unknown_reach_m is -1, beyond it 99."""
    scene = _Scene(0.0, 0.0)
    scene.data[:, :] = -1
    scene.data[:, :int(2.0 / RES)] = 0       # known free for the first 2 m
    node = _with_pub(scene, _unknown_reach_m=0.5)
    node._extract_centerline()
    msg = node._search_grid_pub.msgs[0]
    assert _value_at(msg, 2.2, 0.0) == -1    # unknown, 0.2 m from free
    assert _value_at(msg, 3.5, 0.0) == 99    # unknown, 1.5 m from free


def test_the_lidar_obstacle_map_is_drawn_into_the_grid():
    """A barrel only in the odom-frame obstacle map shows as blocked."""
    scene = _Scene(0.0, 0.0).corridor(HALF_LANE_M).back_strip()
    og = OccupancyGrid()
    og.info.resolution = RES
    og.info.width = og.info.height = 100
    og.info.origin.position.x = -2.0
    og.info.origin.position.y = -5.0
    og.info.origin.orientation.w = 1.0
    data = np.zeros((100, 100), dtype=np.int8)
    yy, xx = np.mgrid[0:100, 0:100]
    wx = -2.0 + (xx + 0.5) * RES
    wy = -5.0 + (yy + 0.5) * RES
    data[np.hypot(wx - 2.5, wy + 1.0) <= 0.3] = 100
    og.data = data.reshape(-1).tolist()
    node = _with_pub(scene)
    node._obstacle_grid = og
    node._extract_centerline()
    assert _value_at(node._search_grid_pub.msgs[0], 2.5, -1.0) == 100


def test_publishing_does_not_change_the_path():
    """The same path with the publisher as without it."""
    quiet = _navigator(_scene(), True)
    loud = _with_pub(_scene(), gap=True)
    assert quiet._extract_centerline() == loud._extract_centerline()


# controls

def test_default_publishes_nothing():
    """Without the publisher (the default) nothing is built or published."""
    node = _navigator(_scene(), True)
    assert getattr(node, '_search_grid_pub', None) is None
    assert node._extract_centerline()
