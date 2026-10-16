"""What the Hough node publishes while it holds a grid and the robot moves.

When a camera frame finds no lane line, `keep_last_grid_on_miss` (true in the
simulator and on the robot) republishes the last grid with the new frame's
stamp, and a 1 s timer does the same.  The grid is in base_link, so it turns
and moves with the robot, and the navigator's age and skew checks see a fresh
grid.  In the whole_line lap of 2026-10-16 (P5c, grab_wl) the detector found no
line from about sim 18 s to 20.9 s; the grid held from 17.89 s asked for a path
ending 25 degrees left of the robot's heading on every cycle while the robot
turned from 26 degrees right of the course to 43 degrees left of it, and the
path end moved from 0.18 m left of the centreline to 3.90 m (paint at 2.88 m).

`held_grid_motion: odom` moves the held grid with the robot: the last grid
built is resampled from the base_link of its frame into the base_link of the
new stamp, through odom (TF at both stamps).  The default, `none`, is what the
node has always done, cell for cell.

The scene is the shipped node's fused points from the frame grabbed at sim
17.89 s (grab_wl t018): one right line, nine points.  The robot's motion from
that frame to 18.90 s and 19.90 s is from the lap's odometry (before any
barrel contact).

Grids are in the navigator's convention (`info.width` forward cells,
`data[row=lateral][col=forward]`, base_link, origin (0, -width/2)).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from builtin_interfaces.msg import Time

from igvc_lane_detection.lane_detection import LaneDetectionNode
from igvc_lane_detection.navigator import IGVCNavigatorNode

RES = 0.05
T0 = Time(sec=17, nanosec=890000000)
T1 = Time(sec=18, nanosec=900000000)
T2 = Time(sec=19, nanosec=900000000)

# grab_wl t018 (rgb 17.89 s): the right paint, seen 1.86 to 3.64 m ahead.
T018_RIGHT = [(1.858, -1.542), (1.979, -1.529), (2.101, -1.493), (2.242, -1.451),
              (2.41, -1.402), (2.649, -1.366), (2.904, -1.295), (3.225, -1.205),
              (3.639, -1.09)]
# The robot's pose in odom: at T0 the origin; at T1 and T2 from the lap's
# odometry, expressed in the T0 base_link.
POSES = {
    (17, 890000000): (0.0, 0.0, 0.0),
    (18, 900000000): (0.894, 0.152, math.radians(19.28)),
    (19, 900000000): (1.680, 0.583, math.radians(38.31)),
}


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _V:
    pass


class _FakeTf:
    """odom -> base_link at the stamps in `poses`; anything else raises."""

    def __init__(self, poses):
        self.poses = poses
        self.calls = []

    def lookup_transform(self, target, source, time, timeout=None):
        self.calls.append((target, source, time))
        ns = int(time.nanoseconds)
        key = (ns // 1000000000, ns % 1000000000)
        if target != 'odom' or source != 'base_link' or key not in self.poses:
            raise RuntimeError('no transform')
        x, y, yaw = self.poses[key]
        tf = _V()
        tf.transform = _V()
        tf.transform.translation = _V()
        tf.transform.rotation = _V()
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.translation.z = 0.0
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return tf


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _detector(**attrs):
    node = object.__new__(LaneDetectionNode)
    node.base_frame = 'base_link'
    node.occupancy_grid_frame = 'base_link'
    node.odom_frame = 'odom'
    node.assumed_lane_width = 3.0
    node.grid_res = RES
    node.grid_width_m = 20.0
    node.grid_height_m = 20.0
    node.min_projected_line_points = 3
    node.synthetic_boundary = 'unknown'          # as the simulator runs it (P5a)
    node.lane_boundary_model = 'whole_line'      # as the simulator runs it (P5b)
    node.get_logger = lambda: _Logger()
    node.tf_buffer = _FakeTf(POSES)
    node.grid_pub = _Pub()
    for name, value in attrs.items():
        setattr(node, name, value)
    return node


def _grid(node):
    left, right = node._fill_missing([], list(T018_RIGHT))
    return node._build_grid(left, right, T0,
                            synthetic_side=getattr(node, '_last_synthetic_side', None))


def _array(grid):
    return np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)


def _held(node, stamp):
    """What the node publishes on a miss at `stamp` after building the t018 grid."""
    grid = _grid(node)
    node.latest_grid = grid
    if getattr(node, 'held_grid_motion', 'none') == 'odom':
        node._held_src = (grid, T0)
        return node._moved_held_grid(stamp)
    grid.header.stamp = stamp
    return grid


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


def _end_bearing_deg(grid):
    points = _navigator(grid)._extract_centerline()
    assert len(points) >= 5
    f, lat = points[-1]
    return math.degrees(math.atan2(lat, f))


def _right_line_lat(pose, fwd):
    """The t018 right line (its least-squares fit) in the base_link of `pose`, at fwd."""
    f = np.array([p[0] for p in T018_RIGHT])
    lat = np.array([p[1] for p in T018_RIGHT])
    b, a = np.polyfit(f, lat, 1)
    x, y, yaw = pose
    # points of the line in T0 base_link, into the base_link of pose
    s = np.linspace(-2.0, 8.0, 2001)
    lx, ly = s, a + b * s
    c, si = math.cos(yaw), math.sin(yaw)
    fw = c * (lx - x) + si * (ly - y)
    la = -si * (lx - x) + c * (ly - y)
    k = int(np.argmin(np.abs(fw - fwd)))
    return float(la[k])


# ── the fix ─────────────────────────────────────────────────────────────────

def test_held_grid_motions_are_declared():
    assert LaneDetectionNode._HELD_GRID_MOTIONS == ('none', 'odom')


def test_odom_with_no_motion_is_the_held_grid_cell_for_cell():
    node = _detector(held_grid_motion='odom')
    moved = _held(node, T0)
    assert _array(moved).tolist() == _array(_grid(_detector())).tolist()


def test_odom_publishes_with_the_new_stamp():
    node = _detector(held_grid_motion='odom')
    moved = _held(node, T1)
    assert (moved.header.stamp.sec, moved.header.stamp.nanosec) == (18, 900000000)


@pytest.mark.parametrize('stamp', [T1, T2])
def test_odom_keeps_the_seen_line_where_the_paint_is(stamp):
    """The lethal right line, seen from the moved robot, is where the line is."""
    node = _detector(held_grid_motion='odom')
    a = _array(_held(node, stamp))
    key = (stamp.sec, stamp.nanosec)
    for fwd in (1.5, 2.0):
        want = _right_line_lat(POSES[key], fwd)
        col = int(fwd / RES)
        rows = np.nonzero(a[:, col] == 100)[0]
        assert len(rows) > 0
        lats = rows * RES - 10.0 + RES / 2.0
        assert np.min(np.abs(lats - want)) < 0.15


@pytest.mark.parametrize('stamp', [T1, T2])
def test_odom_path_turns_back_right_once_the_robot_has_turned_left(stamp):
    """Held unchanged the grid asks for 25 degrees left on every cycle; moved
    with the robot it asks for a right turn once the robot has turned 19 or
    38 degrees left."""
    node = _detector(held_grid_motion='odom')
    assert _end_bearing_deg(_held(node, stamp)) < -5.0


def test_odom_without_a_transform_publishes_an_all_unknown_grid():
    node = _detector(held_grid_motion='odom', tf_buffer=_FakeTf({}))
    moved = _held(node, T1)
    assert set(moved.data) == {-1}


def test_odom_does_not_fall_back_to_the_latest_transform():
    node = _detector(held_grid_motion='odom')
    _held(node, T1)
    stamps = {int(c[2].nanoseconds) for c in node.tf_buffer.calls}
    assert stamps == {17890000000, 18900000000}


def test_odom_timer_republishes_the_moved_grid():
    node = _detector(held_grid_motion='odom')
    grid = _grid(node)
    node.latest_grid = grid
    node._held_src = (grid, T0)

    class _Clock:
        def now(self):
            class _N:
                def to_msg(self_inner):
                    return T2
            return _N()

    node.get_clock = lambda: _Clock()
    node._republish_grid()
    sent = node.grid_pub.sent[-1]
    assert _array(sent).tolist() == _array(node._moved_held_grid(T2)).tolist()
    assert _array(sent).tolist() != _array(grid).tolist()


# ── the bug, pinned (passes before and after) ───────────────────────────────

@pytest.mark.parametrize('stamp', [T1, T2])
def test_default_holds_the_grid_unchanged_so_the_path_keeps_turning_left(stamp):
    node = _detector()
    held = _held(node, stamp)
    assert _array(held).tolist() == _array(_grid(_detector())).tolist()
    assert _end_bearing_deg(held) > 15.0


# ── the default is unchanged ─────────────────────────────────────────────────

def test_none_named_explicitly_is_the_default():
    a = _array(_held(_detector(), T1)).tolist()
    b = _array(_held(_detector(held_grid_motion='none'), T1)).tolist()
    assert a == b


def test_default_timer_restamps_the_same_grid():
    node = _detector()
    grid = _grid(node)
    node.latest_grid = grid
    before = list(grid.data)

    class _Clock:
        def now(self):
            class _N:
                def to_msg(self_inner):
                    return T2
            return _N()

    node.get_clock = lambda: _Clock()
    node._republish_grid()
    sent = node.grid_pub.sent[-1]
    assert sent is grid
    assert list(sent.data) == before
    assert (sent.header.stamp.sec, sent.header.stamp.nanosec) == (19, 900000000)
    assert node.tf_buffer.calls == []


def test_a_grid_in_a_fixed_frame_is_not_moved():
    node = _detector(held_grid_motion='odom', occupancy_grid_frame='odom')
    grid = _grid(_detector())
    node.latest_grid = grid
    node._held_src = (grid, T0)

    class _Clock:
        def now(self):
            class _N:
                def to_msg(self_inner):
                    return T2
            return _N()

    node.get_clock = lambda: _Clock()
    node._republish_grid()
    assert node.grid_pub.sent[-1] is grid
    assert node.tf_buffer.calls == []


# ── a TF lag must not blank the grid (2026-10-16 fix) ───────────────────────
# Live, four laps: "no odom->base_link transform" 57 to 59 times per lap. TF
# lags the newest image and always lags now(), so an exact lookup at the new
# stamp failed and the node published an all-unknown grid. Now it moves the
# grid to the newest transform TF holds and stamps it with that time.

T3 = Time(sec=20, nanosec=500000000)          # newer than any transform


class _NsTime:
    def __init__(self, ns):
        self.nanoseconds = ns


class _FakeTfWithLatest(_FakeTf):
    """As _FakeTf; time 0 (rclpy's 'latest') gives the newest pose, header stamped."""

    def lookup_transform(self, target, source, time, timeout=None):
        if int(time.nanoseconds) == 0 and self.poses:
            key = max(self.poses)
            tf = super().lookup_transform(
                target, source, _NsTime(key[0] * 1000000000 + key[1]), timeout)
            tf.header = _V()
            tf.header.stamp = Time(sec=key[0], nanosec=key[1])
            return tf
        return super().lookup_transform(target, source, time, timeout)


def _clock_at(stamp):
    class _Clock:
        def now(self):
            class _N:
                def to_msg(self_inner):
                    return stamp
            return _N()
    return lambda: _Clock()


def _held_node():
    node = _detector(held_grid_motion='odom', tf_buffer=_FakeTfWithLatest(POSES))
    grid = _grid(node)
    node.latest_grid = grid
    node._held_src = (grid, T0)
    return node


def test_odom_new_stamp_ahead_of_tf_moves_to_the_newest_transform():
    node = _held_node()
    out = node._moved_held_grid(T3)
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (T2.sec, T2.nanosec)
    assert (_array(out) != -1).any(), 'a TF lag blanked the grid'
    ref = _held_node()._moved_held_grid(T2)
    assert _array(out).tolist() == _array(ref).tolist()


def test_odom_timer_is_not_blank_when_now_is_ahead_of_tf():
    node = _held_node()
    node.get_clock = _clock_at(T3)
    node._republish_grid()
    sent = node.grid_pub.sent[-1]
    assert (_array(sent) == 100).sum() > 0
    assert (sent.header.stamp.sec, sent.header.stamp.nanosec) == (T2.sec, T2.nanosec)


def test_odom_exact_stamp_available_uses_it_not_the_latest():
    node = _held_node()
    node._moved_held_grid(T1)
    assert all(int(c[2].nanoseconds) != 0 for c in node.tf_buffer.calls)


def test_odom_no_transform_at_all_is_still_all_unknown():
    node = _detector(held_grid_motion='odom', tf_buffer=_FakeTfWithLatest({}))
    grid = _grid(node)
    node.latest_grid = grid
    node._held_src = (grid, T0)
    out = node._moved_held_grid(T3)
    assert set(_array(out).ravel().tolist()) == {-1}
