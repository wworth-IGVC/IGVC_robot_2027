"""Compensate the camera lane grid's latency (P5g).

Measured 2026-10-16 (P5f laps 1 to 4, camera lane mode): /lane_costmap reaches
the navigator a median 0.74 s after its image stamp (95th percentile 1.94 s).
The grid is in base_link at that stamp, but the navigator sends the path it
extracts as base_link at the newest odom stamp, so the path is drawn from where
the robot was. At 0.9 m/s and up to 45 deg/s that is 0.7 m and up to 33 deg of
error, and every lap weaves across the lane with a growing swing (heading
+-45 to +-75 deg against the course, period about 10 s) until the robot
crosses the paint or turns round.

`compensate_grid_latency` (default False, today's behaviour) keeps a short odom
history, extracts the centerline with the odom pose at the grid stamp (so the
obstacle overlay and the previous-path bias line up with the paint) and moves
the extracted points into the newest base_link before conditioning.

These tests drive the helpers and `_lane_path_from_costmap` with the grid
search stubbed: no grid, no Nav2, no TF. Tests marked "fails on the shipped
navigator" use what only the fix has.
"""

from __future__ import annotations

from collections import deque
import math
import re
from pathlib import Path as FilePath
from types import SimpleNamespace

from igvc_lane_detection import navigator as navigator_module
from igvc_lane_detection.navigator import IGVCNavigatorNode


class _Logger:
    def __init__(self):
        self.warnings = []

    def info(self, *args, **kwargs):
        pass

    def warn(self, msg, *args, **kwargs):
        self.warnings.append(msg)

    def error(self, *args, **kwargs):
        pass


def _stamp(t):
    sec = int(math.floor(t))
    return SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))


STRAIGHT = [(0.4 + 0.1 * i, 0.0) for i in range(30)]   # 0.4 to 3.3 m ahead


def _navigator(on=None, history=(), robot=(0.0, 0.0, 0.0), grid_t=10.0):
    """A navigator whose grid search returns STRAIGHT in the grid's base_link."""
    node = object.__new__(IGVCNavigatorNode)
    logger = _Logger()
    node.get_logger = lambda: logger
    node.logger = logger
    node._grid = SimpleNamespace(header=SimpleNamespace(stamp=_stamp(grid_t)))
    node._odom_stamp = _stamp(history[-1][0] if history else grid_t)
    node._max_costmap_age = 5.0
    node._max_odom_age = 0.5
    node._max_odom_costmap_skew = 5.0
    node._stamp_age_sec = lambda stamp: 0.0
    node._stamp_delta_sec = lambda lhs, rhs: 0.0
    node._robot_xy = (robot[0], robot[1])
    node._robot_yaw = robot[2]
    node._min_follow_path_poses = 5
    node._allow_straight_fallback = False
    node._last_lane_path_reason = ''
    node.extract_args = []

    def _extract(*args):
        node.extract_args.append(args)
        return list(STRAIGHT)

    node._extract_centerline = _extract
    node._condition_path_points = lambda pts: pts
    node._build_path = lambda pts, stamp=None: list(pts)
    node._odom_history = deque(history)
    if on is not None:
        node._compensate_grid_latency = on
    return node


# The robot drove 0.7 m straight on and turned 30 deg left since the grid stamp.
TURNED = ((9.9, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.0), (10.7, 0.7, 0.0, math.radians(30.0)))


# The failures: the shipped navigator has no compensation.

def test_compensate_grid_latency_is_declared_and_off_by_default():
    """Fails on the shipped navigator: the parameter is not declared."""
    source = FilePath(navigator_module.__file__).read_text(encoding='utf-8')
    match = re.search(r"\('compensate_grid_latency',\s*(True|False)\)", source)
    assert match is not None, 'compensate_grid_latency is not declared'
    assert match.group(1) == 'False'


def test_forward_motion_shortens_the_path_from_behind():
    """Fails on the shipped navigator: no _grid_points_to_current."""
    moved = IGVCNavigatorNode._grid_points_to_current(
        STRAIGHT, (0.0, 0.0, 0.0), (0.7, 0.0, 0.0))
    # 0.4 and 0.5 m ahead of the old pose are now behind or under the robot.
    assert all(fwd >= 0.05 for fwd, _ in moved)
    assert abs(moved[0][0] - 0.1) < 1e-6
    assert abs(moved[-1][0] - (3.3 - 0.7)) < 1e-6
    assert all(abs(lat) < 1e-9 for _, lat in moved)


def test_a_left_turn_since_the_grid_bends_the_path_right():
    """Fails on the shipped navigator: the lane did not turn, the robot did."""
    moved = IGVCNavigatorNode._grid_points_to_current(
        STRAIGHT, (0.0, 0.0, 0.0), (0.0, 0.0, math.radians(30.0)))
    end_fwd, end_lat = moved[-1]
    assert end_lat < 0.0
    assert abs(math.degrees(math.atan2(end_lat, end_fwd)) + 30.0) < 1e-6


def test_odom_pose_is_interpolated_at_the_grid_stamp():
    """Fails on the shipped navigator: no odom history."""
    node = _navigator(on=True, history=TURNED)
    x, y, yaw = node._odom_pose_at(10.35)
    assert abs(x - 0.35) < 1e-9 and abs(y) < 1e-9
    assert abs(math.degrees(yaw) - 15.0) < 1e-6
    assert node._odom_pose_at(5.0) is None      # older than the history


def test_on_the_path_is_moved_into_the_current_base_link():
    """Fails on the shipped navigator: the grid's path is sent as if current."""
    node = _navigator(on=True, history=TURNED, robot=(0.7, 0.0, math.radians(30.0)))
    pts = node._lane_path_from_costmap()
    assert node.extract_args == [((0.0, 0.0, 0.0),)]   # the pose at the grid stamp
    assert pts[0][0] >= 0.05
    assert pts[-1][1] < -1.0       # 30 deg right of the robot's new heading


def test_odom_callback_keeps_a_history_only_when_on():
    """Fails on the shipped navigator: _on_odom keeps no history."""
    node = _navigator(on=True)
    node._odom_history = deque()
    msg = SimpleNamespace(header=SimpleNamespace(stamp=_stamp(12.5)), pose=SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_odom(msg)
    assert list(node._odom_history) == [(12.5, 1.0, 2.0, 0.0)]


# The controls: today's behaviour, which must not change.

def test_off_sends_the_grid_path_unchanged():
    node = _navigator(on=False, history=TURNED, robot=(0.7, 0.0, math.radians(30.0)))
    assert node._lane_path_from_costmap() == STRAIGHT
    assert node.extract_args == [()]


def test_unset_is_off():
    node = _navigator(on=None, history=TURNED, robot=(0.7, 0.0, math.radians(30.0)))
    assert node._lane_path_from_costmap() == STRAIGHT
    assert node.extract_args == [()]


def test_on_without_odom_at_the_grid_stamp_falls_back_to_today():
    node = _navigator(on=True, history=((11.0, 0.0, 0.0, 0.0), (11.5, 0.5, 0.0, 0.0)),
                      grid_t=10.0)
    pts = node._lane_path_from_costmap()
    if hasattr(node, '_odom_pose_at'):
        assert node.logger.warnings, 'the fallback should be logged'
    assert pts == STRAIGHT
    assert node.extract_args == [()]


def test_odom_callback_keeps_no_history_when_off():
    node = _navigator(on=False)
    node._odom_history = deque()
    msg = SimpleNamespace(header=SimpleNamespace(stamp=_stamp(12.5)), pose=SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_odom(msg)
    assert list(node._odom_history) == []
    assert node._robot_xy == (1.0, 2.0)
