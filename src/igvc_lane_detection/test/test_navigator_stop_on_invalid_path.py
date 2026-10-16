"""Stop on an invalid lane path (P5e): cancel the held FollowPath goal.

Measured 2026-10-16 (P5d, lap grab_wlh): in camera lane mode, when the lane
path becomes invalid ("centerline extraction found no connected free band
ahead") the navigator only starts its 3 s blocked timer. It neither cancels nor
replaces the active FollowPath goal, and that goal's path is in base_link, so
it turns with the robot: the controller drove a constant-curvature circle
(-37 deg/s for 2.7 s, heading +10 to -128 deg) until the robot faced back down
the course.

`stop_on_invalid_path` (default False, today's behaviour) cancels the active
FollowPath goal on the first invalid cycle and publishes one zero command, so
the robot stops and waits; the next valid path is sent as a new goal. The
blocked timer and the stuck reverse run as before.

These tests drive `_update` with the path validator and the path source
stubbed: no grid, no Nav2, no TF. Tests marked "fails on the shipped
navigator" set the attribute that only the fix reads; on the shipped navigator
it changes nothing.
"""

from __future__ import annotations

import re
from pathlib import Path as FilePath

from nav_msgs.msg import Path

from igvc_lane_detection import navigator as navigator_module
from igvc_lane_detection.navigator import IGVCNavigatorNode

REASON = 'centerline extraction found no connected free band ahead'


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)


class _Clock:
    def __init__(self):
        self.sec = 100.0

    def now(self):
        return _Time(int(round(self.sec * 1e9)))


class _Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _GoalHandle:
    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1


def _navigator(stop=None, reverse=True, **overrides):
    node = object.__new__(IGVCNavigatorNode)
    clock = _Clock()
    node.get_clock = lambda: clock
    node.get_logger = lambda: _Logger()
    node._path_pub = _Publisher()
    node._reverse_cmd_pub = _Publisher()
    node._mission_state = 'lane_follow'
    node._paused = False
    node._goal_pending = False
    node._abort_backoff_until = None
    node._last_goal_send_time = None
    node._replan_min_dt = 0.33
    node._stuck_reverse_enabled = reverse
    node._stuck_timeout_sec = 3.0
    node._reverse_speed_mps = 0.30
    node._reverse_max_dist_m = 0.80
    node._blocked_since_sec = None
    node._reverse_active = False
    node._reverse_start_xy = None
    node._robot_xy = (0.0, 0.0)
    node._goal_handle = _GoalHandle()
    node._current_goal_seq = 1
    # The lane path: its validity is all _update reads here.
    node.invalid = REASON
    node.sent_goals = []
    node._lane_path_from_costmap = lambda: Path()
    node._path_invalid_reason = lambda path: node.invalid
    node._uses_follow_path = lambda: True
    node._path_changed_enough = lambda path: False

    def _send_path_goal(path, force=False):
        node.sent_goals.append(force)
        return True

    node._send_path_goal = _send_path_goal
    if stop is not None:
        node._stop_on_invalid_path = stop
    for key, value in overrides.items():
        setattr(node, key, value)
    return node, clock


def _commands(node):
    return [msg.linear.x for msg in node._reverse_cmd_pub.sent]


# The failures: the shipped navigator keeps the goal whatever the parameter says.

def test_stop_on_invalid_path_is_declared_and_off_by_default():
    """stop_on_invalid_path exists and its default is False (fails on the shipped navigator)."""
    source = FilePath(navigator_module.__file__).read_text(encoding='utf-8')
    match = re.search(r"\('stop_on_invalid_path',\s*(True|False)\)", source)
    assert match is not None, 'stop_on_invalid_path is not declared'
    assert match.group(1) == 'False'


def test_first_invalid_path_cancels_the_active_goal_and_stops():
    """Fails on the shipped navigator: the goal is kept and nothing is published."""
    node, _ = _navigator(stop=True)
    handle = node._goal_handle
    node._update()
    assert handle.cancels == 1
    assert node._goal_handle is None
    assert _commands(node) == [0.0]


def test_the_goal_is_cancelled_once_not_every_invalid_cycle():
    """Fails on the shipped navigator (no cancel in the first 3 s at all)."""
    node, clock = _navigator(stop=True)
    handle = node._goal_handle
    for _ in range(10):
        node._update()
        clock.sec += 0.1
    assert handle.cancels == 1
    assert _commands(node) == [0.0]


def test_stop_works_with_the_stuck_reverse_disabled():
    """Fails on the shipped navigator: with no reverse it holds the goal for ever."""
    node, _ = _navigator(stop=True, reverse=False)
    handle = node._goal_handle
    node._update()
    assert handle.cancels == 1
    assert node._goal_handle is None


def test_next_valid_path_is_sent_as_a_new_goal():
    """After the stop, a valid path is sent with force (fails on the shipped navigator).

    The shipped navigator still holds the old goal, so an unchanged path is
    not resent and the robot keeps following the old one.
    """
    node, clock = _navigator(stop=True)
    node._update()
    clock.sec += 0.5
    node.invalid = None
    node._update()
    assert node.sent_goals == [True]
    assert node._blocked_since_sec is None


def test_blocked_timer_starts_on_the_same_cycle_as_the_stop():
    """The stop does not replace the blocked timer (fails on the shipped navigator: no stop)."""
    node, clock = _navigator(stop=True)
    node._update()
    assert node._goal_handle is None
    assert node._blocked_since_sec == clock.sec
    assert node._reverse_active is False


# The controls: the default, the reverse, and the cases with no goal to cancel.

def test_default_keeps_the_goal_while_the_path_is_invalid():
    """Default (attribute absent, or False): today's behaviour, both ways."""
    for node, _ in (_navigator(), _navigator(stop=False)):
        handle = node._goal_handle
        node._update()
        assert handle.cancels == 0
        assert node._goal_handle is handle
        assert _commands(node) == []


def test_reverse_still_starts_after_the_stuck_timeout():
    """The reverse begins at stuck_timeout_sec with or without the stop."""
    for stop in (None, True):
        node, clock = _navigator(stop=stop)
        handle = node._goal_handle
        node._update()
        clock.sec += 3.1
        node._update()
        assert node._reverse_active is True
        assert node._goal_handle is None
        assert handle.cancels == 1
        assert _commands(node)[-1] == -0.30


def test_no_goal_active_publishes_nothing():
    """With no goal to cancel the stop does nothing (no zero spam while blocked)."""
    node, _ = _navigator(stop=True, _goal_handle=None)
    node._update()
    assert _commands(node) == []
    assert node._blocked_since_sec is not None


def test_a_pending_goal_is_left_alone():
    """While a goal is being accepted _update returns before the validator, as before."""
    node, _ = _navigator(stop=True, _goal_pending=True)
    handle = node._goal_handle
    node._update()
    assert handle.cancels == 0
    assert node._blocked_since_sec is None


def test_valid_path_with_a_goal_active_is_unchanged():
    """A valid, unchanged path with a goal active is not resent, with or without the stop."""
    for stop in (None, True):
        node, _ = _navigator(stop=stop)
        node.invalid = None
        handle = node._goal_handle
        node._update()
        assert handle.cancels == 0
        assert node.sent_goals == []
