"""Which robot pose a YOLOPv2 frame is written into the lane map at.

`lane_segmentation_node` writes each camera frame's lane points into an
odom-frame persistent map at the robot's pose at the frame's CAPTURE stamp (a
TF lookup at that stamp), so inference latency does not shift the lanes. When
that lookup fails, `_persistent_pose` falls back to the newest odometry: the
lanes seen at capture time are then written at the pose of now, the
image/pose mismatch the 2026 team reported. Nothing counted how often.

`capture_pose_fallback` ('latest', the default and the old behaviour, or
'skip') chooses what a failed lookup does, and the node now counts hits and
misses either way.
"""

from __future__ import annotations

import math

import pytest

from builtin_interfaces.msg import Time

from igvc_lane_detection.lane_segmentation import LaneSegmentationNode

CAPTURE = Time(sec=20, nanosec=0)


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _V:
    pass


def _tf(x, y, yaw):
    t = _V()
    t.transform = _V()
    t.transform.translation = _V()
    t.transform.rotation = _V()
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.translation.z = 0.0
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return t


class _FakeTf:
    """odom -> base_link only at the stamps given (sec); anything else raises."""

    def __init__(self, poses):
        self.poses = poses

    def lookup_transform(self, target, source, time, timeout=None):
        sec = int(time.nanoseconds // 1000000000)
        if (target, source) != ('odom', 'base_link') or sec not in self.poses:
            raise RuntimeError('no transform')
        return _tf(*self.poses[sec])


def _odom(x, y, yaw):
    o = _V()
    o.header = _V()
    o.header.frame_id = 'odom'
    o.pose = _V()
    o.pose.pose = _V()
    o.pose.pose.position = _V()
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, 0.0
    q = o.pose.pose.orientation = _V()
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return o


# The robot at capture (20 s): (1, 0, 0). Now, after inference: (3, 1, 0.5).
AT_CAPTURE = (1.0, 0.0, 0.0)
NOW = (3.0, 1.0, 0.5)


def _node(tf_poses, **attrs):
    node = object.__new__(LaneSegmentationNode)
    node.persist_frame = 'odom'
    node.base_frame = 'base_link'
    node.persist_pose_source = 'odom'
    node.odom_topic = '/odom'
    node._latest_odom = _odom(*NOW)
    node.tf_buffer = _FakeTf(tf_poses)
    node.get_logger = lambda: _Logger()
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


def _xy_yaw(pose):
    return (round(pose[0], 6), round(pose[1], 6), round(pose[3], 6))


# controls: today's behaviour, unchanged

def test_capture_stamp_available_gives_the_capture_pose():
    pose = _node({20: AT_CAPTURE})._persistent_pose(CAPTURE)
    assert _xy_yaw(pose) == (1.0, 0.0, 0.0)


def test_default_falls_back_to_the_newest_odometry_when_capture_lookup_fails():
    """The mismatch, pinned: a failed lookup writes at the pose of now."""
    pose = _node({})._persistent_pose(CAPTURE)
    assert _xy_yaw(pose) == (3.0, 1.0, 0.5)


def test_latest_named_explicitly_is_the_default():
    pose = _node({}, capture_pose_fallback='latest')._persistent_pose(CAPTURE)
    assert _xy_yaw(pose) == (3.0, 1.0, 0.5)


# the fix

def test_skip_drops_the_write_when_capture_lookup_fails():
    assert _node({}, capture_pose_fallback='skip')._persistent_pose(CAPTURE) is None


def test_skip_still_uses_the_capture_pose_when_it_exists():
    pose = _node({20: AT_CAPTURE}, capture_pose_fallback='skip')._persistent_pose(CAPTURE)
    assert _xy_yaw(pose) == (1.0, 0.0, 0.0)


@pytest.mark.parametrize('mode', ['latest', 'skip'])
def test_hits_and_misses_are_counted(mode):
    node = _node({20: AT_CAPTURE}, capture_pose_fallback=mode)
    node._persistent_pose(CAPTURE)
    node._persistent_pose(CAPTURE)
    node._persistent_pose(Time(sec=21, nanosec=0))     # no transform at 21 s
    assert (node._capture_pose_hits, node._capture_pose_misses) == (2, 1)


def test_a_lookup_without_a_stamp_is_not_counted():
    """stamp=None is the 'latest' path callers ask for on purpose."""
    node = _node({20: AT_CAPTURE})
    node._capture_pose_hits = node._capture_pose_misses = 0
    node._persistent_pose(None)
    assert (node._capture_pose_hits, node._capture_pose_misses) == (0, 0)
