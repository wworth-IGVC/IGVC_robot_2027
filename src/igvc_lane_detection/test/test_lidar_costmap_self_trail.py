"""The lidar's returns off the robot's own body leave a trail of phantom barrels.

Measured 2026-10-20 (docs/GAZEBO_SETUP.md 10.20): with the 2027 robot, every
/scan has 4 rays at 161 to 164 degrees (behind the lidar, on the robot) that
return 0.10 to 0.14 m. min_range_m 0.15 drops most of them, but the sensor's
1 cm noise pushes some just past it, and with the simulator's hit_weight 5 and
hit_threshold 4 one hit is lethal. The cell lands about 0.2 m ahead of
base_link, and because the map is persistent in odom it stays there as the
robot moves on: a trail of lethal cells along the whole driven path, which lap 2
drives back over.

self_filter_enabled drops any return whose endpoint, in base_link, lies inside
the robot's footprint box plus a margin. It is off by default, so the real
robot's map is unchanged unless it is configured. The box here is the 2027
chassis collision box (urdf/robot_2027/body_2027.urdf.xacro: 0.97 x 0.81 m,
centred on base_link to within 7 mm), and the lidar mount is x 0.345 m
(igvc_2027.urdf.xacro, rplidar_c1 prefix top).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from igvc_lane_detection.lidar_obstacle_costmap import (  # noqa: E402
    LidarObstacleCostmapNode,
)

LIDAR_X = 0.345
BODY_DEGREES = (161, 162, 163, 164)
# Just past min_range_m 0.15, where the 1 cm noise puts the tail of the body returns.
NOISY_BODY_RANGES = (0.151, 0.153, 0.155, 0.158, 0.16)

FILTER_ON = ['-p', 'self_filter_enabled:=true',
             '-p', 'self_filter_length_m:=0.97',
             '-p', 'self_filter_width_m:=0.81',
             '-p', 'self_filter_offset_x_m:=0.0',
             '-p', 'self_filter_offset_y_m:=0.0',
             '-p', 'self_filter_margin_m:=0.05']


def _scan(body_range, extra=None):
    """A 360-ray scan, 1 degree apart, from -180: body returns plus `extra` {deg: m}."""
    scan = LaserScan()
    scan.header.frame_id = 'top_rplidar_c1_link'
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(1.0)
    scan.range_min = 0.05
    scan.range_max = 12.0
    ranges = [math.inf] * 360
    if body_range is not None:
        for deg in BODY_DEGREES:
            ranges[deg + 180] = body_range
    for deg, rng in (extra or {}).items():
        ranges[deg + 180] = rng
    scan.ranges = ranges
    return scan


def _lethal_cells(extra_args, poses, body_range, extra=None, scans_per_pose=3):
    """Run the node over `poses` [(x, y, yaw)] and return lethal cell centres (N x 2)."""
    rclpy.init(args=['--ros-args',
                     '-p', 'min_range_m:=0.15',
                     '-p', f'lidar_x_m:={LIDAR_X}',
                     '-p', 'hit_weight:=5.0',
                     '-p', 'hit_threshold:=4.0',
                     '-p', 'inflate_radius_m:=0.0',
                     '-p', 'use_odom_pose:=true',
                     '-p', 'max_odom_age_sec:=0.0'] + list(extra_args))
    try:
        node = LidarObstacleCostmapNode()
        scan = _scan(body_range, extra)
        for x, y, yaw in poses:
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            node._on_odom(odom)
            for _ in range(scans_per_pose):
                node._on_scan(scan)
        cells = np.argwhere(node._hits >= node._hit_thresh)
        res, ox, oy = node._res, node._origin_x, node._origin_y
        xy = np.column_stack((ox + (cells[:, 1] + 0.5) * res,
                              oy + (cells[:, 0] + 0.5) * res))
        node.destroy_node()
        return xy
    finally:
        rclpy.shutdown()


def _near(xy, x, y, r):
    if len(xy) == 0:
        return 0
    return int(np.count_nonzero(np.hypot(xy[:, 0] - x, xy[:, 1] - y) < r))


# ── the control: today's behaviour, unchanged with the option off ──────────

def test_self_filter_defaults_off():
    rclpy.init(args=['--ros-args', '-p', 'use_odom_pose:=true'])
    try:
        node = LidarObstacleCostmapNode()
        assert node.get_parameter('self_filter_enabled').value is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('body_range', NOISY_BODY_RANGES)
def test_without_the_filter_a_noisy_body_return_is_lethal(body_range):
    """The defect, kept visible: with the option off nothing changes."""
    xy = _lethal_cells([], [(0.0, 0.0, 0.0)], body_range)
    assert _near(xy, 0.0, 0.0, 0.5) > 0


def test_without_the_filter_driving_leaves_a_trail():
    poses = [(0.1 * i, 0.0, 0.0) for i in range(51)]
    xy = _lethal_cells([], poses, 0.155, scans_per_pose=1)
    assert len(xy) >= 20     # one lethal cell per 0.1 m of path, give or take


# ── the fix ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('body_range', NOISY_BODY_RANGES)
def test_self_filter_keeps_a_noisy_body_return_out_of_the_map(body_range):
    xy = _lethal_cells(FILTER_ON, [(0.0, 0.0, 0.0)], body_range)
    assert len(xy) == 0


def test_self_filter_leaves_no_trail_along_the_driven_path():
    poses = [(0.1 * i, 0.0, 0.0) for i in range(51)]
    xy = _lethal_cells(FILTER_ON, poses, 0.155, scans_per_pose=1)
    assert len(xy) == 0


def test_self_filter_uses_the_robot_pose_not_the_odom_axes():
    """Robot at (2, 1) facing +y: the body return is still dropped."""
    xy = _lethal_cells(FILTER_ON, [(2.0, 1.0, math.pi / 2.0)], 0.158)
    assert len(xy) == 0


def test_self_filter_still_marks_a_barrel_ahead():
    """A return 1.5 m straight ahead of the lidar is a barrel and stays lethal."""
    xy = _lethal_cells(FILTER_ON, [(0.0, 0.0, 0.0)], 0.155, extra={0: 1.5})
    assert _near(xy, LIDAR_X + 1.5, 0.0, 0.15) > 0
    assert _near(xy, 0.0, 0.0, 0.5) == 0


def test_self_filter_still_marks_a_barrel_ahead_when_yawed():
    xy = _lethal_cells(FILTER_ON, [(2.0, 1.0, math.pi / 2.0)], 0.155,
                       extra={0: 1.5})
    assert _near(xy, 2.0, 1.0 + LIDAR_X + 1.5, 0.15) > 0


def test_self_filter_still_marks_a_barrel_just_outside_the_box():
    """base_link y 0.75 m is outside the box (half-width 0.405 m plus 0.05 m margin)."""
    # From the lidar at (0.345, 0), straight left (+90 deg) 0.75 m: base_link y 0.75.
    xy = _lethal_cells(FILTER_ON, [(0.0, 0.0, 0.0)], None, extra={90: 0.75})
    assert _near(xy, LIDAR_X, 0.75, 0.15) > 0
