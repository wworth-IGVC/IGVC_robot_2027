"""The simulated lidar sees the robot's own body; min_range_m keeps it out of the map.

Measured 2026-10-09 (docs/GAZEBO_SETUP.md 10.20): with the robot standing still,
4 of the 360 rays return 0.11 to 0.13 m, at 161 to 164 degrees from the lidar
(behind it, on the robot). The sensor's 1 cm noise puts some of them past the
competition's min_range_m of 0.15, where the simulator's parameters (hit_weight
5, hit_threshold 4, as igvc_fused_drive.launch.py) make one hit lethal. The
cell lands about 0.2 m ahead of base_link, inside the footprint, and inflated by
the map's radius it walls the robot in at the start. The mount x is 0.345 m.
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
BODY_RANGE_M = 0.16        # the noisy tail of the 0.11 to 0.13 m body returns


def _lethal_cells_near_base(min_range_m):
    rclpy.init(args=['--ros-args',
                     '-p', f'min_range_m:={min_range_m}',
                     '-p', f'lidar_x_m:={LIDAR_X}',
                     '-p', 'hit_weight:=5.0',
                     '-p', 'hit_threshold:=4.0',
                     '-p', 'inflate_radius_m:=0.0',
                     '-p', 'use_odom_pose:=true',
                     '-p', 'max_odom_age_sec:=0.0'])
    try:
        node = LidarObstacleCostmapNode()
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.0
        node._on_odom(odom)

        scan = LaserScan()
        scan.header.frame_id = 'top_rplidar_c1_link'
        scan.angle_min = -math.pi
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.05
        scan.range_max = 12.0
        ranges = [math.inf] * 360
        for deg in (161, 162, 163, 164):
            ranges[deg + 180] = BODY_RANGE_M
        scan.ranges = ranges
        for _ in range(3):
            node._on_scan(scan)

        cells = np.argwhere(node._hits >= node._hit_thresh)
        res, ox, oy = node._res, node._origin_x, node._origin_y
        xs = ox + (cells[:, 1] + 0.5) * res
        ys = oy + (cells[:, 0] + 0.5) * res
        node.destroy_node()
        return int(np.count_nonzero((np.abs(xs) < 0.5) & (np.abs(ys) < 0.5)))
    finally:
        rclpy.shutdown()


def test_competition_min_range_lets_a_body_return_into_the_footprint():
    """The competition's 0.15 m admits the noisy tail: the defect, kept visible."""
    assert _lethal_cells_near_base(0.15) > 0


def test_min_range_025_keeps_the_body_out_of_the_map():
    """0.25 m is above every body return seen (0.134 m plus 3 sigma of noise)."""
    assert _lethal_cells_near_base(0.25) == 0
