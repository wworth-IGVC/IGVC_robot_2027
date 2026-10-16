"""Gap choice (F21): which side of a barrel the lane planner takes.

Measured 2026-10-16: barrel_3 sits 1.23 m left of the course centreline. On
lap 1 the robot passes it on the wide right side. On lap 2 it often arrives
heading left and takes the 0.88 m gap between the barrel and the left paint,
which a 0.81 m chassis crosses with 1 cm to spare (2026 robot) or -3 cm (2027
robot). The search treats that gap as an ordinary corridor, only a little more
expensive, so the side it takes follows where the robot is and which way it
points, not where it fits.

`gap_choice_enabled` (default False) drops from the search the cells too close
to a blocked cell for the robot (half-width 0.405 m plus a 0.15 m margin) when
a path through the rest still reaches the end of the lookahead. These tests
build the ground-truth bridge's local grid by hand, in the navigator's
convention (`info.width` forward cells, `data[row=lateral][col=forward]`,
base_link, origin (0, -3)); costmap_fixtures.py is transposed and is not used.
The corridor runs along world +x; the robot starts at (0, y0) heading `psi`.

Tests marked "fails on the shipped navigator" set the attribute that only the
fix reads; on the shipped navigator it changes nothing.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid

from igvc_lane_detection.navigator import IGVCNavigatorNode

RES = 0.1
FWD_M = 6.0
LAT_M = 3.0
CHASSIS_HALF_M = 0.405
STAMPED_R = 0.28 + 0.30          # barrel_3 radius plus the bridge's inflation
BARREL_3 = (3.2, 1.34)           # (x ahead, y left of the centreline), as on the course
HALF_LANE_M = 2.8                # the course's inner half-width at barrel_3


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Scene:
    """The bridge's local grid for a robot at (0, y0) heading psi."""

    def __init__(self, y0=0.0, psi_deg=0.0):
        self.y0 = y0
        self.psi = math.radians(psi_deg)
        self.w = int(round(FWD_M / RES))
        self.h = int(round(2 * LAT_M / RES))
        self.data = np.full((self.h, self.w), -1, dtype=np.int8)
        fwd = (np.arange(self.w) + 0.5) * RES
        lat = -LAT_M + (np.arange(self.h) + 0.5) * RES
        f, l = np.meshgrid(fwd, lat)
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.x = f * c - l * s
        self.y = y0 + f * s + l * c

    def corridor(self, half, centre=0.0):
        """Free inside, one-cell-and-a-half lethal paint, unknown outside."""
        d = np.abs(self.y - centre)
        self.data[d < half] = 0
        self.data[(d >= half) & (d < half + 1.5 * RES)] = 100
        return self

    def curve(self, half, radius):
        """A left bend: the corridor centre follows a circle of this radius."""
        cx, cy = 0.0, radius
        d = np.abs(np.hypot(self.x - cx, self.y - cy) - radius)
        self.data[d < half] = 0
        self.data[(d >= half) & (d < half + 1.5 * RES)] = 100
        return self

    def barrel(self, x, y, r=STAMPED_R):
        self.data[np.hypot(self.x - x, self.y - y) <= r] = 100
        return self

    def back_strip(self, m=0.3):
        """The bridge's lethal strip behind the robot."""
        self.data[:, :int(math.ceil(m / RES))] = 100
        return self

    def grid(self):
        g = OccupancyGrid()
        g.header.frame_id = 'base_link'
        g.info.resolution = RES
        g.info.width = self.w
        g.info.height = self.h
        g.info.origin.position.x = 0.0
        g.info.origin.position.y = -LAT_M
        g.info.origin.orientation.w = 1.0
        g.data = self.data.reshape(-1).tolist()
        return g

    def world(self, points):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return [(f * c - l * s, self.y0 + f * s + l * c) for f, l in points]


def _navigator(scene, enabled=None):
    node = object.__new__(IGVCNavigatorNode)
    node._grid = scene.grid()
    node._obstacle_grid = None
    node._robot_xy = (0.0, scene.y0)
    node._robot_yaw = scene.psi
    node._lookahead = 4.0
    node._grid_res = 0.05
    node._min_follow_path_poses = 5
    node._min_follow_path_length_m = 1.5
    node._path_sample_spacing_m = 0.10
    node._path_smooth_window = 5
    node._max_path_lateral_jump_m = 0.60
    node._max_path_heading_rad = 1.57
    node._prev_path_bias_weight = 0.4
    node._prev_path_bias_radius_m = 0.30
    node._prev_centerline_odom = None
    node._base_frame = 'base_link'
    node._last_lane_path_reason = ''
    node.get_logger = lambda: _Logger()
    if enabled is not None:
        node._gap_choice_enabled = enabled
        node._gap_choice_half_width_m = CHASSIS_HALF_M
        node._gap_choice_margin_m = 0.15
    return node


def _plan(scene, enabled=None):
    """Raw centreline in world coordinates, and the validator's verdict."""
    node = _navigator(scene, enabled)
    raw = node._extract_centerline()
    assert len(raw) >= node._min_follow_path_poses, 'no centerline found'
    points = node._condition_path_points(raw)
    path = node._build_path(points, Time(sec=1))
    return scene.world(raw), node._path_invalid_reason(path), node


def _y_at(world, x):
    return min(world, key=lambda p: abs(p[0] - x))[1]


def _gap_to_barrel(world, barrel=BARREL_3, r=STAMPED_R):
    return min(math.hypot(px - barrel[0], py - barrel[1]) for px, py in world) - r


def _passes_left_of(world, barrel=BARREL_3):
    return _y_at(world, barrel[0]) > barrel[1]


# fails on the shipped navigator

@pytest.mark.parametrize('y0, psi', [(1.5, 25.0), (0.0, 45.0), (0.5, 45.0)])
def test_robot_left_of_the_barrel_takes_the_wide_right_side(y0, psi):
    """Fails on the shipped navigator: robot on or aimed at the left, so the
    shipped search takes the 0.88 m gap between barrel_3 and the left paint."""
    scene = _Scene(y0, psi).corridor(HALF_LANE_M).barrel(*BARREL_3).back_strip()
    world, reason, node = _plan(scene, enabled=True)
    assert reason is None
    assert not _passes_left_of(world), 'took the narrow left gap'
    assert _gap_to_barrel(world) >= CHASSIS_HALF_M
    assert getattr(node, '_last_gap_choice', 'adequate') == 'adequate'


def test_the_narrow_gap_is_narrower_than_the_robot_plus_margin():
    """Control: the scene really is the one measured. The gap between the
    stamped barrel and the paint is 0.88 m, the robot needs 2 x 0.555 m."""
    gap = (HALF_LANE_M - 0.0) - (BARREL_3[1] + STAMPED_R)
    assert 0.8 < gap < 0.95
    assert gap < 2 * (CHASSIS_HALF_M + 0.15)


def test_mirror_image_scene_takes_the_wide_left_side():
    """Fails on the shipped navigator: barrel_3 mirrored to the right of the
    centreline, robot arriving from the right, aimed at the narrow right gap."""
    scene = _Scene(-1.5, -25.0).corridor(HALF_LANE_M).barrel(
        BARREL_3[0], -BARREL_3[1]).back_strip()
    world, reason, _ = _plan(scene, enabled=True)
    assert reason is None
    assert _y_at(world, BARREL_3[0]) > -BARREL_3[1], 'took the narrow right gap'
    assert _gap_to_barrel(world, (BARREL_3[0], -BARREL_3[1])) >= CHASSIS_HALF_M


# controls: pass on both

def test_default_is_off_and_changes_nothing():
    """Control: with the parameter at its default the planner is the shipped
    one. The attribute is absent on the shipped navigator, False on the fix."""
    scene = _Scene(1.5, 25.0).corridor(HALF_LANE_M).barrel(*BARREL_3).back_strip()
    node = _navigator(scene)
    assert getattr(node, '_gap_choice_enabled', False) is False
    raw_default = node._extract_centerline()
    node_off = _navigator(scene, enabled=False)
    assert node_off._extract_centerline() == raw_default
    assert getattr(node_off, '_last_gap_choice', 'off') == 'off'


@pytest.mark.parametrize('scene_name', ['straight', 'curve'])
def test_ground_truth_scenes_without_obstacles_give_the_same_path(scene_name):
    """Control: no barrel, a straight corridor and a left bend. Enabled and
    default paths are identical, so the fix only acts where something splits
    the band."""
    scene = _Scene(0.0, 0.0)
    if scene_name == 'straight':
        scene.corridor(1.5)
    else:
        scene.curve(1.5, 8.0)
    scene.back_strip()
    on, reason_on, _ = _plan(scene, enabled=True)
    off, reason_off, _ = _plan(scene, enabled=False)
    assert reason_on is None and reason_off is None
    assert on == off
    assert max(x for x, _ in on) > 3.0


def test_barrel_splitting_the_lane_evenly_is_passed_with_room_either_way():
    """Control: barrel on the centreline of a 5.6 m lane. Both sides are wide,
    so either is fine; the path must clear the barrel and stay off the paint,
    and enabling the fix must not change which side is taken or the path."""
    scene = _Scene(0.0, 0.0).corridor(HALF_LANE_M).barrel(3.2, 0.0).back_strip()
    on, reason_on, _ = _plan(scene, enabled=True)
    off, reason_off, _ = _plan(scene, enabled=False)
    assert reason_on is None and reason_off is None
    assert _gap_to_barrel(on, (3.2, 0.0)) >= CHASSIS_HALF_M
    assert max(abs(y) for _, y in on) <= HALF_LANE_M - CHASSIS_HALF_M
    assert _passes_left_of(on, (3.2, 0.0)) == _passes_left_of(off, (3.2, 0.0))
    assert on == off


@pytest.mark.parametrize('barrel_y, expect_left', [(0.55, False), (-0.55, True)])
def test_only_one_adequate_side_is_taken(barrel_y, expect_left):
    """Control: a 3.2 m lane with the barrel 0.55 m off its centre. One gap is
    0.47 m (too narrow), the other 1.57 m, and the robot starts on the narrow
    side. The wide side is the only adequate one and must be taken."""
    y0 = 0.9 if barrel_y > 0 else -0.9
    scene = _Scene(y0, 0.0).corridor(1.6).barrel(3.2, barrel_y).back_strip()
    world, reason, _ = _plan(scene, enabled=True)
    assert reason is None
    assert _passes_left_of(world, (3.2, barrel_y)) == expect_left
    assert _gap_to_barrel(world, (3.2, barrel_y)) >= CHASSIS_HALF_M
    assert max(abs(y) for _, y in world) <= 1.6 - 0.3


def test_no_adequate_side_keeps_todays_path():
    """Control: a 2.4 m lane with the barrel in the middle leaves two 0.2 m
    gaps. Nothing is adequate, so the path is exactly the shipped one."""
    scene = _Scene(0.0, 0.0).corridor(1.2).barrel(3.2, 0.0).back_strip()
    on, reason_on, node = _plan(scene, enabled=True)
    off, reason_off, _ = _plan(scene, enabled=False)
    assert reason_on == reason_off
    assert on == off
    assert getattr(node, '_last_gap_choice', 'none') == 'none'


def test_robot_beside_the_paint_can_leave_it():
    """Control: robot 0.3 m from the left paint, no barrel. The near field is
    exempt from the width test, so the fix still finds a path; and it is the
    path the shipped planner finds."""
    scene = _Scene(HALF_LANE_M - 0.3, 0.0).corridor(HALF_LANE_M).back_strip()
    on, reason_on, _ = _plan(scene, enabled=True)
    off, reason_off, _ = _plan(scene, enabled=False)
    assert reason_on is None and reason_off is None
    assert max(x for x, _ in on) > 3.0
    assert on == off


def test_a_band_narrower_than_the_robot_stays_in_the_band():
    """Control: a camera-style grid, a 0.7 m free band between two lines with
    unknown (open road) outside them. No branch is adequate; the path must not
    slip out through the unknown cells beside the band."""
    scene = _Scene(0.0, 0.0)
    scene.data[:] = -1
    inside = (np.abs(scene.y) < 0.35) & (scene.x > 0.2)
    line = (np.abs(scene.y) >= 0.35) & (np.abs(scene.y) < 0.35 + RES) & (scene.x > 0.2)
    scene.data[inside] = 0
    scene.data[line] = 100
    on, reason_on, _ = _plan(scene, enabled=True)
    off, reason_off, _ = _plan(scene, enabled=False)
    assert reason_on == reason_off
    assert on == off
    assert all(abs(y) < 0.4 for x, y in on if x > 0.4)
