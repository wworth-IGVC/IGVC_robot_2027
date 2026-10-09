"""Unit tests for mission-planner GPS start gating and anchoring.

Until 2026-10-08 the gate compared the first three DIGITS of latitude and
longitude with the start waypoint's.  That accepted a fix 6 km from the start
and rejected one 20 cm away across a digit boundary, and the shipped
placeholder waypoints (0, 0) rejected every real fix without saying so.  The
gate is now a distance in metres, the placeholder is named loudly, and the
anchor pairs the first matching fix with the robot's odom position.
"""

from __future__ import annotations

import math

from igvc_lane_detection.mission_planner import MissionPlannerNode, _Waypoint

START_LAT, START_LON = 42.293123, -83.715456
M_PER_DEG_LAT = 111_190.0           # close enough for choosing test offsets


class _Logger:
    def __init__(self):
        self.warnings = []
        self.errors = []
        self.infos = []

    def info(self, msg, *args, **kwargs):
        self.infos.append(msg)

    def warn(self, msg, *args, **kwargs):
        self.warnings.append(msg)

    def error(self, msg, *args, **kwargs):
        self.errors.append(msg)


def _make_node(*, required_fixes: int = 1, radius_m: float = 10.0,
               start=(START_LAT, START_LON)) -> MissionPlannerNode:
    node = object.__new__(MissionPlannerNode)
    node._waypoints = [
        _Waypoint(lat=start[0], lon=start[1], kind='start'),
        _Waypoint(lat=start[0] + 0.0001, lon=start[1], kind='obstacle'),
        _Waypoint(lat=start[0] + 0.0002, lon=start[1], kind='end'),
    ]
    node._gps_start_match_enabled = True
    node._gps_start_match_radius_m = radius_m
    node._gps_start_match_required_fixes = required_fixes
    node._gps_start_match_count = 0
    node._origin_lat = None
    node._origin_lon = None
    node._origin_xy = (0.0, 0.0)
    node._robot_xy = None
    node._gps_start_area_confirmed = False
    node._published = 0
    node._publish_virtual_obstacles = lambda: setattr(
        node, '_published', node._published + 1)
    node._log = _Logger()
    node.get_logger = lambda: node._log
    # heading fit off by default here; the heading tests turn it on
    node._yaw_fit_enabled = False
    node._yaw_fit_min_travel_m = 10.0
    node._yaw_fit_max_residual_m = 2.0
    node._odom_enu_yaw = None
    node._fit_ref = None
    node._fit_samples = []
    node._fit_residual = float('nan')
    node._fit_travel = 0.0
    node._yaw_published = []
    node._yaw_pub = type('P', (), {'publish': lambda self, m: node._yaw_published.append(m)})()
    node._stamped_header = lambda: None
    return node


class _Fix:
    class status:           # noqa: N801 - mimics sensor_msgs/NavSatStatus
        status = 0

    def __init__(self, lat, lon):
        self.latitude = lat
        self.longitude = lon


def _north_of_start(metres):
    return START_LAT + metres / M_PER_DEG_LAT


def test_fix_far_from_start_is_rejected():
    node = _make_node()

    assert not node._gps_start_area_matches(_north_of_start(200.0), START_LON)
    assert node._gps_start_match_count == 0


def test_fix_6_km_away_in_the_same_digit_box_is_rejected():
    # The old gate accepted exactly this: same first three digits of both
    # coordinates, about 6.6 km from the start waypoint.
    node = _make_node()

    assert not node._gps_start_area_matches(42.299999, -83.799999)


def test_fix_within_the_radius_is_accepted():
    node = _make_node()

    assert node._gps_start_area_matches(_north_of_start(6.0), START_LON)
    assert node._gps_start_match_count == 1


def test_fix_just_over_a_digit_boundary_is_accepted():
    # 20 cm apart, but the first three digits of the latitude differ
    # (42.299999 -> 422, 42.300001 -> 423).  The old gate rejected this.
    node = _make_node(start=(42.299999, -83.715456))

    assert node._gps_start_area_matches(42.300001, -83.715456)


def test_radius_is_the_limit():
    node = _make_node(radius_m=10.0)

    assert node._gps_start_area_matches(_north_of_start(9.0), START_LON)
    node._gps_start_match_count = 0
    assert not node._gps_start_area_matches(_north_of_start(11.0), START_LON)


def test_can_require_consecutive_matches():
    node = _make_node(required_fixes=2)

    assert not node._gps_start_area_matches(_north_of_start(3.0), START_LON)
    assert node._gps_start_area_matches(_north_of_start(2.0), START_LON)
    assert node._gps_start_match_count == 2


def test_a_miss_resets_the_run_of_matches():
    node = _make_node(required_fixes=2)

    assert not node._gps_start_area_matches(_north_of_start(3.0), START_LON)
    assert not node._gps_start_area_matches(_north_of_start(300.0), START_LON)
    assert node._gps_start_match_count == 0


def test_placeholder_start_waypoint_never_matches_and_says_so():
    node = _make_node(start=(0.0, 0.0))

    assert node._start_is_placeholder()
    assert not node._gps_start_area_matches(START_LAT, START_LON)
    assert any('placeholder' in w for w in node._log.warnings)


def test_disabled_gate_accepts_anything():
    node = _make_node()
    node._gps_start_match_enabled = False

    assert node._gps_start_area_matches(0.0, 0.0)


def test_anchor_pairs_the_fix_with_the_odom_position():
    # The robot has driven to (30, -4) in odom when it reaches the start
    # waypoint.  A waypoint 10 m north of the start must land 10 m north of
    # THAT, not 10 m north of the odom origin.
    node = _make_node()
    node._robot_xy = (30.0, -4.0)
    node._gps_start_area_confirmed = True

    node._on_gps(_Fix(START_LAT, START_LON))

    start, obstacle, end = node._waypoints
    assert math.isclose(start.x, 30.0, abs_tol=0.01)
    assert math.isclose(start.y, -4.0, abs_tol=0.01)
    north_per_1e4 = 1.0e-4 * M_PER_DEG_LAT
    assert math.isclose(obstacle.x, 30.0, abs_tol=0.05)
    assert math.isclose(obstacle.y, -4.0 + north_per_1e4, abs_tol=0.1)
    assert math.isclose(end.y, -4.0 + 2 * north_per_1e4, abs_tol=0.2)
    assert node._published == 1


def test_anchor_waits_for_odometry():
    node = _make_node()
    node._gps_start_area_confirmed = True

    node._on_gps(_Fix(START_LAT, START_LON))

    assert node._origin_lat is None
    assert all(w.x is None for w in node._waypoints)


def test_far_fix_does_not_anchor():
    node = _make_node()
    node._robot_xy = (0.0, 0.0)

    node._on_gps(_Fix(_north_of_start(500.0), START_LON))

    assert node._origin_lat is None


# ---- heading: odom axes follow the start heading, not east (2026-10-08) ----

def _rot(theta, x, y):
    return (math.cos(theta) * x - math.sin(theta) * y,
            math.sin(theta) * x + math.cos(theta) * y)


def test_fit_recovers_the_rotation_at_any_heading():
    for deg in (0.0, 90.0, 135.0, -160.0):
        theta = math.radians(deg)
        samples = []
        for k in range(16):                      # 15 m straight along odom x
            ox, oy = float(k), 0.3 * math.sin(k / 3.0)
            e, n = _rot(theta, ox, oy)
            samples.append((ox + 5.0, oy - 2.0, e + 100.0, n + 40.0))
        yaw, residual, travel = MissionPlannerNode.fit_odom_to_enu_yaw(samples)
        assert abs(math.atan2(math.sin(yaw - theta), math.cos(yaw - theta))) < 1e-6
        assert residual < 1e-6
        assert travel > 14.9


def test_fit_needs_three_samples():
    assert MissionPlannerNode.fit_odom_to_enu_yaw([(0, 0, 0, 0), (1, 0, 0, 1)]) is None


def test_anchor_rotates_by_the_estimated_heading():
    # odom x points north (yaw +90 deg).  A waypoint 1e-4 deg north of the
    # anchoring fix is about 11.1 m north, which is +11.1 m along odom x.
    node = _make_node()
    node._yaw_fit_enabled = True
    node._odom_enu_yaw = math.pi / 2
    node._robot_xy = (0.0, 0.0)
    node._gps_start_area_confirmed = True

    node._on_gps(_Fix(START_LAT, START_LON))

    north = 1.0e-4 * M_PER_DEG_LAT
    obstacle = node._waypoints[1]
    assert math.isclose(obstacle.x, north, abs_tol=0.1)
    assert math.isclose(obstacle.y, 0.0, abs_tol=0.1)


def test_the_old_assumption_puts_it_90_degrees_off():
    # The same with the fit disabled: north goes to odom +y, 11 m from truth.
    node = _make_node()
    node._yaw_fit_enabled = False
    node._odom_enu_yaw = math.pi / 2
    node._robot_xy = (0.0, 0.0)
    node._gps_start_area_confirmed = True

    node._on_gps(_Fix(START_LAT, START_LON))

    obstacle = node._waypoints[1]
    assert math.isclose(obstacle.x, 0.0, abs_tol=0.1)
    assert obstacle.y > 11.0


def test_does_not_anchor_without_a_heading_estimate():
    node = _make_node()
    node._yaw_fit_enabled = True
    node._robot_xy = (0.0, 0.0)
    node._gps_start_area_confirmed = True

    node._on_gps(_Fix(START_LAT, START_LON))

    assert node._origin_lat is None
    assert any('heading' in w for w in node._log.warnings)


def test_driving_builds_the_estimate_then_anchors():
    # The robot faces north, so it drives along odom +x while the fixes go
    # north.  After 12 m the fit exists, the gate is met (start waypoint
    # placed at the end of the drive), and the anchor uses yaw +90 deg.
    node = _make_node(start=(_north_of_start(12.0), START_LON))
    node._yaw_fit_enabled = True
    for k in range(13):
        node._robot_xy = (float(k), 0.0)
        node._on_gps(_Fix(_north_of_start(float(k)), START_LON))
    assert node._odom_enu_yaw is not None
    assert abs(math.degrees(node._odom_enu_yaw) - 90.0) < 0.5
    assert node._origin_lat is not None
    assert len(node._yaw_published) >= 1
