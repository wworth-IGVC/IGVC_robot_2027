"""A sideways run in the raw path must not make the conditioned path invalid.

The centerline search may step sideways without advancing (neighbours (-1, 0)
and (1, 0)). `_limit_path_heading` flattens such a run into repeated points at
one forward position, and `_smooth_path_points` then gives those repeats
different lateral values: straight sideways steps, heading exactly pi/2.
`_path_invalid_reason` rejects any heading over `max_path_heading_rad`, which it
caps at pi/2 - 1e-3, so even the simulator's "most the validator accepts"
(1.57) refuses them: "pose 9 path heading 1.57rad > 1.57rad". The navigator then
holds the previous path for up to stuck_timeout_sec and reverses (measured:
every lidar-map run, 6 to 22 warnings; one run never recovered and sat on the
paint).

Grids and paths here are in the navigator's own convention (forward along
columns); the older costmap_fixtures.py is transposed and is not used.
"""

from __future__ import annotations

import math

import pytest

from builtin_interfaces.msg import Time

from igvc_lane_detection.navigator import IGVCNavigatorNode


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def _navigator(**overrides):
    """A navigator shell with the parameters the path conditioning reads."""
    node = object.__new__(IGVCNavigatorNode)
    node._max_path_heading_rad = 1.57
    node._grid_res = 0.05
    node._path_smooth_window = 5
    node._path_sample_spacing_m = 0.10
    node._min_follow_path_poses = 5
    node._min_follow_path_length_m = 1.5
    node._max_path_lateral_jump_m = 0.60
    node._base_frame = 'base_link'
    node._last_lane_path_reason = ''
    node.get_logger = lambda: _Logger()
    for key, value in overrides.items():
        setattr(node, key, value)
    return node


def _raw_with_sideways_run(run_cells, drift_m=None):
    """Forward 0.3 m, a sideways run of `run_cells` cells, forward again."""
    raw = [(0.35 + 0.1 * i, 0.05) for i in range(3)]
    fwd = raw[-1][0]
    raw += [(fwd, 0.05 - 0.1 * k) for k in range(1, run_cells + 1)]
    lat = 0.05 - 0.1 * run_cells
    raw += [(fwd + 0.1 * i, lat) for i in range(1, 30)]
    return raw


def _reason(node, raw):
    path = node._build_path(node._condition_path_points(raw), Time(sec=1))
    return node._path_invalid_reason(path)


@pytest.mark.parametrize('run_cells', [6, 10])
def test_sideways_run_survives_conditioning_at_the_sim_heading_limit(run_cells):
    """Six cells or more sideways at one forward position was rejected."""
    assert _reason(_navigator(), _raw_with_sideways_run(run_cells)) is None


def test_short_sideways_run_was_never_a_problem():
    """Control: two cells sideways passes on the shipped code and still does."""
    assert _reason(_navigator(), _raw_with_sideways_run(2)) is None


def test_forward_only_paths_come_out_exactly_as_before():
    """Control: a path with no sideways step is untouched by the repair.

    The reference is the shipped composition, limit then smooth then resample.
    """
    node = _navigator()
    raw = [(0.35 + 0.1 * i, 0.05 - 0.02 * i) for i in range(35)]
    anchored = [(0.0, 0.0)] + raw
    reference = node._resample_path_points(
        node._smooth_path_points(node._limit_path_heading(anchored)),
        node._path_sample_spacing_m)
    assert node._condition_path_points(raw) == reference


def test_valid_paths_with_a_sideways_run_are_unchanged_at_the_robots_limit():
    """Control: at 0.75 rad a path that validates today is returned as is.

    A two-cell run is the case that validates today (see above); its output
    must equal the shipped composition whether or not the repair exists.
    """
    node = _navigator(_max_path_heading_rad=0.75)
    raw = _raw_with_sideways_run(2)
    anchored = [(0.0, 0.0)] + raw
    reference = node._resample_path_points(
        node._smooth_path_points(node._limit_path_heading(anchored)),
        node._path_sample_spacing_m)
    reference_path = node._build_path(reference, Time(sec=1))
    assert node._path_invalid_reason(reference_path) is None
    assert node._condition_path_points(raw) == reference


def test_repaired_path_still_starts_at_the_robot_and_reaches_the_end():
    """The repair must not truncate or shift the path: control on both."""
    node = _navigator()
    raw = _raw_with_sideways_run(8)
    points = node._condition_path_points(raw)
    assert points[0] == (0.0, 0.0)
    assert points[-1][0] == pytest.approx(raw[-1][0], abs=0.11)
    assert points[-1][1] == pytest.approx(raw[-1][1], abs=0.15)
    assert all(b[0] >= a[0] - 1.0e-9 for a, b in zip(points, points[1:]))


def test_validator_still_rejects_a_real_lateral_jump():
    """Control: the validator itself is untouched."""
    node = _navigator()
    points = [(0.1 * i, 0.0) for i in range(10)] + [(1.0, 1.2), (1.1, 1.2)]
    path = node._build_path(points, Time(sec=1))
    reason = node._path_invalid_reason(path)
    assert reason is not None and 'lateral jump' in reason


def test_exact_pi_over_two_step_is_what_the_validator_refuses():
    """Control: documents the rejection the repair exists to avoid."""
    node = _navigator()
    points = ([(0.1 * i, 0.0) for i in range(8)]
              + [(0.7, 0.1), (0.7, 0.2)]
              + [(0.7 + 0.1 * i, 0.2) for i in range(1, 12)])
    path = node._build_path(points, Time(sec=1))
    reason = node._path_invalid_reason(path)
    assert reason is not None and 'path heading' in reason
    assert math.isclose(math.pi / 2.0, 1.5707963267948966)


# ── the clamp and the validator must agree at the limit itself ───────────────

def _path_with_step_heading(heading_rad):
    """A straight 2 m path whose every step has exactly this heading."""
    points = [(0.1 * i * math.cos(heading_rad), 0.1 * i * math.sin(heading_rad))
              for i in range(21)]
    return points


def test_validator_accepts_a_step_at_the_limit_to_within_rounding():
    """The limiter saturates at tan(limit) * df, which comes back from atan2 as
    limit plus about 2e-15 rad; that is the limit, not a violation. Measured on
    the shipped 0.75 rad default: 51 of 59 sampled cycles rejected with
    "path heading 0.75rad > 0.75rad"."""
    node = _navigator(_max_path_heading_rad=0.75)
    path = node._build_path(_path_with_step_heading(0.75 + 1.0e-12), Time(sec=1))
    assert node._path_invalid_reason(path) is None


def test_validator_still_rejects_a_step_over_the_limit():
    """Control: a real violation, a thousandth of a radian over, is refused."""
    node = _navigator(_max_path_heading_rad=0.75)
    path = node._build_path(_path_with_step_heading(0.75 + 1.0e-3), Time(sec=1))
    reason = node._path_invalid_reason(path)
    assert reason is not None and 'path heading' in reason


def test_a_saturated_clamp_output_validates_at_the_robots_limit():
    """A raw path steeper than the limit is clamped to it and must then pass."""
    node = _navigator(_max_path_heading_rad=0.75)
    raw = [(0.35 + 0.1 * i, 0.05 - 0.17 * i) for i in range(20)]
    raw += [(raw[-1][0] + 0.1 * i, raw[-1][1]) for i in range(1, 15)]
    assert _reason(node, raw) is None
