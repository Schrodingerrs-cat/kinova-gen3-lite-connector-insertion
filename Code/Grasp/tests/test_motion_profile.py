import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.motion_profile import duration_for, joint_move, path_move  # noqa: E402


def test_joint_move_starts_and_stops_at_rest_and_reaches_the_goal():
    q0, q1 = np.zeros(6), np.radians([80, 20, -10, 5, 0, 40])
    t, p, v, a = joint_move(q0, q1, 3.2)
    assert np.allclose(p[0], q0) and np.allclose(p[-1], q1)
    assert np.abs(v[[0, -1]]).max() < 1e-12 and np.abs(a[[0, -1]]).max() < 1e-12
    assert np.all(np.diff(t) > 0)


def test_velocity_and_acceleration_are_the_derivatives_of_position_no_jumps():
    t, p, v, a = joint_move(np.zeros(6), np.radians([60, -30, 20, 10, 5, -40]), 2.5, rate_hz=200)
    assert np.abs(np.gradient(p, t, axis=0) - v).max() < np.radians(0.5)        # v matches dp/dt
    assert np.abs(np.gradient(v, t, axis=0) - a).max() < np.radians(2.0)        # a matches dv/dt
    assert np.abs(np.diff(a, axis=0)).max() < 0.05 * np.abs(a).max()           # acceleration changes smoothly (bounded jerk)


def test_duration_respects_speed_peak_and_minimum():
    sp, pk = np.radians(25), np.radians(60)
    assert np.isclose(duration_for(np.radians(80), sp, pk, 1.0), 3.2)           # average speed rules
    assert np.isclose(duration_for(np.radians(80), np.radians(200), pk, 1.0), 1.875 * np.radians(80) / pk)   # peak cap rules
    assert duration_for(np.radians(2), sp, pk, 1.0) == 1.0                      # tiny move: the minimum
    t, p, v, a = joint_move(np.zeros(6), np.radians([80, 0, 0, 0, 0, 0]), duration_for(np.radians(80), sp, pk, 1.0))
    assert np.degrees(np.abs(v).max()) <= 60 + 1e-6 or np.degrees(np.abs(v).max()) <= 1.875 * 25 + 1e-6


def test_line_waypoints_get_a_profile_that_starts_and_stops_at_rest():
    pts = np.cumsum(np.tile([0.004, -0.002, 0.003, 0.001, -0.002, 0.0015], (25, 1)), axis=0)
    t, p, v, a = path_move(pts, 2.0)
    assert np.allclose(p, pts) and np.all(np.diff(t) > 0) and np.isclose(t[-1], 2.0)
    assert np.abs(v[[0, -1]]).max() < 1e-9 and np.abs(a[[0, -1]]).max() < 1e-6
    old = np.abs(np.gradient(pts, np.linspace(0, 2.0, len(pts)), axis=0)[0]).max()   # constant-speed timing: a speed step at t = 0
    assert old > 0.03 and np.abs(v[0]).max() == 0.0
