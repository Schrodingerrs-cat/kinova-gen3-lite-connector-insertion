"""Direct joint path keeps the same limits as a planned one; fast profile stays inside the hard guards."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
import alignment_test  # noqa: E402
from joint2_priority_servo import fast_profile, smooth_profile  # noqa: E402
from ibvs.alignment import smooth_servo_path  # noqa: E402


def experiment(tmp_path, limit_deg=5.5):
    e = alignment_test.Experiment.__new__(alignment_test.Experiment)
    e.r = SimpleNamespace(q=np.zeros(6))
    e.c = dict(max_servo_joint_step_deg=limit_deg, max_camera_travel_m=1., max_camera_rotation_deg=90.)
    e.output, e.phase = tmp_path, 'ALIGN'
    e.fk = lambda q: np.eye(4)
    e.T_EC = e.T_tcp = np.eye(4)
    e.floor, e.origin = -1., None
    return e


def test_direct_path_is_start_plus_target(tmp_path):
    e = experiment(tmp_path)
    path = e.direct_path(np.radians([1, 0, 0, 0, 0, 0]))
    assert path.shape == (2, 6) and np.allclose(path[0], 0) and np.isclose(np.degrees(path[1, 0]), 1)


def test_direct_path_rejects_big_joint_step_and_logs_it(tmp_path):
    e = experiment(tmp_path)
    with pytest.raises(RuntimeError, match='step limit'):
        e.direct_path(np.radians([0, 0, 6, 0, 0, 0]))
    assert (tmp_path / 'rejected_paths.jsonl').exists()


def test_direct_path_rejects_non_finite(tmp_path):
    with pytest.raises(RuntimeError, match='non-finite'):
        experiment(tmp_path).direct_path(np.array([np.nan, 0, 0, 0, 0, 0]))


def test_direct_path_applies_geometric_limits(tmp_path):
    e = experiment(tmp_path)
    e.floor = 1.   # fingertip/camera below the floor
    with pytest.raises(ValueError, match='table clearance'):
        e.direct_path(np.radians([1, 0, 0, 0, 0, 0]))


def test_collision_sample_spacing_follows_config():
    path = np.array([np.zeros(6), np.radians([0, 0, 3, 0, 0, 0])])
    fine, coarse = smooth_servo_path(path, .25), smooth_servo_path(path, 1.)
    assert len(coarse) < len(fine)
    for samples, spacing in ((fine, .25), (coarse, 1.)):
        assert np.degrees(np.abs(np.diff(samples, axis=0))).max() <= spacing + 1e-9
        assert np.allclose(samples[0], path[0]) and np.allclose(samples[-1], path[-1])


def test_fast_profile_inside_hard_guards():
    c = {}
    fast_profile(c)
    assert c['gain'] <= 1 and 'control_hz' not in c
    approach, fine = c['approach_limits_deg'], c['fine_limits_deg']
    assert max(approach[i] for i in (0, 2, 3, 4, 5)) <= 3.   # the 3-degree guard on the non-joint-2 axes
    assert max(approach) <= 5.5 and all(f < a for f, a in zip(fine, approach))
    d = {}
    smooth_profile(d)
    assert c['joint_speed_deg_s'] > d['joint_speed_deg_s'] and c['min_move_s'] < d['min_move_s']


def test_velocities_flow_through_waypoints_and_stop_at_the_ends():
    from ibvs.alignment import smooth_velocities
    path = np.array([[0.] * 6, [.01] * 6, [.02] * 6, [.03] * 6]); times = np.array([.1, .2, .3, .4])
    v = smooth_velocities(path, times)
    assert np.allclose(v[0], 0) and np.allclose(v[-1], 0) and np.allclose(v[1:-1], .1)    # constant speed inside, no stops at knots
    back = smooth_velocities(np.array([[0.] * 6, [.01] * 6, [0.] * 6]), np.array([.1, .2, .3]))
    assert np.allclose(back[1], 0)                                                         # reversal: stops at the turning point


def test_continuous_timing_is_faster_than_the_stopping_profile():
    from ibvs.alignment import trajectory_times
    path = np.array([[0.] * 6, [np.radians(20)] + [0.] * 5, [np.radians(40)] + [0.] * 5])
    stop = trajectory_times(path, 10., 0., .1)[-1]
    flow = trajectory_times(path, 10., 0., .1, peak_factor=1.25)[-1]
    assert flow == pytest.approx(.1 + (stop - .1) * 1.25 / 1.875)


def test_speed_scale_multiplies_limits_and_shortens_waits():
    from alignment_test import scale_speeds
    c = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5,
             max_step_m=.002, max_step_deg=.4)
    scale_speeds(c, 2.)
    assert (c['joint_speed_deg_s'], c['linear_speed_m_s'], c['angular_speed_deg_s']) == (10., .01, 4.)
    assert c['min_move_s'] == .3 and c['arrival_settle_s'] == .25
    assert c['max_step_m'] == .004 and c['max_step_deg'] == .8                              # bigger steps: lateral moves of cm are not 2 mm at a time
    assert c['approach_limits_deg'] == [2., 5., 2., 2., 2., 2.] and c['fine_limits_deg'] == [.5, 1.5, .5, .5, .5, .5]
    d = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5, max_step_m=.002, max_step_deg=.4)
    scale_speeds(d, 1.)
    assert d['joint_speed_deg_s'] == 5. and d['max_step_m'] == .002 and d['min_move_s'] == .6    # 1.0 changes no speed or step


def test_scaled_step_limits_stay_under_the_hard_guards():
    from alignment_test import scale_speeds
    for scale in (1., 2., 3.):
        c = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5)
        scale_speeds(c, scale)
        assert max(c['approach_limits_deg']) <= 5. and max(c['approach_limits_deg'][i] for i in (0, 2, 3, 4, 5)) <= 2.5   # 3 deg guard on joints 1,3-6
        assert max(c['fine_limits_deg']) <= 1.5
