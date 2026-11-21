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
