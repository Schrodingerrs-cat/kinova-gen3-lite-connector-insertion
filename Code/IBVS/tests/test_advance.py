"""Wall plane, clearance and travel planning on synthetic depth. Logic checks, not hardware evidence."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ibvs.advance import backproject, clearance, fit_wall, plan_travel  # noqa: E402

K = np.array([[435., 0, 426.5], [0, 434.4, 238.2], [0, 0, 1]])
D = np.zeros(5)


def wall_depth(tilt_deg, dist=.29, noise=.0005, seed=0):
    """Depth image of a plane `dist` m away along the optical axis, rotated by tilt_deg about camera y."""
    n = Rot.from_euler("y", tilt_deg, degrees=True).apply([0, 0, -1.])           # normal toward the camera
    v, u = np.mgrid[0:480, 0:848]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    t = dist / (-(rays @ n))                                                     # plane: n.P + dist = 0 at the axis point
    return t * 1.0 + np.random.default_rng(seed).normal(0, noise, t.shape), n


@pytest.mark.parametrize("tilt", [0., 4., -7.])
def test_wall_normal_and_distance_recovered(tilt):
    depth, n_true = wall_depth(tilt)
    n, d, count = fit_wall(depth, (300, 250, 560, 470), np.zeros(depth.shape, bool), K, D, .07, 1.)
    assert np.degrees(np.arccos(np.clip(n @ n_true, -1, 1))) < .5 and n[2] < 0
    assert abs(d - .29) < .003 and count > 20000


def test_excluded_gripper_pixels_do_not_bend_the_plane():
    depth, _ = wall_depth(0.)
    depth[400:470, 400:500] = .10                       # fingers in the region
    gripper = np.zeros(depth.shape, bool); gripper[400:470, 400:500] = True
    n, d, _ = fit_wall(depth, (300, 250, 560, 470), gripper, K, D, .07, 1.)
    assert abs(d - .29) < .003 and abs(n[2]) > .999


def test_wall_not_measured_when_too_little_depth():
    depth = np.zeros((480, 848))
    with pytest.raises(ValueError, match="wall not measured"):
        fit_wall(depth, (300, 250, 560, 470), np.zeros(depth.shape, bool), K, D, .07, 1.)


def test_clearance_is_nearest_gripper_point():
    n, d = np.array([0, 0, -1.]), .29
    pts = np.array([[0, 0, .10], [.01, 0, .12], [0, .01, .11]])
    assert clearance(pts, n, d) == pytest.approx(.17)
    assert clearance(np.array([[0, 0, .30]]), n, d) < 0                    # past the wall


def test_backproject_uses_pixel_rays():
    depth = np.zeros((480, 848)); depth[238, 426] = .5
    mask = depth > 0
    p = backproject(depth, mask, K, D)
    assert p.shape == (1, 3) and abs(p[0, 2] - .5) < 1e-6 and abs(p[0, 0]) < 1e-3


def test_travel_trimmed_without_contact_permission():
    assert plan_travel(.163, .15, .015, False) == (pytest.approx(.148), True)
    assert plan_travel(.30, .15, .015, False) == (.15, False)
    assert plan_travel(.01, .15, .015, False) == (0., True)
    assert plan_travel(.163, .15, .015, True) == (.15, False)               # explicit permission: full stroke


def test_scene_starts_beyond_gripper_and_held_object():
    from ibvs.depth import scene_near_depth
    from ibvs.frames import matrix
    T_EC = matrix([-.008, -.0496, -.091], [0, 0, 0, 1])                 # camera 9.1 cm behind the tool origin
    T_tcp = matrix([0, 0, .035], [0, 0, 0, 1])
    assert scene_near_depth({}, T_EC, T_tcp) == pytest.approx(.18)       # not the old 0.11: a plug sticks out to 0.12-0.13 m
    assert scene_near_depth({'scene_min_depth_m': .11}, T_EC, T_tcp) == .11
    far_tcp = matrix([0, 0, .20], [0, 0, 0, 1])
    assert scene_near_depth({}, T_EC, far_tcp) == pytest.approx(.20 + .091 + .04)
