"""Synthetic tests: geometry, signs, bounds, logic. NOT hardware evidence."""
import pathlib, sys
import cv2
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ibvs.config import load_config  # noqa: E402
from ibvs.control import Convergence, axes_matrix, damped_pinv, interaction, limit, servo_twist  # noqa: E402
from ibvs.depth import corner_depths, fill_depths  # noqa: E402
from ibvs.frames import adjoint, matrix  # noqa: E402
from ibvs.guards import check_axial, check_servo  # noqa: E402
from ibvs.logger import TrialLogger  # noqa: E402
from ibvs.markers import Tracker  # noqa: E402
from ibvs.search import candidates  # noqa: E402
from ibvs.state_machine import run_cycle  # noqa: E402
from ibvs.verify import Seat, seat_state  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

SQUARE = np.array([[-.05, -.05, 0], [.05, -.05, 0], [.05, .05, 0], [-.05, .05, 0]])   # board points, board frame


def project(T_cb, pts=SQUARE):
    """Normalized image points of board points, board pose T_cb in the camera frame."""
    P = pts @ T_cb[:3, :3].T + T_cb[:3, 3]
    return P[:, :2] / P[:, 2:], P[:, 2]


def step_pose(T_cb, xi, dt):
    """Camera moves with twist xi (camera frame): board pose in camera frame changes by the inverse motion."""
    dT = np.eye(4)
    dT[:3, :3] = Rot.from_rotvec(xi[3:] * dt).as_matrix()
    dT[:3, 3] = xi[:3] * dt
    return np.linalg.inv(dT) @ T_cb


@pytest.mark.parametrize("axes", [np.eye(3), Rot.from_euler("xyz", [0.3, -0.2, 0.5]).as_matrix()])
def test_closed_loop_converges(axes):
    # board's pose chosen so the goal is reachable inside the controlled DoFs (lateral + yaw about z_S)
    T_goal = matrix([0.01, -0.02, 0.30], Rot.from_euler("xyz", [0.05, -0.04, 0.2]).as_quat())
    A = axes_matrix(axes)
    s_star, _ = project(T_goal)
    T = T_goal.copy()
    # perturb inside the controlled subspace: move the camera by A @ u
    for u in ([0.02, -0.015, 0.0], [0.0, 0.0, 0.25], [-0.015, 0.02, -0.2]):
        T = step_pose(T_goal, A @ np.array(u), 1.0)
        e0 = np.abs(project(T)[0] - s_star).max()
        for _ in range(1500):
            s, Z = project(T)
            u3 = servo_twist(s, s_star, Z, A, 2.0, 1e-3, 1e4)
            T = step_pose(T, A @ u3, 0.02)
        assert np.abs(project(T)[0] - s_star).max() < 0.02 * e0 + 1e-4, u


def test_sign_one_axis():
    # target to the right of centre in the image -> camera must move +x (right) to centre it
    s_star = SQUARE[:, :2] / 0.3; s = s_star + [0.05, 0.0]
    u = servo_twist(s, s_star, np.full(4, 0.3), axes_matrix(np.eye(3)), 1.0, 1e-3, 1e4)
    assert u[0] > 0 and abs(u[1]) < 1e-9


def test_ill_conditioned_returns_none():
    s = np.zeros((4, 2))
    assert servo_twist(s, s, np.full(4, 0.3), axes_matrix(np.eye(3)), 1, 1e-3, 1.0) is None


def test_interaction_matches_finite_difference():
    T = matrix([0.01, 0.02, 0.3], Rot.from_euler("xyz", [.1, .2, .3]).as_quat())
    xi = np.array([.01, -.02, .03, .1, -.2, .15])
    s0, Z = project(T)
    s1, _ = project(step_pose(T, xi, 1e-6))
    assert np.allclose((s1 - s0).ravel() / 1e-6, interaction(s0, Z) @ xi, atol=1e-3)


def test_adjoint_rotation_offset():
    T = matrix([0.1, 0, 0], [0, 0, 0, 1])           # frame C is 0.1 m along +x of E, same orientation
    v = adjoint(T) @ np.array([0, 0, 0, 0, 0, 1.0])   # spin about C z -> E origin sweeps at w x (-t)... = t x w
    assert np.allclose(v[:3], np.cross([0.1, 0, 0], [0, 0, 1.0])) and np.allclose(v[3:], [0, 0, 1])


def test_limit_caps_and_slews():
    u = limit([1, 0, 5], np.zeros(3), 0.02, 0.1, 0.5, 0.1)
    assert np.allclose(u, [0.02, 0, 0.05])            # lateral cap 0.02 binds, yaw slew 0.5*0.1 binds
    u = limit([1, 0, 5], np.array([0.02, 0, 0.1]), 0.02, 0.1, 10, 0.1)
    assert np.allclose(u, [0.02, 0, 0.1])


def test_convergence_needs_consecutive_frames():
    c = Convergence(1.0, 3)
    assert [c.update(x) for x in (0.5, 0.5, 2.0, 0.5, 0.5, 0.5)] == [False] * 5 + [True]


def test_depth_fill():
    d = np.zeros((20, 20)); d[:, :10] = 0.3
    Z = corner_depths(d, np.array([[3., 5.], [15., 5.]]), 0.07, 0.5)
    assert Z[0] == pytest.approx(0.3) and np.isnan(Z[1])
    assert np.allclose(fill_depths(Z), [0.3, 0.3]) and fill_depths(np.full(2, np.nan)) is None


S = dict(step_x_m=0.001, step_y_m=0.001, step_yaw_rad=0.02, max_abs_x_m=0.002, max_abs_y_m=0.001,
         max_abs_yaw_rad=0.02, sigma_xy_m=0.002, sigma_yaw_rad=0.02, max_attempts=100, max_total_time_s=60)


def test_search_bounded_nominal_first():
    c = candidates(S)
    assert c[0] == (0, 0, 0) and len(c) == len(set(c)) == 5 * 3 * 3
    assert max(abs(x) for x, _, _ in c) <= 0.002 and max(abs(y) for _, y, _ in c) <= 0.001
    assert candidates({**S, "max_attempts": 4}) == c[:4]
    assert all(max(abs(x) / 0.001, abs(y) / 0.001, abs(w) / 0.02) <= 1 for x, y, w in c[:27 - 18])  # inner ring first


G = dict(camera_stale_s=0.2, joint_stale_s=0.2)
BOX = [0, 1, 0, 1, 0, 1]


def test_servo_guards():
    ok = dict(now=10, cam_stamp=9.95, joint_stamp=9.95, tool_xyz=(.5, .5, .5), g=G, box=BOX, t_start=9, timeout=5,
              missing_frames=0, max_missing=3)
    assert check_servo(**ok) is None
    assert check_servo(**{**ok, "cam_stamp": 9}) == "CAMERA_STALE"
    assert check_servo(**{**ok, "joint_stamp": 9}) == "JOINTS_STALE"
    assert check_servo(**{**ok, "tool_xyz": (2, .5, .5)}) == "OUT_OF_BOUNDS"
    assert check_servo(**{**ok, "t_start": 1}) == "TIMEOUT"
    assert check_servo(**{**ok, "missing_frames": 4}) == "MARKERS_LOST"


def test_axial_guard():
    ax = dict(max_travel_m=0.01, min_progress_frac=0.5)
    assert check_axial(0.002, 0.002, 0.0, ax) is None
    assert check_axial(0.002, 0.0005, 0.0, ax) == "NO_PROGRESS"
    assert check_axial(0.002, 0.002, 0.009, ax) == "MAX_TRAVEL"


V = dict(stable_dwell_s=1.0, stale_s=0.3, success_value=True)


def test_seat_states():
    on = [(t / 10, True) for t in range(0, 21)]
    assert seat_state(on, 2.0, V) is Seat.INSERTED_VERIFIED
    assert seat_state(on[:5], 0.4, V) is Seat.UNKNOWN_SENSOR           # not enough history
    assert seat_state(on, 3.0, V) is Seat.UNKNOWN_SENSOR               # stale
    assert seat_state(on[:-1] + [(2.0, False)], 2.0, V) is Seat.NOT_INSERTED
    assert seat_state([], 1.0, V) is Seat.UNKNOWN_SENSOR


class Fake:
    def __init__(self, seats, **ret):
        self.seats, self.ret, self.calls = list(seats), ret, []

    def __getattr__(self, n):
        def f(*a):
            self.calls.append(n)
            if n == "verify_seated":
                return self.seats.pop(0)
            return self.ret.get(n, "OK" if n == "attempt" else True)
        return f


class Log:
    def __init__(self): self.rows = []
    def row(self, **kw): self.rows.append(kw)


def cycle(p, mode, cands=((0., 0., 0.), (0.001, 0., 0.), (0., 0.001, 0.))):
    return run_cycle(p, mode, cands, Log(), 60, lambda: 0.0)


def test_machine_a2_search_until_verified():
    p = Fake([Seat.NOT_INSERTED, Seat.NOT_INSERTED, Seat.INSERTED_VERIFIED])
    assert cycle(p, "A2") == ("DONE", "verified")
    assert p.calls.count("attempt") == 3 and p.calls.count("retract") == 2 and p.calls.count("apply_search_offset") == 2


def test_machine_a1_single_attempt_and_b0_no_servo():
    p = Fake([Seat.NOT_INSERTED]); assert cycle(p, "A1") == ("DONE", "not_inserted") and p.calls.count("attempt") == 1
    p = Fake([Seat.INSERTED_VERIFIED]); cycle(p, "B0"); assert "servo_align" not in p.calls


@pytest.mark.parametrize("seat", [Seat.UNKNOWN_SENSOR, Seat.FAULT])
def test_machine_aborts_on_bad_sensor(seat):
    p = Fake([seat])
    assert cycle(p, "A2")[0] == "ABORT" and p.calls.count("attempt") == 1 and "abort" in p.calls


def test_machine_aborts_on_faults():
    assert cycle(Fake([], servo_align="MARKERS_LOST"), "A1")[0] == "ABORT"
    assert cycle(Fake([], attempt="JAM"), "A1")[0] == "ABORT"
    assert cycle(Fake([], guard_check="OUT_OF_BOUNDS"), "A1")[0] == "ABORT"
    assert cycle(Fake([], verify_grasp=False), "A1")[0] == "ABORT"


def test_machine_search_timeout():
    p = Fake([Seat.NOT_INSERTED] * 3)
    t = iter(range(0, 1000, 100))
    r = run_cycle(p, "A2", [(0., 0., 0.), (0.001, 0., 0.)], Log(), 60, lambda: next(t))
    assert r == ("ABORT", "SEARCH_TIMEOUT")


def test_config_refuses_unmeasured_and_uncalibrated(tmp_path):
    cfg = pathlib.Path(__file__).resolve().parents[1] / "config" / "ibvs.yaml"
    with pytest.raises(ValueError, match="camera.K"):
        load_config(cfg)
    with pytest.raises(ValueError, match="status"):
        load_config(cfg, contact=True)
    d = yaml.safe_load(cfg.read_text())
    def fill(x):
        return {k: fill(v) if isinstance(v, dict) else (0 if v is None else v) for k, v in x.items()}
    d = fill(d)
    f = tmp_path / "c.yaml"; f.write_text(yaml.safe_dump(d))
    assert load_config(f)["config_hash"]
    with pytest.raises(ValueError, match="supervisor_approved"):
        load_config(f, contact=True)
    d["status"] = "CALIBRATED"
    for k in ("positive_control_passed", "negative_control_passed"): d["verification"][k] = True
    for k in d["safety"]: d["safety"][k] = True
    f.write_text(yaml.safe_dump(d))
    assert load_config(f, contact=True)["status"] == "CALIBRATED"


def test_logger_columns(tmp_path):
    lg = TrialLogger(tmp_path / "l.csv", run_id="r")
    lg.row(phase="X", features_px=np.zeros((2, 2)))
    assert len(open(tmp_path / "l.csv").read().splitlines()) == 2


def _canvas(ids, scale=120):
    img = np.full((480, 640), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for k, i in enumerate(ids):
        m = cv2.aruco.generateImageMarker(d, i, scale)
        r, c = 40 + 200 * (k // 2), 60 + 250 * (k % 2)
        img[r:r + scale, c:c + scale] = m
    return img


def test_marker_tracker_corners_validity_and_jump():
    K = [[600, 0, 320], [0, 600, 240], [0, 0, 1]]
    # canvas puts markers at known corners: id0 at (60,40)..(180,160) etc.
    tgt = {i: [[60 + 250 * (k % 2) + dx, 40 + 200 * (k // 2) + dy] for dx, dy in ((0, 0), (120, 0), (120, 120), (0, 120))]
           for k, i in enumerate((0, 1, 2))}
    mk = dict(aruco_dict="DICT_4X4_50", ids=[0, 1, 2, 3], target_pixels={**tgt, 3: tgt[0]}, min_side_px=50, max_jump_px=30)
    t = Tracker(dict(K=K, distortion=[0] * 5), mk)
    px, valid = t.detect(_canvas([0, 1, 2]))
    assert valid.tolist() == [True, True, True, False]
    assert np.abs(px[0] - np.array(tgt[0])).max() < 2.0
    s, s_star, rms = t.features(px, valid)
    assert s.shape == (12, 2) and rms < 2.0
    px2, valid2 = t.detect(np.roll(_canvas([0, 1, 2]), 60, axis=1))   # jump of 60 px > max_jump_px
    assert not valid2.any() or valid2.sum() < 3
