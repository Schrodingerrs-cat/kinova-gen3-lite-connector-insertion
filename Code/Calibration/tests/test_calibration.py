"""Synthetic checks of calib_lib (geometry only: the true answer is known). Not hardware evidence."""
import pathlib, sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import calib_lib as cl  # noqa: E402


def _poses(n, rng):
    Ts = []
    for _ in range(n):
        T = np.eye(4)
        T[:3, :3] = Rot.from_rotvec(rng.normal(size=3) * 0.5).as_matrix() @ np.diag([1, -1, -1])   # roughly looking down, tilted about all axes
        T[:3, 3] = [rng.uniform(0.25, 0.5), rng.uniform(-0.15, 0.15), rng.uniform(0.25, 0.4)]
        Ts.append(T)
    return Ts


def _scene(rng, noise_mm=0.0, noise_deg=0.0, n=12):
    X = cl.to_T([-0.008, -0.05, -0.09], Rot.from_rotvec([0.01, -0.02, 0.03]).as_quat())   # true tool_T_camera
    T_bm = cl.to_T([0.4, 0.0, -0.04], Rot.from_euler("x", 180, degrees=True).as_quat())    # fixed marker in the base
    T_bt = _poses(n, rng)
    T_cm = []
    for T in T_bt:
        M = np.linalg.inv(T @ X) @ T_bm
        M[:3, 3] += rng.normal(size=3) * noise_mm * 1e-3
        M[:3, :3] = Rot.from_rotvec(rng.normal(size=3) * np.radians(noise_deg)).as_matrix() @ M[:3, :3]
        T_cm.append(M)
    return X, T_bt, T_cm


def test_hand_eye_exact_without_noise():
    X, T_bt, T_cm = _scene(np.random.default_rng(0))
    mm, deg = cl.pose_difference(cl.hand_eye(T_bt, T_cm), X)
    assert mm < 1e-3 and deg < 1e-3


def test_hand_eye_with_marker_noise_stays_small_and_residual_reports_it():
    X, T_bt, T_cm = _scene(np.random.default_rng(1), noise_mm=0.5, noise_deg=0.3, n=15)
    Xe = cl.hand_eye(T_bt, T_cm)
    mm, deg = cl.pose_difference(Xe, X)
    assert mm < 3.0 and deg < 0.7
    r = cl.hand_eye_residual(T_bt, T_cm, Xe)
    assert r["max_dev_mm"] < 3.0
    assert cl.hand_eye_residual(T_bt, T_cm, cl.to_T([0, 0, 0], [0, 0, 0, 1]))["max_dev_mm"] > 20   # a wrong hand-eye is visible as a large spread


def test_plane_fit_gives_tilt_and_height():
    rng = np.random.default_rng(2)
    xy = rng.uniform(-0.2, 0.2, (20000, 2))
    P = np.c_[xy, 0.05 * xy[:, 0] + 0.01 + rng.normal(0, 0.0005, 20000)]
    P = np.vstack([P, np.c_[rng.uniform(-0.01, 0.01, (800, 2)), rng.uniform(0.03, 0.05, 800)]])   # an object on it
    n, d, _ = cl.fit_plane(P)
    tilt, h = cl.plane_tilt_height(n, d, (0.3, 0.0))
    assert abs(tilt - np.degrees(np.arctan(0.05))) < 0.2 and abs(h - (0.015 + 0.01)) < 0.001


def test_intrinsics_are_recovered_from_chessboard_views():
    rng = np.random.default_rng(3)
    K0 = np.array([[600.0, 0, 424], [0, 600.0, 240], [0, 0, 1]])
    d0 = np.array([0.05, -0.1, 0.0, 0.0, 0.0])
    obj = cl.chessboard_points(9, 6, 0.025)
    O, I = [], []
    for _ in range(14):
        rv, tv = rng.normal(size=3) * 0.25, np.array([rng.uniform(-0.08, 0.04), rng.uniform(-0.05, 0.03), rng.uniform(0.3, 0.5)])
        p, _ = cv2.projectPoints(obj, rv, tv, K0, d0)
        O.append(obj); I.append((p.reshape(-1, 2) + rng.normal(0, 0.2, (len(obj), 2))).astype(np.float32))
    rms, K, dist = cl.intrinsics(O, I, (848, 480))
    assert rms < 0.5 and abs(K[0, 0] - 600) < 6 and abs(K[1, 1] - 600) < 6 and abs(K[0, 2] - 424) < 6 and abs(K[1, 2] - 240) < 6


def test_aruco_marker_is_detected_with_the_right_distance():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    m = cv2.aruco.generateImageMarker(d, 7, 200)
    img = np.full((600, 600), 255, np.uint8)
    img[200:400, 200:400] = m
    K = np.array([[600.0, 0, 300], [0, 600.0, 300], [0, 0, 1]])
    T = cl.marker_pose(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, np.zeros(5), "DICT_4X4_50", 7, 0.05)
    assert T is not None and abs(T[2, 3] - 600 * 0.05 / 200) < 0.01 and abs(T[0, 3]) < 0.005
    assert cl.marker_pose(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, np.zeros(5), "DICT_4X4_50", 3, 0.05) is None


def test_view_poses_look_down_and_tilt_toward_the_centre():
    sc = dict(table_z_m=-0.04, look_at=[0.4, 0.0], views=[[0.3, 0.0, 0.3], [0.2, 0.2, 0.25, 30]])
    T0 = np.eye(4); T0[:3, :3] = np.diag([1, -1, -1])
    X = cl.to_T([0, 0, 0], [0, 0, 0, 1])
    down, tilt = cl.view_poses(sc, T0, X)
    assert abs(down[2, 2] + 1) < 1e-6                                  # straight-down view keeps the start orientation (z axis down)
    assert tilt[:3, 2] @ np.array([0.2, -0.2, 0]) / np.linalg.norm([0.2, -0.2]) > 0.3   # optical axis leans toward look_at
