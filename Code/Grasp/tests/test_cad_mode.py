import pathlib
import sys

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp import cad_fit as cf  # noqa: E402
from housing_grasp import cad_place as cp  # noqa: E402

PLANE = dict(n=np.array([0.0, 0.0, 1.0]), d=0.0)


def _synthetic(name, lean_deg=0.0, seed=1, cable=True):
    """Connector standing in a panel at z = 0, seen from 3 tilted cameras (visible surface only, 0.5 mm noise, a cable leaving the top)."""
    cfg, spec = cf.load_config(name)
    M, N = cf.model_cloud(spec)
    rng = np.random.default_rng(seed)
    R = Rot.from_euler("xy", [np.radians(lean_deg), 0]).as_matrix() @ Rot.from_euler("z", np.radians(rng.uniform(0, 360))).as_matrix()
    t = np.array([250.0, 20.0, -spec["seat_u_mm"]])
    P, Nn = M @ R.T + t, N @ R.T
    views = []
    for az in (0, 120, 240):
        a, th = np.radians(az), np.radians(38)
        cam = np.array([250.0, 20.0, 10.0]) + 170 * np.array([np.sin(th) * np.cos(a), np.sin(th) * np.sin(a), np.cos(th)])
        vis = ((cam - P) / np.linalg.norm(cam - P, axis=1, keepdims=True) * Nn).sum(1) > 0.15
        Q = P[vis]
        Q = Q[rng.choice(len(Q), min(260, len(Q)), replace=False)]
        Q = Q + rng.normal(0, 0.5, Q.shape)
        if cable:
            s = np.linspace(2, 30, 40)[:, None]
            Q = np.vstack([Q, P[np.argmax(M[:, 2])] + s * np.array([0.7, 0.2, 0.6]) / 0.94 + rng.normal(0, 0.5, (40, 3))])
        views.append(Q)
    return cfg, spec, M, views, R, t


@pytest.mark.parametrize("name", ["micro_jack", "usb_a"])
def test_fit_recovers_the_pose_with_a_cable_in_the_points(name):
    cfg, spec, M, views, R, t = _synthetic(name)
    rep = cf.fit_views(views, M, spec, PLANE, cfg)
    T, ref = rep["T_base_model_mm"], np.array([0, 0, 0.5 * sum(spec["fit_u_mm"])])
    assert np.linalg.norm((T[:3, :3] @ ref + T[:3, 3]) - (R @ ref + t)) < 1.0
    assert np.degrees(np.arccos(np.clip(T[:3, 2] @ R[:, 2], -1, 1))) < 1.0 and rep["gate_pass"]
    assert abs(rep["implied_seat_u_mm"] - spec["seat_u_mm"]) < 1.0


def test_a_leaning_plug_is_found_by_the_held_out_views_not_assumed_upright():
    cfg, spec, M, views, R, t = _synthetic("micro_jack", lean_deg=8.0)
    rep = cf.fit_views(views, M, spec, PLANE, cfg)
    T = rep["T_base_model_mm"]
    assert rep["used_tilt_fit"] and np.degrees(np.arccos(np.clip(T[:3, 2] @ R[:, 2], -1, 1))) < 3.0


def test_a_wrong_panel_plane_fails_the_seat_gate():
    cfg, spec, M, views, R, t = _synthetic("micro_jack")
    rep = cf.fit_views(views, M, spec, dict(n=PLANE["n"], d=-10.0), cfg)               # panel 10 mm higher than the plug's seat
    assert not rep["gate_pass"] and any("panel plane" in f for f in rep["gate_failures"])


def test_a_connector_that_does_not_match_the_data_fails_the_gate():
    cfg, spec, M, views, R, t = _synthetic("micro_jack")
    _, usb = cf.load_config("usb_a")
    Musb, _ = cf.model_cloud(usb)
    rep = cf.fit_views(views, Musb, dict(usb, seat_u_mm=spec["seat_u_mm"], symmetry="flip"), PLANE, cfg)   # jack data, USB model
    assert not rep["gate_pass"]


def test_close_view_looks_at_the_target_from_the_tilt_and_distance():
    Rh = Rot.from_euler("x", 180, degrees=True).as_matrix()                                 # camera looking straight down
    tgt = np.array([0.26, -0.02, 0.0])
    T = cf.close_view_pose(tgt, 120, 0.17, 38, Rh)
    assert np.isclose(np.linalg.norm(T[:3, 3] - tgt), 0.17)
    assert np.allclose(T[:3, 2], (tgt - T[:3, 3]) / 0.17, atol=1e-9)                       # optical axis through the target
    assert np.isclose(np.degrees(np.arccos(-T[2, 2])), 38, atol=1e-6)                      # tilt from straight down
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-9)


def _jack_pose():
    cfg, spec = cf.load_config("micro_jack")
    T = np.eye(4)
    T[:3, 3] = [250, 20, -spec["seat_u_mm"]]
    return cfg, spec, T


def test_axial_connector_gets_the_jaw_yaws_the_arm_wants():
    cfg, spec, T = _jack_pose()
    cs, st = cp.place(T, cp.load_candidates(spec, cf.ROOT), spec, cfg["place"], PLANE, np.zeros((0, 3)), np.radians(85.5), 0.006, 0.0077, 0.012, 0.085)
    yaws = sorted({round(float(np.degrees(np.arctan2(c["jaw"][1], c["jaw"][0])) % 180)) for c in cs})
    assert yaws == [85, 175] and all(c["tip_above_panel_mm"] >= cfg["place"]["clearance_mm"] for c in cs) and np.allclose(cs[0]["approach"], [0, 0, -1])


def test_candidates_below_the_panel_clearance_are_dropped():
    cfg, spec, T = _jack_pose()
    raised = dict(PLANE, d=-3.0)                                                          # panel 3 mm higher: the lowest tips are now too close
    cs, st = cp.place(T, cp.load_candidates(spec, cf.ROOT), spec, cfg["place"], raised, np.zeros((0, 3)), 0.0, 0.006, 0.0077, 0.012, 0.085)
    assert st["panel"] > 0 and min(c["u0_mm"] for c in cs) > 6


def test_a_block_where_a_finger_closes_removes_that_candidate():
    cfg, spec, T = _jack_pose()
    cs, _ = cp.place(T, cp.load_candidates(spec, cf.ROOT), spec, cfg["place"], PLANE, np.zeros((0, 3)), 0.0, 0.006, 0.0077, 0.012, 0.085)
    b = cs[0]
    finger = b["p"] * 1e3 + b["jaw"] * (b["open_width_m"] * 500 + 3.0) + [0, 0, 6.0]       # inside the finger box, 6 mm above the tip
    block = finger + np.random.default_rng(0).normal(0, 0.3, (40, 3))
    cs2, st = cp.place(T, cp.load_candidates(spec, cf.ROOT), spec, cfg["place"], PLANE, block, 0.0, 0.006, 0.0077, 0.012, 0.085)
    assert st["scene"] > 0 and len(cs2) < len(cs)


def test_panel_plane_ignores_a_higher_tilted_white_plate():
    cfg, _ = cf.load_config("micro_jack")
    K = np.array([[434.7, 0, 424.0], [0, 434.7, 240.0], [0, 0, 1.0]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.diag([1.0, -1.0, -1.0]), [0.25, 0.02, 0.30]                  # camera 0.3 m above (250, 20), looking down
    u, v = np.meshgrid(np.arange(848), np.arange(480))
    X, Y = (u - K[0, 2]) * 0.3 / K[0, 0] * 1000, -(v - K[1, 2]) * 0.3 / K[1, 1] * 1000     # mm in the plane z ~ 0 (small-angle)
    depth = np.full((480, 848), 0.30, np.float32)
    plate = X > 30                                                                         # a plate 30 mm to the +x side, 15 mm higher and tilted 10 deg
    depth[plate] = 0.30 - (0.015 + 0.176 * (X[plate] - 30) / 1000)
    color = np.full((480, 848, 3), 255, np.uint8)
    cv2.circle(color, (424, 240), 25, (20, 20, 20), -1)                                    # dark connector in the middle
    plane = cf.panel_plane([dict(depth=depth, color=color, K=K, T_BC=T)], np.array([250.0, 20.0]), cfg["panel"], 4.0)
    assert plane["tilt_deg"] < 3 and abs(-(plane["d"]) - 0.0) < 3.0                        # the flat panel at z = 0, not the plate


def test_close_tilted_views_pin_down_the_tilt_far_better_than_near_vertical_ones():
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
    import trimesh
    from scan_observability import evaluate
    cfg, spec = cf.load_config("micro_jack")
    M, N = cf.model_cloud(spec)
    sc = trimesh.load(cf.ROOT / spec["mesh"])
    area = (sc.to_geometry() if hasattr(sc, "to_geometry") else sc).area * 1e6 / 40000
    T = np.eye(4)
    T[:3, 3] = [260.0, -20.0, -spec["seat_u_mm"]]
    Rh, tgt = np.diag([1.0, -1.0, -1.0]), np.array([0.26, -0.02, 0.01])
    close = evaluate(M, N, area, [cf.close_view_pose(tgt, az, 0.20, 35, Rh) for az in (0, 120, 240)], T)
    flat = evaluate(M, N, area, [cf.close_view_pose(tgt, az, 0.31, 8, Rh) for az in (0, 120, 240)], T)
    assert close["sigma_rx"] < 0.5 and flat["sigma_rx"] > 10 * close["sigma_rx"] and close["coverage_pct"] > 3 * flat["coverage_pct"]
