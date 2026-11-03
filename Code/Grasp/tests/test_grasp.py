"""Synthetic-scene tests. They check the geometry, NOT hardware performance."""
import sys, pathlib
import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp import plan_grasp, verify_grasp, cloud_from_depth  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402

L, W, H = 0.045, 0.016, 0.010
CFG = dict(
    housing=dict(length_m=L, width_m=W, height_m=H, grasp_zone_from_tip_m=[0.012, 0.030],
                 length_tol_m=0.006, width_tol_m=0.003, height_tol_m=0.003,
                 min_height_m=0.002, min_points=300, cluster_radius_m=0.004, bin_m=0.002,
                 min_bin_points=5, wide_frac=0.7, cable_probe_m=0.015, min_points_cable=20),
    gripper=dict(max_open_m=0.085, finger_width_m=0.012, finger_thickness_m=0.004,
                 finger_length_m=0.03, finger_tip_below_tcp_m=0.003, close_tol_m=0.002),
    grasp=dict(n_offsets=5, contact_height_frac=0.5, open_margin_m=0.006, housing_slack_m=0.002,
               plane_clearance_m=0.0015, max_collision_points=3, standoff_m=0.06, lift_m=0.08),
)


def box_top_and_sides(c, size, n, rng):
    """Surface samples of an axis-aligned box (top + 4 sides visible from above)."""
    pts = []
    for _ in range(n):
        f = rng.integers(0, 5)
        p = (rng.random(3) - .5) * size
        if f == 0: p[2] = size[2] / 2
        elif f == 1: p[0] = size[0] / 2
        elif f == 2: p[0] = -size[0] / 2
        elif f == 3: p[1] = size[1] / 2
        else: p[1] = -size[1] / 2
        pts.append(p + c)
    return np.array(pts)


def scene(yaw, xy, cable=True, cable_obstacle=False, seed=0):
    rng = np.random.default_rng(seed)
    table = np.c_[rng.uniform(-.12, .12, (6000, 2)), np.zeros(6000)]
    # housing in local frame: center at (0,0,H/2), tip at x=-L/2, cable leaves at +x
    hous = box_top_and_sides(np.array([0, 0, H / 2]), np.array([L, W, H]), 4000, rng)
    parts = [hous]
    if cable:
        th = rng.uniform(0, 2 * np.pi, 1500); xs = rng.uniform(L / 2, L / 2 + 0.10, 1500)
        parts.append(np.c_[xs, 0.0025 * np.sin(th), H / 2 + 0.0025 * np.cos(th)])
    loc = np.vstack(parts)
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    obj = loc @ R.T + [*xy, 0]
    pts = np.vstack([table, obj])
    if cable_obstacle:  # a second cable lying right beside the housing (where a finger goes)
        side = np.c_[rng.uniform(-.02, .02, 500), W / 2 + 0.008 + rng.normal(0, .001, 500), rng.uniform(.002, .008, 500)]
        pts = np.vstack([pts, side @ R.T + [*xy, 0]])
    pts = pts + rng.normal(0, 0.0003, pts.shape)
    d = (pts - [*xy, 0])
    loc_xy = d[:, :2] @ R[:2, :2]
    in_roi = (np.abs(loc_xy[:, 0]) < L / 2 + 0.005) & (np.abs(loc_xy[:, 1]) < W / 2 + 0.005)
    return pts, in_roi, R


CAM = np.array([0, 0, 0.30])


@pytest.mark.parametrize("yaw", [0.0, 0.7, 2.0, -2.5])
def test_pose_and_grasp(yaw):
    xy = (0.03, -0.02)
    pts, roi, R = scene(yaw, xy)
    res = plan_grasp(pts, roi, CAM, CFG)
    assert res["ok"], res["reasons"]
    T = res["T_BH"]
    assert np.linalg.norm(T[:3, 3] - [*xy, H / 2]) < 0.002
    assert abs(T[:3, 0] @ R[:, 0] - 1) < 1e-2          # x: tip -> cable
    g = res["grasps"][0]
    assert abs(g["T_BG"][:3, 2] @ [0, 0, -1] - 1) < 4e-5   # top-down within 0.5 deg of the fitted plane normal
    assert abs(g["T_BG"][:3, 1] @ R[:, 1]) > 0.999          # closes across width
    # TCP inside the approved zone, on the housing centerline
    loc = np.linalg.inv(T) @ np.r_[g["T_BG"][:3, 3], 1]
    tip_dist = loc[0] + L / 2
    assert 0.012 - 1e-6 <= tip_dist <= 0.030 + 1e-6 and abs(loc[1]) < 1e-6
    assert np.allclose(g["T_BG"] @ g["T_G_housing"], T, atol=1e-9)


def test_no_cable_visible_is_rejected():
    pts, roi, _ = scene(0.3, (0, 0), cable=False)
    res = plan_grasp(pts, roi, CAM, CFG)
    assert not res["ok"] and "tip from cable" in res["reasons"][0]


def test_wrong_housing_size_is_rejected():
    pts, roi, _ = scene(0.3, (0, 0))
    bad = {**CFG, "housing": {**CFG["housing"], "width_m": 0.024}}
    assert not plan_grasp(pts, roi, CAM, bad)["ok"]


def test_obstacle_beside_housing_rejects_colliding_grasps():
    pts, roi, _ = scene(0.5, (0, 0), cable_obstacle=True)
    pts0, roi0, _ = scene(0.5, (0, 0))
    free = plan_grasp(pts0, roi0, CAM, CFG)
    blocked = plan_grasp(pts, roi, CAM, CFG)
    assert blocked["rejected"] and len(blocked["grasps"]) < len(free["grasps"])


def test_gripper_too_small():
    pts, roi, _ = scene(0.3, (0, 0))
    small = {**CFG, "gripper": {**CFG["gripper"], "max_open_m": 0.02}}
    assert not plan_grasp(pts, roi, CAM, small)["ok"]


def test_verify_grasp():
    hs = CFG["housing"]
    assert verify_grasp(W + 0.001, hs, 0.002)[0]
    assert not verify_grasp(0.0, hs, 0.002)[0]
    assert not verify_grasp(W + 0.006, hs, 0.002)[0]


def test_cloud_from_depth_roundtrip():
    K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    T_BC = np.eye(4)
    depth = np.full((480, 640), 0.3)
    pb, roi = cloud_from_depth(depth, K, T_BC, (300, 220, 340, 260), margin_px=40)
    assert np.allclose(pb[:, 2], 0.3) and roi.sum() == 40 * 40
    i = np.argmin(np.linalg.norm(pb[:, :2], axis=1))
    assert np.linalg.norm(pb[i, :2]) < 0.001


def test_unmeasured_config_is_refused():
    with pytest.raises(ValueError, match="unmeasured"):
        load_config(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml")
