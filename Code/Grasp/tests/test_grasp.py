"""Synthetic-scene tests. They check the geometry, NOT hardware performance."""
import pathlib, sys
import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp import cloud_from_depth, grasp_held  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.gripper import measure_fingers  # noqa: E402
from housing_grasp.scan import fuse_clouds, view_poses  # noqa: E402

SIZES = [(0.045, 0.016, 0.010), (0.025, 0.0125, 0.0075), (0.060, 0.022, 0.015),
         (0.020, 0.007, 0.007)]  # last = AUX-jack-like head; none known to the code
CFG = dict(
    cloud=dict(voxel_m=0.001, outlier_radius_m=0.003, outlier_min_nb=4, edge_jump_m=0.003),
    housing=dict(plane_ring_m=0.06, table_gap_m=0.010, crop_radius_m=0.06, min_height_m=0.002, max_height_m=0.04,
                 core_frac=0.5, min_points=40, cluster_radius_m=0.0025, bin_m=0.0015,
                 min_bin_points=3, thick_ratio=1.4, min_length_m=0.010, cable_probe_m=0.012,
                 min_points_cable=15, tip_keepout_m=0.003, cable_end_keepout_frac=0.3,
                 ambiguous_keepout_frac=0.2),
    gripper=dict(max_open_m=0.085, finger_width_m=0.008, finger_thickness_m=0.004,
                 finger_length_m=0.03, finger_tip_below_tcp_m=0.0),
    heads=dict(cell_m=0.003, ground_window_m=0.15, ground_percentile=15, min_head_height_m=0.008, max_head_height_m=0.04,
               min_area_m2=0.00015, max_area_m2=0.006, region=[-0.2, 0.2, -0.2, 0.2]),
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


def scene(yaw, xy, dims=SIZES[0], cable=True, cable_obstacle=False, seed=0):
    L, W, H = dims
    rng = np.random.default_rng(seed)
    table = np.c_[rng.uniform(-.10, .10, (50000, 2)), np.zeros(50000)]
    # housing in local frame: center at (0,0,H/2), tip at x=-L/2, cable leaves at +x
    hous = box_top_and_sides(np.array([0, 0, H / 2]), np.array([L, W, H]), 25000, rng)
    parts = [hous]
    if cable:
        r = min(0.0025, 0.24 * W); th = rng.uniform(0, 2 * np.pi, 5000); xs = rng.uniform(L / 2, L / 2 + 0.10, 5000)
        parts.append(np.c_[xs, r * np.sin(th), H / 2 + r * np.cos(th)])
    loc = np.vstack(parts)
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    obj = loc @ R.T + [*xy, 0]
    pts = np.vstack([table, obj])
    if cable_obstacle:  # a second cable lying right beside the housing (where a finger goes)
        side = np.c_[rng.uniform(-.02, .02, 4000), W / 2 + 0.008 + rng.normal(0, .001, 4000), rng.uniform(.002, .008, 4000)]
        pts = np.vstack([pts, side @ R.T + [*xy, 0]])
    pts = pts + rng.normal(0, 0.0003, pts.shape)
    d = (pts - [*xy, 0])
    loc_xy = d[:, :2] @ R[:2, :2]
    in_roi = (np.abs(loc_xy[:, 0]) < L / 2 + 0.005) & (np.abs(loc_xy[:, 1]) < W / 2 + 0.005)
    return pts, in_roi, R


CAM = np.array([0, 0, 0.30])


def test_grasp_held():
    gp = dict(close_cmd=0.81, empty_frac=0.975)
    assert grasp_held(0.50, gp)          # stopped on an object
    assert not grasp_held(0.80, gp)      # fully closed: nothing (or only thin prongs)


def test_cloud_from_depth_roundtrip():
    K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    T_BC = np.eye(4)
    depth = np.full((480, 640), 0.3)
    pb, roi = cloud_from_depth(depth, K, T_BC, (300, 220, 340, 260), margin_px=40)
    assert np.allclose(pb[:, 2], 0.3) and 300 < roi.sum() <= 20 * 20  # central half of the box
    i = np.argmin(np.linalg.norm(pb[:, :2], axis=1))
    assert np.linalg.norm(pb[i, :2]) < 0.001


def test_unmeasured_config_is_refused(tmp_path):
    import yaml
    c = yaml.safe_load(open(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"))
    c["gripper"]["finger_width_m"] = None
    (tmp_path / "c.yaml").write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError, match="unmeasured"):
        load_config(tmp_path / "c.yaml")


def test_fusion_recovers_height_error_between_views():
    pts, _, _ = scene(0.4, (0, 0), SIZES[0])
    rng = np.random.default_rng(1)
    a = pts[rng.random(len(pts)) < 0.6]
    b = pts[rng.random(len(pts)) < 0.6]
    b_bad = b + [0, 0, 0.002]                           # view b is 2 mm too high (observable on the table)
    fused, info = fuse_clouds([a, b_bad])
    assert abs(info[1]["shift_mm"] - 2.0) < 0.3 and info[1]["rms_after_mm"] < info[1]["rms_before_mm"]


def test_view_poses_place_the_camera_and_tilt_toward_the_centre():
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])     # tool pointing down
    T0 = np.eye(4); T0[:3, :3] = R
    X = np.eye(4); X[:3, 3] = [-0.008, -0.05, -0.09]         # camera behind the tool origin, optical axis = tool z
    sc = dict(table_z_m=-0.04, look_at=[0.4, 0.0], views=[[0.4, 0.1, 0.3], [0.2, 0.2, 0.25, 30]])
    down, obl = view_poses(sc, T0, X)
    assert np.allclose(down[:3, 3] + down[:3, :3] @ X[:3, 3], [0.4, 0.1, 0.26])   # camera 0.30 m over the table
    assert np.allclose(obl[:3, 3] + obl[:3, :3] @ X[:3, 3], [0.2, 0.2, 0.21])
    axis = obl[:3, :3] @ X[:3, :3] @ [0, 0, 1.0]            # optical axis of the oblique view
    to = np.array([0.2, -0.2, 0]) / np.hypot(0.2, 0.2)      # horizontal direction toward the centre
    assert abs(np.degrees(np.arccos(-axis[2])) - 30) < 0.5 and axis @ to > 0.4


def test_gripper_scan_measures_known_fingers():
    rng = np.random.default_rng(0)
    def finger(sign, inner, thick=0.008, width=0.012):  # tool frame, tip at z=0.035, 25 mm visible
        p = rng.uniform([0, -width / 2, 0.010], [thick, width / 2, 0.035], (3000, 3))
        p[:, 0] = sign * (inner + p[:, 0])
        return p
    m = measure_fingers(np.vstack([finger(-1, 0.020), finger(1, 0.020)]))
    assert abs(m["gap_m"] - 0.040) < 0.001 and abs(m["thickness_m"] - 0.008) < 0.001
    assert abs(m["width_m"] - 0.012) < 0.001 and abs(m["tip_z_m"] - 0.035) < 0.001


OPT = dict(weights=dict(contact=1, insertion=1, clearance=1, reach=1), flat_scale=0.05, protrusion_target_frac=0.3,
           min_protrusion_m=0.003, clearance_margins_m=[0, 0.002, 0.004], sigma_axis_m=0.002, sigma_lateral_m=0.002,
           sigma_yaw_deg=2.0, n_samples=40, n_shortlist=6, kappa=1.0, max_fail_rate=0.1, seed=0)


