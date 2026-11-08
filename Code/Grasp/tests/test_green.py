"""Synthetic checks of the green-marker stages (geometry only, not hardware evidence)."""
import pathlib, sys
import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.green import dbscan, green_mask, marker_planes, ransac_plane  # noqa: E402

G = load_config(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"), allow_unscanned=True)["green"]


def test_mask_takes_head_green_not_wood_or_white():
    img = np.zeros((40, 40, 3), np.uint8)
    img[:] = (60, 110, 190)                    # wood, BGR
    img[5:15, 5:15] = (200, 200, 20)           # teal/green (H~90, S high)
    img[25:35, 25:35] = (250, 250, 245)        # white
    m = green_mask(img, G)
    assert m[5:15, 5:15].mean() > 0.9 and not m[25:35, 25:35].any() and not m[20:24, 0:4].any()


def test_ransac_finds_tilted_table_and_ignores_object():
    rng = np.random.default_rng(1)
    xy = rng.uniform(-0.15, 0.15, (20000, 2))
    z = 0.05 * xy[:, 0] + rng.normal(0, 0.0005, 20000)
    obj = np.c_[rng.uniform(-0.01, 0.01, (2000, 2)), rng.uniform(0.01, 0.03, 2000)]
    n, d, _ = ransac_plane(np.vstack([np.c_[xy, z], obj]), 0.003)
    assert n[2] > 0.99 and abs(np.degrees(np.arccos(n[2])) - np.degrees(np.arctan(0.05))) < 0.3


def test_two_marker_plates_split_with_outward_normals():
    rng = np.random.default_rng(2)
    plate = lambda y: np.c_[rng.uniform(-0.01, 0.01, 1500), rng.normal(y, 0.0002, 1500), rng.uniform(0.0, 0.007, 1500)]
    (P1, c1, n1), (P2, c2, n2) = marker_planes(np.vstack([plate(-0.00885), plate(0.00885)]), G)
    lo, hi = sorted([(c1, n1), (c2, n2)], key=lambda t: t[0][1])
    assert abs(hi[0][1] - lo[0][1] - 0.0177) < 0.001
    assert lo[1][1] < -0.95 and hi[1][1] > 0.95    # normals point away from each other


def test_dbscan_separates_blobs():
    rng = np.random.default_rng(3)
    P = np.vstack([rng.normal(0, 0.001, (200, 3)), rng.normal(0.05, 0.001, (200, 3))])
    assert len(set(dbscan(P, 0.006, 15)) - {-1}) == 2


def test_green_frame_of_a_flat_patch():
    from housing_grasp.green import green_frame, largest_blob
    rng = np.random.default_rng(4)
    C = np.c_[rng.uniform(-0.015, 0.015, 3000), rng.uniform(-0.006, 0.006, 3000), rng.normal(0.01, 0.0001, 3000)]
    f = green_frame(C)
    assert abs(f["normal"][2]) > 0.99 and abs(abs(f["long_axis"][0]) - 1) < 0.02 and abs(f["length_m"] - 0.03) < 0.002 and abs(f["width_m"] - 0.012) < 0.002
    m = np.zeros((50, 50), bool); m[10:20, 30:40] = True; m[40:42, 5:7] = True
    assert largest_blob(m, 20) == (30, 10, 40, 20) and largest_blob(m, 500) is None


def _synthetic_head(tilt_z=0.0):
    """Camera 0.30 m above the table looking down; head 25 mm (x) x 13 mm (y) with top face at z = 15 mm, a teal sticker on top, dark body around it."""
    K = np.array([[434.7, 0, 424.0], [0, 434.7, 240.0], [0, 0, 1.0]])
    T = np.eye(4); T[:3, :3] = np.diag([1.0, -1.0, -1.0]); T[:3, 3] = [0.3, 0.0, 0.30]
    img = np.zeros((480, 848, 3), np.uint8); img[:] = (60, 110, 190)
    depth = np.full((480, 848), 0.30, np.float32)
    zc = 0.30 - 0.015
    hw, hh = round(12.5e-3 * K[0, 0] / zc), round(6.5e-3 * K[0, 0] / zc)
    img[240 - hh:240 + hh, 424 - hw:424 + hw] = (25, 25, 25)
    depth[240 - hh:240 + hh, 424 - hw:424 + hw] = zc
    img[240 - hh + 3:240 + hh - 3, 424 - 8:424 + 8] = (200, 200, 20)
    return img, depth, K, T


def test_image_mode_finds_jaw_axis_width_and_height_of_a_synthetic_head():
    from housing_grasp.green import green_points, head_points, synthesize_head_grasps
    img, depth, K, T = _synthetic_head()
    G_ = green_points(depth, img, K, T, G)
    c = np.median(G_, 0)
    P = head_points(depth, img, K, T, c, G)
    cands = synthesize_head_grasps(P, G_, c, G, 0.006, 0.085, 0.012, 0.008)
    assert len(P) > len(G_) and cands
    b = cands[0]
    assert abs(b["jaw"][1]) > 0.97                                  # jaw closes across the short side (y)
    assert abs(b["width_m"] - 0.013) < 0.0015 and abs(b["open_width_m"] - 0.037) < 0.0015
    assert abs(b["p"][2] - (0.015 - 0.008)) < 0.0015                # TCP 8 mm below the sticker/top face
    assert [q["cost"] for q in cands] == sorted(q["cost"] for q in cands)
    assert all(abs(q["p"][2] - b["p"][2]) <= G["max_z_spread_m"] for q in cands)


def test_image_mode_drops_candidates_where_the_height_changes():
    from housing_grasp.green import synthesize_head_grasps
    rng = np.random.default_rng(5)
    P = np.c_[rng.uniform(-0.012, 0.012, 4000), rng.uniform(-0.0065, 0.0065, 4000), np.zeros(4000)]
    P[:, 2] = np.where(P[:, 0] > 0.0, 0.015, 0.015 + 0.02 * (-P[:, 0] / 0.012))   # flat for x > 0, ramp for x < 0
    c = np.array([0.003, 0.0, 0.015])
    drop = synthesize_head_grasps(P, P, c, dict(G, head_height_rule="drop", yaw_steps_deg=[0]), 0.006, 0.085, 0.012, 0.008)
    keep = synthesize_head_grasps(P, P, c, dict(G, head_height_rule="penalize", yaw_steps_deg=[0]), 0.006, 0.085, 0.012, 0.008)
    extra = [q for q in keep if all(abs(q["offset_m"] - d["offset_m"]) > 1e-9 for d in drop)]   # the PCA axis sign is arbitrary: do not assume which side is the ramp
    assert len(keep) == len(drop) + 1 and len(extra) == 1                                      # drop removes the ramp position, penalize keeps it in the search ...
    assert extra[0]["p"][2] > max(q["p"][2] for q in drop) + 0.003 and extra[0]["cost"] > max(q["cost"] for q in drop)   # ... higher, and ranked below every flat position


def _sticker_image(angle_deg=0.0):
    """Camera 0.30 m above the table looking down; flat teal sticker 12.5 mm (long, along base y) x 9.5 mm (along x) at z = 15 mm, rotated by angle_deg."""
    K = np.array([[434.7, 0, 424.0], [0, 434.7, 240.0], [0, 0, 1.0]])
    T = np.eye(4); T[:3, :3] = np.diag([1.0, -1.0, -1.0]); T[:3, 3] = [0.3, 0.0, 0.30]
    img = np.zeros((480, 848, 3), np.uint8); img[:] = (60, 110, 190)
    depth = np.full((480, 848), 0.30, np.float32)
    zc = 0.285
    half = np.array([4.75e-3, 6.25e-3]) * K[0, 0] / zc                       # (x, y) half sizes in px
    a = np.radians(angle_deg)
    R2 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    poly = (np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half) @ R2.T + [424, 240]
    cv2.fillPoly(img, [poly.astype(np.int32)], (200, 200, 20))
    m = np.zeros((480, 848), np.uint8); cv2.fillPoly(m, [poly.astype(np.int32)], 1)
    depth[m > 0] = zc
    return img, depth, K, T


def test_cloud_mode_axes_come_from_the_image_edges_and_candidates_are_searched():
    from housing_grasp.green import green_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image()
    G_ = green_points(depth, img, K, T, G); c = np.median(G_, 0)
    rect = sticker_rect(img, K, T, c, G)
    assert abs(rect["long_m"] - 0.0125) < 0.0012 and abs(rect["short_m"] - 0.0095) < 0.0012 and abs(rect["long_axis"][1]) > 0.98
    cands = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008)
    assert len(cands) > 10 and [q["cost"] for q in cands] == sorted(q["cost"] for q in cands)
    b = cands[0]
    assert b["edge"] == G["jaw_edge"] == "long" and abs(b["jaw"][1]) > 0.98 and abs(b["open_width_m"] - (0.0125 + 0.024)) < 0.0015
    assert abs(b["p"][2] - (0.015 - 0.008)) < 0.0015 and {q["edge"] for q in cands} == {"long", "short"} and len({q["yaw_deg"] for q in cands}) == len(G["yaw_steps_deg"])


def test_cloud_mode_follows_a_rotated_sticker():
    from housing_grasp.green import green_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image(angle_deg=30.0)
    G_ = green_points(depth, img, K, T, G); c = np.median(G_, 0)
    rect = sticker_rect(img, K, T, c, G)
    ang = np.degrees(np.arccos(min(1.0, abs(rect["long_axis"] @ np.array([0.0, 1.0])))))   # long edge was along y before the rotation
    assert abs(ang - 30.0) < 4.0
    b = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008)[0]
    assert abs(np.degrees(np.arccos(min(1.0, abs(b["jaw"][:2] @ rect["long_axis"])))) ) < 1.0


def test_candidate_search_switch_off_gives_one_closed_form_grasp_on_gives_many():
    from housing_grasp.green import green_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image()
    G_ = green_points(depth, img, K, T, G); c = np.median(G_, 0); rect = sticker_rect(img, K, T, c, G)
    on = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008, search=True)
    off = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008, search=False)
    assert len(off) == 1 and len(on) > 10
    assert off[0]["edge"] == G["jaw_edge"] and off[0]["yaw_deg"] == 0 and off[0]["offset_m"] == 0
    assert np.allclose(off[0]["p"], on[0]["p"], atol=1e-9) and np.allclose(off[0]["jaw"], on[0]["jaw"])   # the single grasp is the top of the searched list


def test_contact_faces_rule_chooses_which_pair_of_faces_the_pads_touch():
    from housing_grasp.green import sticker_candidates
    rect = dict(long_axis=np.array([0.0, 1.0]), short_axis=np.array([1.0, 0.0]), long_m=0.0247, short_m=0.0134, center_xy=np.zeros(2))
    c = np.array([0, 0, 0.01])
    pick = lambda faces, edge="long": sticker_candidates(rect, c, dict(G, contact_faces=faces, jaw_edge=edge), 0.006, 0.085, 0.012, 0.008, search=False)[0]
    larger, smaller = pick("larger"), pick("smaller")
    assert abs(larger["jaw"][0]) > 0.99 and abs(larger["face_width_m"] - 0.0247) < 1e-9     # jaw along the SHORT extent: pads on the 24.7 mm wide faces
    assert abs(smaller["jaw"][1]) > 0.99 and abs(smaller["face_width_m"] - 0.0134) < 1e-9
    assert pick("edge_prior", "long")["edge"] == "long" and pick("edge_prior", "short")["edge"] == "short"


def test_head_mode_search_switch():
    from housing_grasp.green import green_points, head_points, synthesize_head_grasps
    img, depth, K, T = _synthetic_head()
    G_ = green_points(depth, img, K, T, G); c = np.median(G_, 0); P = head_points(depth, img, K, T, c, G)
    assert len(synthesize_head_grasps(P, G_, c, G, 0.006, 0.085, 0.012, 0.008, search=False)) == 1
    assert len(synthesize_head_grasps(P, G_, c, G, 0.006, 0.085, 0.012, 0.008, search=True)) > 1


def test_head_mode_search_includes_yaw_and_off_gives_one():
    from housing_grasp.green import green_points, head_points, synthesize_head_grasps
    img, depth, K, T = _synthetic_head()
    G_ = green_points(depth, img, K, T, G); c = np.median(G_, 0); P = head_points(depth, img, K, T, c, G)
    on = synthesize_head_grasps(P, G_, c, G, 0.006, 0.085, 0.012, 0.008, search=True)
    assert {q["yaw_deg"] for q in on} == set(float(y) for y in G["yaw_steps_deg"]) and len(on) >= 10
    assert on[0]["yaw_deg"] == 0 and [q["cost"] for q in on] == sorted(q["cost"] for q in on)
