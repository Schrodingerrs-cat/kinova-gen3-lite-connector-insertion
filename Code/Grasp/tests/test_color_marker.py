"""Synthetic checks of the colour-marker stages (geometry only, not hardware evidence)."""
import pathlib, sys
import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.color_marker import dbscan, marker_mask, marker_planes, ransac_plane  # noqa: E402

G = load_config(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"), allow_unscanned=True)["color_marker"]


def test_mask_takes_the_marker_colour_not_wood_or_white():
    img = np.zeros((40, 40, 3), np.uint8)
    img[:] = (60, 110, 190)                    # wood, BGR
    img[5:15, 5:15] = (200, 200, 20)           # a teal marker colour (H~90, S high)
    img[25:35, 25:35] = (250, 250, 245)        # white
    m = marker_mask(img, G)
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


def test_marker_frame_of_a_flat_patch():
    from housing_grasp.color_marker import marker_frame, largest_blob
    rng = np.random.default_rng(4)
    C = np.c_[rng.uniform(-0.015, 0.015, 3000), rng.uniform(-0.006, 0.006, 3000), rng.normal(0.01, 0.0001, 3000)]
    f = marker_frame(C)
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


def _grid_inputs():
    from housing_grasp.color_marker import marker_points, head_points, scene_points, outline_normals
    img, depth, K, T = _synthetic_head()
    G_ = marker_points(depth, img, K, T, G); c = np.median(G_, 0)
    P = head_points(depth, img, K, T, c, G)
    return P, outline_normals(P), scene_points(depth, img, K, T, c, G), G_, c


def test_image_mode_grid_search_finds_jaw_axis_width_and_height_of_a_synthetic_head():
    from housing_grasp.color_marker import grid_grasp_search
    P, N, S, G_, c = _grid_inputs()
    cands, st = grid_grasp_search(P, N, S, G_, c, G, 0.006, 0.0077, 0.085, 0.012, 0.008)
    assert st["generated"] > 1000 and len(cands) > 100 and st["valid"] == len(cands)
    b = cands[0]
    assert abs(b["jaw"][1]) > 0.97                                  # jaw closes across the short side (y)
    assert abs(b["width_m"] - 0.013) < 0.0015 and abs(b["open_width_m"] - 0.037) < 0.0015
    assert b["depth_extra_m"] in G["extra_depth_m"] and abs(b["p"][2] - (0.015 - 0.008 - b["depth_extra_m"])) < 0.0015   # TCP = top - drop - extra depth
    assert st["depth_levels"] == len(G["extra_depth_m"]) and {q["depth_extra_m"] for q in cands} <= set(G["extra_depth_m"])
    assert b["offset_m"] < 0.003                                    # centred on the sticker
    assert b["face_len_m"] > 0.020                                  # the pads land on the long walls (25 mm), not the 13 mm ends
    assert [q["cost"] for q in cands] == sorted(q["cost"] for q in cands)


def test_grid_search_jaw_yaw_restricted_to_home_and_plus_minus_90():
    from housing_grasp.color_marker import grid_grasp_search
    P, N, S, G_, c = _grid_inputs()
    ref = np.radians(85.5)
    cands, st = grid_grasp_search(P, N, S, G_, c, dict(G, jaw_yaw_offsets_deg=[0, 90]), 0.006, 0.0077, 0.085, 0.012, 0.008, ref)
    yaws = {round(float(q["yaw_deg"]), 3) for q in cands}
    assert st["yaws"] == 2 and yaws <= {85.5, 175.5} and len(yaws) >= 1


def test_grid_search_finger_clearance_removes_deep_candidates_close_to_the_table():
    from housing_grasp.color_marker import grid_grasp_search
    P, N, S, G_, c = _grid_inputs()
    base, _ = grid_grasp_search(P, N, S, G_, c, G, 0.006, 0.0077, 0.085, 0.012, 0.008)
    far, st = grid_grasp_search(P, N, S, G_, c, dict(G, finger_clearance_m=0.02), 0.006, 0.0077, 0.085, 0.012, 0.008)   # tips would need to stay 20 mm above the table
    assert len(base) > 0 and far == [] and st["collision"] > 0


def test_grid_search_finger_collision_removes_candidates_with_scene_points_in_a_finger_box():
    from housing_grasp.color_marker import grid_grasp_search
    P, N, S, G_, c = _grid_inputs()
    ok, _ = grid_grasp_search(P, N, S, G_, c, G, 0.006, 0.0077, 0.085, 0.012, 0.008)
    b = ok[0]
    wall = b["p"][:2] + b["jaw"][:2] * (b["open_width_m"] / 2 + 0.003)               # a block right where one finger would close
    blk = np.c_[wall[0] + np.zeros(60), wall[1] + np.zeros(60), np.linspace(b["p"][2], b["p"][2] + 0.01, 60)]
    hit, st = grid_grasp_search(P, N, np.vstack([S, blk]), G_, c, G, 0.006, 0.0077, 0.085, 0.012, 0.008)
    assert st["collision"] > 0 and len(hit) < len(ok)


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
    from housing_grasp.color_marker import marker_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image()
    G_ = marker_points(depth, img, K, T, G); c = np.median(G_, 0)
    rect = sticker_rect(img, K, T, c, G)
    assert abs(rect["long_m"] - 0.0125) < 0.0012 and abs(rect["short_m"] - 0.0095) < 0.0012 and abs(rect["long_axis"][1]) > 0.98
    cands = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008)
    assert len(cands) > 10 and [q["cost"] for q in cands] == sorted(q["cost"] for q in cands)
    b = cands[0]
    assert b["edge"] == G["jaw_edge"] == "long" and abs(b["jaw"][1]) > 0.98 and abs(b["open_width_m"] - (0.0125 + 0.024)) < 0.0015
    assert abs(b["p"][2] - (0.015 - 0.008)) < 0.0015 and {q["edge"] for q in cands} == {"long", "short"} and len({q["yaw_deg"] for q in cands}) == len(G["yaw_steps_deg"])


def test_cloud_mode_follows_a_rotated_sticker():
    from housing_grasp.color_marker import marker_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image(angle_deg=30.0)
    G_ = marker_points(depth, img, K, T, G); c = np.median(G_, 0)
    rect = sticker_rect(img, K, T, c, G)
    ang = np.degrees(np.arccos(min(1.0, abs(rect["long_axis"] @ np.array([0.0, 1.0])))))   # long edge was along y before the rotation
    assert abs(ang - 30.0) < 4.0
    b = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008)[0]
    assert abs(np.degrees(np.arccos(min(1.0, abs(b["jaw"][:2] @ rect["long_axis"])))) ) < 1.0


def test_candidate_search_switch_off_gives_one_closed_form_grasp_on_gives_many():
    from housing_grasp.color_marker import marker_points, sticker_candidates, sticker_rect
    img, depth, K, T = _sticker_image()
    G_ = marker_points(depth, img, K, T, G); c = np.median(G_, 0); rect = sticker_rect(img, K, T, c, G)
    on = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008, search=True)
    off = sticker_candidates(rect, c, G, 0.006, 0.085, 0.012, 0.008, search=False)
    assert len(off) == 1 and len(on) > 10
    assert off[0]["edge"] == G["jaw_edge"] and off[0]["yaw_deg"] == 0 and off[0]["offset_m"] == 0
    assert np.allclose(off[0]["p"], on[0]["p"], atol=1e-9) and np.allclose(off[0]["jaw"], on[0]["jaw"])   # the single grasp is the top of the searched list


def test_contact_faces_rule_chooses_which_pair_of_faces_the_pads_touch():
    from housing_grasp.color_marker import sticker_candidates
    rect = dict(long_axis=np.array([0.0, 1.0]), short_axis=np.array([1.0, 0.0]), long_m=0.0247, short_m=0.0134, center_xy=np.zeros(2))
    c = np.array([0, 0, 0.01])
    pick = lambda faces, edge="long": sticker_candidates(rect, c, dict(G, contact_faces=faces, jaw_edge=edge), 0.006, 0.085, 0.012, 0.008, search=False)[0]
    larger, smaller = pick("larger"), pick("smaller")
    assert abs(larger["jaw"][0]) > 0.99 and abs(larger["face_width_m"] - 0.0247) < 1e-9     # jaw along the SHORT extent: pads on the 24.7 mm wide faces
    assert abs(smaller["jaw"][1]) > 0.99 and abs(smaller["face_width_m"] - 0.0134) < 1e-9
    assert pick("edge_prior", "long")["edge"] == "long" and pick("edge_prior", "short")["edge"] == "short"


def test_marker_colour_is_a_setting_including_a_hue_range_that_wraps_for_red():
    from housing_grasp.color_marker import marker_mask
    img = np.zeros((40, 40, 3), np.uint8); img[:] = (60, 110, 190)          # wood
    img[5:15, 5:15] = (30, 30, 220)                                          # red (hue ~0)
    img[5:15, 25:35] = (220, 40, 40)                                         # blue (hue ~120)
    red = marker_mask(img, dict(G, h=[170, 10]))                             # wraps past 179
    assert red[5:15, 5:15].mean() > 0.9 and not red[5:15, 25:35].any() and not red[20:24, 0:4].any()
    blue = marker_mask(img, dict(G, h=[110, 130]))
    assert blue[5:15, 25:35].mean() > 0.9 and not blue[5:15, 5:15].any()
    assert marker_mask(img, G)[5:15, 5:15].sum() == 0                        # the default (teal) does not pick red or blue


def test_nearest_blob_drops_a_larger_marker_coloured_strip():
    from housing_grasp.color_marker import nearest_blob
    K = np.array([[434.7, 0, 424.0], [0, 434.7, 240.0], [0, 0, 1.0]])
    T = np.eye(4); T[:3, :3] = np.diag([1.0, -1.0, -1.0]); T[:3, 3] = [0.3, 0.0, 0.30]   # camera above (0.3, 0), looking down
    m = np.zeros((480, 848), bool)
    m[230:250, 414:434] = True      # sticker blob at the image centre
    m[100:130, 500:800] = True      # long strip, much larger, elsewhere
    keep = nearest_blob(m, K, T, np.array([0.3, 0.0, 0.0]))
    assert keep[230:250, 414:434].all() and not keep[100:130, 500:800].any()
    assert nearest_blob(m, K, T, np.array([0.3, 0.0, 0.0]), min_px=10000).sum() == 0     # nothing big enough: empty, no crash


def test_scan_target_is_the_sticker_sized_marker_cluster_not_the_largest_strip():
    from housing_grasp.config import load_config
    from housing_grasp.color_marker import analyze_scan
    cfg = load_config(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"), allow_unscanned=True)
    tz = cfg["scan"]["table_z_m"]
    rng = np.random.default_rng(0)
    table = np.c_[rng.uniform(0.2, 0.4, 40000), rng.uniform(-0.1, 0.1, 40000), np.full(40000, tz) + rng.normal(0, 0.0003, 40000)]
    gx, gy = np.meshgrid(np.arange(0, 0.012, 0.001), np.arange(0, 0.012, 0.001))
    sticker = np.c_[0.30 + gx.ravel(), 0.00 + gy.ravel(), np.full(gx.size, tz + 0.02)]            # 12 x 12 mm patch, 20 mm above the table
    sx = np.arange(0, 0.09, 0.001)
    strip = np.c_[0.25 + sx, np.full(sx.size, 0.05), np.full(sx.size, tz + 0.015)]                 # 90 mm long strip: more points than the sticker if dense
    strip = np.vstack([strip + [0, dy, 0] for dy in np.arange(0, 0.008, 0.001)])
    G = np.vstack([sticker, strip])
    r = analyze_scan(np.vstack([table, G]), G, cfg)
    assert len(r["marker_clusters"][0]) == len(sticker) or abs(len(r["marker_clusters"][0]) - len(sticker)) < 10   # first cluster is the sticker though the strip has more points
    assert len(strip) > len(sticker) and abs(r["frame"]["centroid"][0] - 0.3055) < 0.01 and r["n_sticker_sized"] == 1


def test_wide_face_offset_follows_the_sticker_plane_normal():
    from housing_grasp.color_marker import wide_face_offset
    az, tilt = np.radians(10.0), np.radians(74.0)                    # plane normal: 10 deg azimuth, 74 deg from horizontal
    n = np.array([np.cos(az) * np.sin(tilt), np.sin(az) * np.sin(tilt), np.cos(tilt)])
    u = np.cross(n, [0, 0, 1.0]); u /= np.linalg.norm(u); v = np.cross(n, u)
    rng = np.random.default_rng(1)
    pts = rng.uniform(-0.006, 0.006, (200, 1)) * u + rng.uniform(-0.006, 0.006, (200, 1)) * v + rng.normal(0, 2e-4, (200, 1)) * n
    off, a, t = wide_face_offset(pts, np.radians(85.5), [0, 90])
    assert off == 90.0 and abs(a - 10.0) < 3 and abs(t - 74.0) < 3    # normal at ~10 deg = 75 deg from the home jaw axis -> the +-90 family is the wider faces
    off0, _, _ = wide_face_offset(pts, np.radians(10.0), [0, 90])      # jaw axis already along the normal: offset 0 is the wider faces
    assert off0 == 0.0
    flat = rng.uniform(-0.006, 0.006, (200, 3)) * [1, 1, 0.01]          # a horizontal patch: no trust
    assert wide_face_offset(flat, 1.0, [0, 90])[0] is None

