"""MODEL mode (STEP/STL model + ArUco tag on the panel). Its own module and config (config/panel_map.yaml): the head and cloud modes never use it.
Chain: tag pose in the close image -> plug pose (stored offset from the tag, the 'virtual map' of the panel) -> candidates of the offline model search
(housing_grasp/model_grasp.py, in the plug frame) placed in the base frame -> executed like any other mode."""
import cv2
import numpy as np
import yaml

HALF_THICKNESS_M = 0.00375   # plug is 7.5 mm thick (STEP): the plug axis is this far behind a wide face


def load_panel_map(path):
    m = yaml.safe_load(open(path))
    off = m["plugs"][m["plug"]]["T_tag_plug"]
    missing = [k for k, v in (("tag.size_m", m["tag"]["size_m"]), ("T_tag_plug.xy_m", off["xy_m"]), ("T_tag_plug.yaw_deg", off["yaw_deg"])) if v is None]
    if missing:
        raise RuntimeError("panel map not measured: %s in %s (print + measure the tag, then tools/calibrate_panel_map.py)" % (", ".join(missing), path))
    return m


def detect_tag_pose(bgr, K, T_BC, tag):
    """Pose of the square ArUco tag in the base frame (x right, y up in the tag image, z out of the panel). Raises if the tag is not seen."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, tag["dictionary"]))
    prm = cv2.aruco.DetectorParameters()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # integer-pixel corners give ~2 % size error = mm of height at this range
    corners, ids, _ = cv2.aruco.ArucoDetector(d, prm).detectMarkers(bgr)
    if ids is None or tag["id"] not in ids.ravel():
        raise RuntimeError("panel tag id %d not seen in the close view (seen: %s)" % (tag["id"], None if ids is None else ids.ravel().tolist()))
    c = corners[list(ids.ravel()).index(tag["id"])].reshape(4, 2).astype(np.float64)
    h = tag["size_m"] / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    _, rvs, tvs, errs = cv2.solvePnPGeneric(obj, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best = None
    for rv, tv, e in zip(rvs, tvs, errs):
        T_ct = np.eye(4)
        T_ct[:3, :3], T_ct[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
        T = T_BC @ T_ct
        if T[2, 2] > 0.5 and (best is None or float(np.ravel(e)[0]) < best[1]):   # the panel is horizontal: the tag normal must point up
            best = (T, float(np.ravel(e)[0]))
    if best is None:
        raise RuntimeError("tag pose: no solution with the tag normal pointing up (panel not horizontal?)")
    return dict(T=best[0], reproj_px=best[1], side_px=float(np.linalg.norm(c[0] - c[1])), corners_px=c)


def tag_plane_height(depth, K, T_BC, corners_px, min_points=200):
    """Panel surface height (base z) from the depth image over the tag area (flat matte paper on the panel): median of the back-projected pixels; None if too few. The tag's own
    scale gives only ~mm accuracy in height, the depth image ~1 mm."""
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(corners_px).astype(np.int32), 1)
    vv, uu = np.nonzero((mask > 0) & np.isfinite(depth) & (depth > 0))
    if len(vv) < min_points:
        return None
    z = depth[vv, uu].astype(float)
    pb = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1) @ T_BC[:3, :3].T + T_BC[:3, 3]
    return float(np.median(pb[:, 2]))


def plug_pose(tag_T, off):
    """Plug frame in the base frame from the stored offset: dict(xy, yaw (rad, of the plug x axis = across the wide face), z (panel surface))."""
    yaw_tag = float(np.arctan2(tag_T[1, 0], tag_T[0, 0]))
    c, s = np.cos(yaw_tag), np.sin(yaw_tag)
    dx, dy = off["xy_m"]
    return dict(xy=tag_T[:2, 3] + np.array([c * dx - s * dy, s * dx + c * dy]), yaw=yaw_tag + np.radians(off["yaw_deg"]), z=float(tag_T[2, 3] + off.get("z_m", 0.0)))


def plug_pose_from_sticker(G, cam_pos):
    """Plug pose from the sticker's 3D points (base frame): the sticker is on a wide face, so the horizontal direction of the plane normal is the plug y axis
    (across the 7.5 mm thickness) and the plug axis lies HALF_THICKNESS_M behind the sticker. Returns dict(xy, yaw (rad, mod pi), tilt_deg)."""
    c = G.mean(0)
    n = np.linalg.svd(G - c, full_matrices=False)[2][2]
    n = n if n @ (cam_pos - c) > 0 else -n   # outward: towards the camera
    return dict(xy=c[:2] - n[:2] * HALF_THICKNESS_M, yaw=float((np.arctan2(n[1], n[0]) - np.pi / 2) % np.pi), tilt_deg=float(np.degrees(np.arccos(abs(n[2])))))


def place_candidates(cands, plug, margin, max_open, n=None):
    """Candidates of the model search (plug frame) -> base-frame grasps (best first, as given). p = fingertip point (TCP), jaw = horizontal jaw axis."""
    cy, sy = np.cos(plug["yaw"]), np.sin(plug["yaw"])
    Rz = np.array([[cy, -sy], [sy, cy]])
    out = []
    for q in cands[:n]:
        r = np.radians(q["yaw_deg"])
        b, a = np.array([np.cos(r), np.sin(r)]), np.array([-np.sin(r), np.cos(r)])
        ow = q["gap_mm"] * 1e-3 + 2 * margin
        if ow > max_open:
            continue
        out.append(dict(p=np.r_[plug["xy"] + Rz @ (b * q["mid_mm"] * 1e-3 + a * q["s_mm"] * 1e-3), plug["z"] + q["u0_mm"] * 1e-3], jaw=np.r_[Rz @ b, 0.0],
                        width_m=q["gap_mm"] * 1e-3, open_width_m=ow, offset_m=abs(q["s_mm"]) * 1e-3, cost=q["cost"], yaw_deg=q["yaw_deg"], u0_mm=q["u0_mm"], s_mm=q["s_mm"]))
    return out
