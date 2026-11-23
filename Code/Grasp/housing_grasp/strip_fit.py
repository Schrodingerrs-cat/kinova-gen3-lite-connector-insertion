"""Strip mode: cables lie on a white strip, plug yaw comes from the strip, plug = rectangle of known yaw on its top band. Offline, no ROS. mm, base frame."""
import cv2
import numpy as np

from .cad_fit import backproject
from .color_marker import head_mask, marker_mask, nearest_blob
from .connectors import marker_cfg


def direction(yaw_deg):
    a = np.radians(yaw_deg)
    return np.array([np.cos(a), np.sin(a)])


def wrap180(d):
    return (d + 90) % 180 - 90


def to_uv(xy, yaw_deg):
    """xy -> (along yaw, across yaw)"""
    return np.c_[xy @ direction(yaw_deg), xy @ direction(yaw_deg + 90)]


def from_uv(uv, yaw_deg):
    return uv[..., 0:1] * direction(yaw_deg) + uv[..., 1:2] * direction(yaw_deg + 90)


def strip_pose(view, sc, region_mm, seed_mm=None):
    """White strip top face from one view (searched around seed_mm, e.g. the stickers): the strip-sized box that is full of white with an empty ring around it (yaw of its long edge, centre, top height). complete = box full enough."""
    hsv = cv2.cvtColor(view["color"], cv2.COLOR_BGR2HSV)
    w = sc["white"]
    mask = (hsv[..., 1] <= w["s_max"]) & (hsv[..., 2] >= w["v_min"])
    if "h" in w:
        mask &= (hsv[..., 0] >= w["h"][0]) & (hsv[..., 0] <= w["h"][1])
    P = backproject(view["depth"], mask, view["K"], view["T_BC"])
    x0, x1, y0, y1 = region_mm
    keep = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
    if seed_mm is not None:
        keep &= np.linalg.norm(P[:, :2] - seed_mm[:2], axis=1) < sc["search_radius_m"] * 1000
    P = P[keep]
    if len(P) < sc["min_points"]:
        return None
    hist, edges = np.histogram(P[:, 2], bins=np.arange(P[:, 2].min(), P[:, 2].max() + 6, 3))
    P = P[np.abs(P[:, 2] - (edges[hist.argmax()] + 1.5)) < sc["top_band_mm"]]
    cells = np.unique(np.round(P[:, :2]), axis=0)
    L, W = sc["size_mm"]
    ring, best = 8, None
    for yaw in range(180):   # long axis direction; keep the placement where the strip-sized box is full and its outside ring is empty
        q = to_uv(cells, yaw)
        lo = q.min(0) - 12
        ij = np.round(q - lo).astype(int)
        g = np.zeros(ij.max(0)[::-1] + 25, np.float32)
        g[ij[:, 1], ij[:, 0]] = 1
        g = cv2.morphologyEx(g, cv2.MORPH_CLOSE, np.ones((3, 3), np.float32))
        inner = cv2.boxFilter(g, -1, (int(L), int(W)), borderType=cv2.BORDER_CONSTANT)
        outer = cv2.boxFilter(g, -1, (int(L) + 2 * ring, int(W) + 2 * ring), borderType=cv2.BORDER_CONSTANT)
        a_in, a_out = int(L) * int(W), (int(L) + 2 * ring) * (int(W) + 2 * ring)
        score = inner - (outer * a_out - inner * a_in) / (a_out - a_in)
        y, x = np.unravel_index(score.argmax(), score.shape)
        if best is None or score[y, x] > best[0]:
            best = (float(score[y, x]), yaw, from_uv(np.array([x, y]) + lo, yaw), float(inner[y, x]))
    score, yaw, centre, inside = best
    return dict(yaw=float(yaw), size=[float(L), float(W)], centre=centre, top_z=float(np.median(P[:, 2])), n=len(P), inside=inside, score=score,
                complete=bool(inside >= sc["min_fill"] and score >= sc["min_score"]))


def strip_summary(poses):
    """Strip yaw (mean over the views that see the whole strip, home first), centre and top height."""
    ok = [p for p in poses if p and p["complete"]]
    if not ok:
        return None
    yaw = float(np.degrees(np.angle(np.mean([np.exp(2j * np.radians(p["yaw"])) for p in ok]))) / 2 % 180)
    return dict(yaw=yaw, top_z=float(np.median([p["top_z"] for p in ok])), centre=np.mean([p["centre"] for p in ok], 0))


def sticker_blobs(view, g, max_extent_mm):
    """Teal blobs with depth: dict(centre 3D mm, long, short xy extent mm of the blob)."""
    n, lab, st, _ = cv2.connectedComponentsWithStats(marker_mask(view["color"], g).astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < 30:
            continue
        P = backproject(view["depth"], lab == i, view["K"], view["T_BC"])
        if len(P) < 15 or np.ptp(P[:, :2], axis=0).max() > max_extent_mm:
            continue
        (_, _), (w, h), _ = cv2.minAreaRect(P[:, :2].astype(np.float32))
        c = np.median(P, 0)
        mm_px = np.linalg.norm(c / 1000 - view["T_BC"][:3, 3]) / view["K"][0, 0] * 1000   # the depth-edge filter drops ~1 pixel per side: add it back
        out.append(dict(centre=c, long=max(w, h) + 2 * mm_px, short=min(w, h) + 2 * mm_px))
    return out


def find_stickers(view, strip, g, sc, max_extent_mm):
    """Blobs lying on the strip."""
    def on(b):
        u, v = to_uv((b["centre"][:2] - strip["centre"])[None], strip["yaw"])[0]
        return abs(u) <= sc["size_mm"][0] / 2 + sc["margin_mm"] and abs(v) <= sc["size_mm"][1] / 2 + sc["margin_mm"]
    return [b for b in sticker_blobs(view, g, max_extent_mm) if on(b)]


def choose_sticker(blobs, strip, slot_mm, tol_mm):
    """The sticker sitting in the connector's predefined slot (u along the crib, v across, mm from the crib centre). Raises if none is within tol_mm."""
    uv = to_uv(np.array([b["centre"][:2] for b in blobs]) - strip["centre"], strip["yaw"])
    d = np.linalg.norm(uv - np.asarray(slot_mm, float), axis=1)
    if d.min() > tol_mm:
        raise RuntimeError("no sticker within %g mm of the slot (%.0f, %.0f) mm; stickers at crib (u, v) mm: %s" % (tol_mm, *slot_mm, np.round(uv).astype(int).tolist()))
    return int(d.argmin())


def view_points(view, spec, g, sticker_mm, floor_z):
    """Plug points of one view: sticker nearest the chosen cable + dark/sticker pixels around it, above the strip. top = sticker 90th percentile height."""
    g = marker_cfg(g, spec)
    K, T = view["K"], view["T_BC"]
    st = backproject(view["depth"], nearest_blob(marker_mask(view["color"], g), K, T, np.asarray(sticker_mm) / 1000), K, T)
    st = st[st[:, 2] > floor_z + 3]
    if len(st) < 15:
        raise RuntimeError("sticker of the chosen cable not found (%d depth points)" % len(st))
    g2 = dict(g, dark_v=spec["dark_v"], footprint_radius_m=spec["roi_radius_mm"] / 1000)
    pts = backproject(view["depth"], head_mask(view["color"], K, T, np.median(st, 0) / 1000, g2)[0], K, T)
    return dict(pts=pts[pts[:, 2] > floor_z + 3], sticker=st, top=float(np.percentile(st[:, 2], 90)))


def fit_rect(xy, yaw_deg, edge_pct):
    """Rectangle of known yaw around xy: centre and (along, across) extents from the edge percentiles."""
    q = to_uv(xy, yaw_deg)
    lo, hi = np.percentile(q, edge_pct, 0), np.percentile(q, 100 - edge_pct, 0)
    return from_uv((lo + hi) / 2, yaw_deg), hi - lo


def align_to_crib(xy, pose, ref):
    """Move a view's plan-view points so that view's crib lies on the reference crib: removes that view's own camera pose error (it hits the crib and the plug alike)."""
    d = np.radians(ref["yaw"] - pose["yaw"])
    R = np.array([[np.cos(d), -np.sin(d)], [np.sin(d), np.cos(d)]])
    return (xy - pose["centre"]) @ R.T + ref["centre"]


def evaluate(obs, strip, spec, cfg, poses):
    """Rectangle of the strip-given yaw on the plug top band + the grasp. obs = view_points per view (home first), poses = strip_pose per view.
    Views that see the whole crib are aligned on it first (removes each view's own pose error). Nothing here blocks the grasp; the numbers are reported."""
    p = cfg["plug"]
    top = float(np.median([o["top"] for o in obs]))
    lo, hi = p["top_band_mm"]
    band = [o["pts"][(o["pts"][:, 2] >= top + lo) & (o["pts"][:, 2] <= top + hi), :2] for o in obs]
    use = [i for i, q in enumerate(poses) if q and q["complete"] and len(band[i]) >= p["min_points_view"]]
    band = [align_to_crib(band[i], poses[i], strip) for i in use] if use else [b for b in band if len(b) >= p["min_points_view"]]
    if not band:
        raise RuntimeError("no plug points in the top band")
    yaw = strip["yaw"] + (spec["long_axis_rel_strip_deg"] or 0.0)
    centre, ext = fit_rect(np.vstack(band), yaw, p["edge_pct"])
    jaw_yaw = strip["yaw"] + spec["jaw_rel_strip_deg"]
    width = ext[0] if abs(direction(jaw_yaw) @ direction(yaw)) > 0.7 else ext[1]
    tip_z = strip["top_z"] + spec["clear_mm"]
    cand = dict(p=np.r_[centre, tip_z] / 1000, jaw=np.r_[direction(jaw_yaw), 0.0], approach=np.array([0.0, 0.0, -1.0]), width_m=width / 1000,
                open_width_m=spec["open_width_m"], offset_m=0.0, cost=0.0)
    return dict(centre_mm=centre, extent_mm=ext, yaw_deg=yaw, jaw_yaw_deg=jaw_yaw, top_mm=top, tip_z_mm=tip_z, views_used=len(band), candidate=cand)


def refine(view, spec, g, plan, cfg):
    """Refit the rectangle from the pre-grasp hover (camera straight above): new centre and how far it moved."""
    p = cfg["plug"]
    o = view_points(view, spec, g, np.r_[plan["centre_mm"], plan["top_mm"]], plan["strip_top_mm"])
    lo, hi = p["top_band_mm"]
    xy = o["pts"][(o["pts"][:, 2] >= o["top"] + lo) & (o["pts"][:, 2] <= o["top"] + hi), :2]
    if len(xy) < p["min_points_view"]:
        raise RuntimeError("hover view: only %d plug points" % len(xy))
    centre, ext = fit_rect(xy, plan["yaw_deg"], p["edge_pct"])
    return dict(centre_mm=centre, extent_mm=ext, top_mm=o["top"], shift_mm=float(np.linalg.norm(centre - plan["centre_mm"])))
