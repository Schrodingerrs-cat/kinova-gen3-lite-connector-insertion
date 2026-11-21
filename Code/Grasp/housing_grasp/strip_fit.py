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


def strip_pose(view, sc, region_mm, hint_mm=None):
    """White strip top face from one view: yaw of the long edge (deg, mod 180), size, centre, top height. complete = whole strip in view."""
    hsv = cv2.cvtColor(view["color"], cv2.COLOR_BGR2HSV)
    w = sc["white"]
    mask = (hsv[..., 1] <= w["s_max"]) & (hsv[..., 2] >= w["v_min"])
    if "h" in w:
        mask &= (hsv[..., 0] >= w["h"][0]) & (hsv[..., 0] <= w["h"][1])
    P = backproject(view["depth"], mask, view["K"], view["T_BC"])
    x0, x1, y0, y1 = region_mm
    keep = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
    if hint_mm is not None:
        keep &= np.linalg.norm(P[:, :2] - hint_mm[:2], axis=1) < sc["search_radius_m"] * 1000
    P = P[keep]
    if len(P) < sc["min_points"]:
        return None
    hist, edges = np.histogram(P[:, 2], bins=np.arange(P[:, 2].min(), P[:, 2].max() + 6, 3))
    P = P[np.abs(P[:, 2] - (edges[hist.argmax()] + 1.5)) < sc["top_band_mm"]]
    o = P[:, :2].min(0) - 5
    ij = np.round(P[:, :2] - o).astype(int)
    grid = np.zeros(ij.max(0)[::-1] + 6, np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 1
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    _, lab, st, _ = cv2.connectedComponentsWithStats(grid)
    ys, xs = np.nonzero(lab == 1 + st[1:, 4].argmax())
    rect = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
    box = cv2.boxPoints(rect)
    edge = max((box[(k + 1) % 4] - box[k] for k in range(4)), key=np.linalg.norm)
    size = sorted(rect[1], reverse=True)
    nominal = np.array(sc["size_mm"], float)
    return dict(yaw=float(np.degrees(np.arctan2(edge[1], edge[0])) % 180), size=size, centre=np.array(rect[0]) + o, top_z=float(np.median(P[:, 2])), n=len(P),
                complete=bool(np.all(np.abs(np.array(size) - nominal) <= sc["size_tol"] * nominal)))


def strip_summary(poses, sc):
    """Combine the per-view strip poses (home first). yaw = mean of the complete views, spread = largest disagreement."""
    ok = [p for p in poses if p and p["complete"]]
    if not ok:
        return None
    yaw = float(np.degrees(np.angle(np.mean([np.exp(2j * np.radians(p["yaw"])) for p in ok]))) / 2 % 180)
    spread = max(abs(wrap180(a["yaw"] - b["yaw"])) for a in ok for b in ok)
    return dict(yaw=yaw, spread=float(spread), used=len(ok), home_ok=bool(poses[0] and poses[0]["complete"]),
                top_z=float(np.median([p["top_z"] for p in ok])), centre=np.mean([p["centre"] for p in ok], 0), size=np.median([p["size"] for p in ok], 0))


def find_stickers(view, strip, g, sc, max_extent_mm):
    """Sticker blobs (3D centres) lying on the strip."""
    n, lab, st, _ = cv2.connectedComponentsWithStats(marker_mask(view["color"], g).astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < 30:
            continue
        P = backproject(view["depth"], lab == i, view["K"], view["T_BC"])
        if len(P) < 15 or np.ptp(P[:, :2], axis=0).max() > max_extent_mm:
            continue
        c = np.median(P, 0)
        u, v = to_uv((c[:2] - strip["centre"])[None], strip["yaw"])[0]
        if abs(u) <= sc["size_mm"][0] / 2 + sc["margin_mm"] and abs(v) <= sc["size_mm"][1] / 2 + sc["margin_mm"]:
            out.append(c)
    return out


def choose_cable(centres, hint_mm=None):
    """Index: nearest to the hint, else the cable with most free space around it."""
    xy = np.array([c[:2] for c in centres])
    if hint_mm is not None:
        return int(np.linalg.norm(xy - hint_mm[:2], axis=1).argmin())
    if len(xy) == 1:
        return 0
    return int(np.argmax([np.linalg.norm(np.delete(xy, i, 0) - xy[i], axis=1).min() for i in range(len(xy))]))


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


def finger_gap(centre, jaw_yaw, open_mm, finger_w, finger_t, others, radius):
    """Smallest distance (mm) between a neighbouring cable's body and the box the open fingers sweep. inf if no neighbours."""
    gap = np.inf
    for o in others:
        a, b = to_uv((np.asarray(o) - centre)[None], jaw_yaw)[0]
        gap = min(gap, np.hypot(max(abs(a) - (open_mm / 2 + finger_t), 0), max(abs(b) - finger_w / 2, 0)) - radius)
    return float(gap)


def evaluate(obs, strip, spec, cfg, finger_w_mm, finger_t_mm, others_mm):
    """Gates + rectangle + grasp candidate from the close views. obs = view_points per close view."""
    p, gates = cfg["plug"], []

    def gate(name, ok, detail):
        gates.append(dict(name=name, ok=bool(ok), detail=detail))

    gate("strip complete in the home view, yaw agrees between views", strip is not None and strip["home_ok"] and strip["spread"] <= cfg["strip"]["yaw_spread_deg"],
         "no strip" if strip is None else "home complete %s, %d views, spread %.1f deg (<= %g)" % (strip["home_ok"], strip["used"], strip["spread"], cfg["strip"]["yaw_spread_deg"]))
    if strip is None:
        return dict(passed=False, gates=gates)
    n = [len(o["pts"]) for o in obs]
    gate("plug points in every view", min(n) >= p["min_points_view"], "%s (>= %d)" % (n, p["min_points_view"]))
    tops = [o["top"] for o in obs]
    top = float(np.median(tops))
    gate("plug top height agrees between views", np.ptp(tops) <= p["top_spread_mm"], "%.1f mm spread (<= %g)" % (np.ptp(tops), p["top_spread_mm"]))
    lo, hi = p["top_band_mm"]
    band = [o["pts"][(o["pts"][:, 2] >= top + lo) & (o["pts"][:, 2] <= top + hi), :2] for o in obs]
    if min(len(b) for b in band) < 20:
        gate("top band has points in every view", False, "%s" % [len(b) for b in band])
        return dict(passed=False, gates=gates)
    rel = spec["long_axis_rel_strip_deg"]
    yaw = strip["yaw"] + (rel or 0.0)
    rng = np.random.default_rng(1)
    pick = [b[rng.choice(len(b), min(600, len(b)), replace=False)] for b in band]
    centre, ext = fit_rect(np.vstack(pick), yaw, p["edge_pct"])
    cs = np.array([fit_rect(np.vstack([b for j, b in enumerate(pick) if j != k]), yaw, p["edge_pct"])[0] for k in range(len(pick))])
    loo = float(max(np.linalg.norm(a - b) for a in cs for b in cs))
    gate("plug centre repeatable leaving a view out", loo <= p["loo_mm"], "%.1f mm (<= %g)" % (loo, p["loo_mm"]))
    fp, tol = np.array(spec["footprint_mm"], float), spec["footprint_tol_mm"]
    gate("footprint matches the connector", np.all(np.abs(ext - fp) <= tol), "measured %.1f x %.1f mm, expected %.1f x %.1f +- %g" % (*ext, *fp, tol))
    free = None
    if rel is not None and fp.max() / fp.min() >= p["yaw_check_aspect"]:
        box = cv2.boxPoints(cv2.minAreaRect(np.vstack(pick).astype(np.float32)))
        edge = max((box[(k + 1) % 4] - box[k] for k in range(4)), key=np.linalg.norm)
        free = abs(wrap180(np.degrees(np.arctan2(edge[1], edge[0])) - (yaw if fp[0] >= fp[1] else yaw + 90)))
        gate("plug orientation agrees with the strip", free <= p["yaw_check_deg"], "free rectangle %.1f deg off (<= %g)" % (free, p["yaw_check_deg"]))
    jaw_yaw = strip["yaw"] + spec["jaw_rel_strip_deg"]
    width = ext[0] if abs(direction(jaw_yaw) @ direction(yaw)) > 0.7 else ext[1]
    tip_z = strip["top_z"] + spec["clear_mm"]
    gate("plug top clears the jaw band", top - (tip_z + spec["band_mm"]) >= spec["top_margin_mm"],
         "top %.1f mm, band top %.1f mm, margin needed %g" % (top - strip["top_z"], tip_z + spec["band_mm"] - strip["top_z"], spec["top_margin_mm"]))
    gate("plug fits the jaw opening", width + 4 <= spec["open_width_m"] * 1000, "%.1f mm in %.1f mm" % (width, spec["open_width_m"] * 1000))
    gap = finger_gap(centre, jaw_yaw, spec["open_width_m"] * 1000, finger_w_mm, finger_t_mm, others_mm, cfg["select"]["neighbour_radius_mm"])
    gate("fingers clear of neighbouring cables", gap >= cfg["select"]["finger_margin_mm"], "%.1f mm (>= %g)" % (gap, cfg["select"]["finger_margin_mm"]))
    cand = dict(p=np.r_[centre, tip_z] / 1000, jaw=np.r_[direction(jaw_yaw), 0.0], approach=np.array([0.0, 0.0, -1.0]), width_m=width / 1000,
                open_width_m=spec["open_width_m"], offset_m=0.0, cost=0.0)
    return dict(passed=all(g["ok"] for g in gates), gates=gates, centre_mm=centre, extent_mm=ext, yaw_deg=yaw, jaw_yaw_deg=jaw_yaw, top_mm=top, tip_z_mm=tip_z,
                loo_mm=loo, free_yaw_err_deg=free, finger_gap_mm=gap, band=band, candidate=cand)


def refine(view, spec, g, plan, cfg):
    """Refit the rectangle from the pre-grasp hover (camera straight above). ok = close to the plan and same footprint / top."""
    p = cfg["plug"]
    o = view_points(view, spec, g, np.r_[plan["centre_mm"], plan["top_mm"]], plan["strip_top_mm"])
    lo, hi = p["top_band_mm"]
    xy = o["pts"][(o["pts"][:, 2] >= o["top"] + lo) & (o["pts"][:, 2] <= o["top"] + hi), :2]
    if len(xy) < 20:
        raise RuntimeError("hover view: only %d plug points" % len(xy))
    centre, ext = fit_rect(xy, plan["yaw_deg"], p["edge_pct"])
    shift = float(np.linalg.norm(centre - plan["centre_mm"]))
    fp_ok = bool(np.all(np.abs(ext - np.array(spec["footprint_mm"])) <= spec["footprint_tol_mm"]))
    top_ok = abs(o["top"] - plan["top_mm"]) <= p["top_spread_mm"]
    return dict(centre_mm=centre, extent_mm=ext, top_mm=o["top"], shift_mm=shift, footprint_ok=fp_ok, top_ok=bool(top_ok),
                ok=bool(shift <= cfg["refine"]["max_shift_mm"] and fp_ok and top_ok))
