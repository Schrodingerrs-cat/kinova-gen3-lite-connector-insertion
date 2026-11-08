"""Stage 3: measure the housing (pose, size, profile) from the segmented object points.

Housing = the tall solid core of the object. A low cable is not part of it; a cable as tall as the
housing is cut off by its width. Housing frame H: x tip->cable, z support normal, origin at box center.
"""
import numpy as np

from .segment import segment_object


def _longest_run(mask):
    runs, start = [], None
    for i, w in enumerate(np.append(mask, False)):
        if w and start is None:
            start = i
        elif not w and start is not None:
            runs.append((start, i)); start = None
    return max(runs, key=lambda r: r[1] - r[0])


def housing_pose(points_B, in_roi, cam_origin, hs):
    """Returns dict(ok, reasons, T_BH, dims=(L,W,H), profile, tip_known, plane, cluster)."""
    reasons = []
    out = dict(ok=False, reasons=reasons, T_BH=None)
    try:
        plane, obj, h_all = segment_object(points_B, in_roi, cam_origin, hs)
    except ValueError as e:
        reasons.append(str(e))
        return out
    c0, n = plane
    out["plane"] = plane
    pts, hgt, roi = points_B[obj], h_all[obj], in_roi[obj]
    out["cluster"] = pts
    if len(pts) < hs["min_points"]:
        reasons.append("object too small: %d points (need %d)" % (len(pts), hs["min_points"]))
        return out
    if not roi.any():
        reasons.append("ROI vanished after segmentation")
        return out
    flat = pts - np.outer(hgt, n)
    # only the neighbourhood of the ROI: a long curved cable would bend the long axis
    near = np.linalg.norm(flat - flat[roi].mean(0), axis=1) < hs["crop_radius_m"]
    pts, hgt, flat = pts[near], hgt[near], flat[near]

    # solid core, relative to the object's own height
    core = hgt >= hs["core_frac"] * np.percentile(hgt, 95)
    if core.sum() < hs["min_points"] // 2:
        reasons.append("no solid core found")
        return out
    ctr = flat[core].mean(0)
    a = np.linalg.svd(flat[core] - ctr, full_matrices=False)[2][0]  # long axis
    a -= (a @ n) * n
    a /= np.linalg.norm(a)
    b = np.cross(n, a)
    s_all = (flat - ctr) @ a
    s, lat, hc = s_all[core], ((flat - ctr) @ b)[core], hgt[core]

    # per-bin profile of the core along a: lateral extent, lateral center, top height
    bw = hs["bin_m"]
    edges = np.arange(s.min(), s.max() + bw, bw)
    idx = np.digitize(s, edges) - 1
    ext, lc, top = (np.full(len(edges), np.nan) for _ in range(3))
    for i in np.unique(idx):
        m = idx == i
        if m.sum() >= hs["min_bin_points"]:
            lo, hi = np.percentile(lat[m], [0.5, 99.5])
            ext[i], lc[i], top[i] = hi - lo, (hi + lo) / 2, np.percentile(hc[m], 98)
    ok = ~np.isnan(ext)
    if ok.sum() < 4:
        reasons.append("housing core too short to profile")
        return out

    # a thin tall tail (cable as tall as the housing) is cut by width ratio; otherwise keep the whole core
    ii = np.flatnonzero(ok)
    k = min(4, max(1, len(ii) // 3))
    cable_w = min(np.median(ext[ii[:k]]), np.median(ext[ii[-k:]]))
    wide_w = np.percentile(ext[ok], 90)
    wide = ok & (ext >= (cable_w + wide_w) / 2) if wide_w >= hs["thick_ratio"] * cable_w else ok
    r0, r1 = _longest_run(wide)
    s_lo, s_hi = edges[r0], edges[r1 - 1] + bw

    # cable side: clearly more (low) points past one end; a few thin prongs must not count
    reach = hs["cable_probe_m"]
    n_hi = ((s_all > s_hi) & (s_all < s_hi + reach)).sum()
    n_lo = ((s_all < s_lo) & (s_all > s_lo - reach)).sum()
    tip_known = max(n_hi, n_lo) >= hs["min_points_cable"] and max(n_hi, n_lo) >= 2 * min(n_hi, n_lo)
    if not tip_known:
        reasons.append("tip/cable end not distinguishable (points past the ends: %d / %d): "
                       "grasping the middle, in-hand direction ambiguous" % (n_lo, n_hi))
    sgn = 1.0 if n_hi >= n_lo else -1.0
    x = sgn * a
    y = np.cross(n, x)
    in_run = (s >= s_lo) & (s <= s_hi)
    t_run = ((flat[core] - ctr) @ x)[in_run]
    t_tip, L = t_run.min(), np.ptp(t_run)
    runbin = np.arange(r0, r1)
    runbin = runbin[ok[runbin]]
    W = np.percentile(ext[runbin], 90)  # the jaws must clear the widest part
    top_h = np.percentile(hc[in_run], 98)
    yc = np.median(sgn * lc[runbin])
    if L < hs["min_length_m"]:
        reasons.append("housing section only %.1f mm long (need %.1f mm)" % (L * 1e3, hs["min_length_m"] * 1e3))
        return out

    sc = edges[runbin] + bw / 2
    s_tip = s[in_run].min() if sgn > 0 else s[in_run].max()
    order = np.argsort(np.abs(sc - s_tip))
    out["profile"] = dict(d=np.abs(sc - s_tip)[order], ext=ext[runbin][order],
                          top=top[runbin][order], yc=(sgn * lc[runbin])[order] - yc)
    out["dims"] = (L, W, top_h)
    out["tip_known"] = bool(tip_known)
    out["core_points"] = pts[core]

    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, n], 1)
    T[:3, 3] = ctr + x * (t_tip + L / 2) + y * yc + n * (top_h / 2)  # ctr lies on the support plane
    out.update(ok=True, T_BH=T)
    return out
