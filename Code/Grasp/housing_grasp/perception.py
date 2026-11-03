"""ROI -> connector housing pose from one RGB-D view.

Assumes the housing lies flat on a support surface (cradle/table) with its cable
attached at one end. Roll/pitch come from the support plane, yaw from the housing's
long axis, and the tip end from where the thin cable leaves the housing.
All points are in the robot base frame B (meters).
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def cloud_from_depth(depth_m, K, T_BC, roi_px, margin_px=120):
    """Back-project depth in an expanded ROI window to base-frame points.

    roi_px = (u0, v0, u1, v1). Returns (points_B Nx3, in_roi N bool).
    Window is expanded so the cable and the support surface are included.
    """
    u0, v0, u1, v1 = roi_px
    h, w = depth_m.shape
    a, b = max(int(v0 - margin_px), 0), min(int(v1 + margin_px), h)
    c, d = max(int(u0 - margin_px), 0), min(int(u1 + margin_px), w)
    vv, uu = np.mgrid[a:b, c:d]
    z = depth_m[a:b, c:d]
    ok = np.isfinite(z) & (z > 0)
    uu, vv, z = uu[ok], vv[ok], z[ok]
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1)
    pb = pc @ T_BC[:3, :3].T + T_BC[:3, 3]
    in_roi = (uu >= u0) & (uu < u1) & (vv >= v0) & (vv < v1)
    return pb, in_roi


def fit_plane(points, cam_origin, threshold=0.002, iters=300, seed=0):
    """RANSAC plane. Returns (point_on_plane, unit normal facing the camera)."""
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        p = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((points - p[0]) @ n) < threshold
        if inl.sum() > best_n:
            best, best_n = (p[0], n, inl), inl.sum()
    if best is None:
        raise ValueError("plane fit failed")
    _, _, inl = best
    q = points[inl]
    c = q.mean(0)
    n = np.linalg.svd(q - c)[2][-1]  # least-squares refit on inliers
    if (cam_origin - c) @ n < 0:
        n = -n
    return c, n


def largest_roi_cluster(points, in_roi, radius=0.004):
    """Connected component (radius graph) holding the most ROI points."""
    pairs = cKDTree(points).query_pairs(radius, output_type="ndarray")
    n = len(points)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, lab = connected_components(g, directed=False)
    counts = np.bincount(lab[in_roi], minlength=lab.max() + 1)
    return lab == counts.argmax()


def housing_pose(points_B, in_roi, cam_origin, hs):
    """Estimate housing pose. hs = config['housing'].

    Returns dict(ok, reasons, T_BH, plane, cluster) where housing frame H has
    x from tip toward cable, z = support normal (up), origin at box center.
    """
    L, W, H = hs["length_m"], hs["width_m"], hs["height_m"]
    reasons = []
    out = dict(ok=False, reasons=reasons, T_BH=None)

    # support plane from points outside the ROI
    c0, n = fit_plane(points_B[~in_roi], cam_origin)
    out["plane"] = (c0, n)
    height = (points_B - c0) @ n
    above = height > hs["min_height_m"]
    if above.sum() < hs["min_points"]:
        reasons.append("no points above support plane")
        return out
    pts, roi, hgt = points_B[above], in_roi[above], height[above]
    if not roi.any():
        reasons.append("no object points inside ROI")
        return out
    keep = largest_roi_cluster(pts, roi, hs["cluster_radius_m"])
    pts, hgt = pts[keep], hgt[keep]
    out["cluster"] = pts
    if len(pts) < hs["min_points"]:
        reasons.append("cluster too small")
        return out

    # in-plane axes: first PC of the cluster = long axis (housing + cable)
    flat = pts - np.outer(hgt, n)
    ctr = flat.mean(0)
    a = np.linalg.svd(flat - ctr, full_matrices=False)[2][0]
    a -= (a @ n) * n
    a /= np.linalg.norm(a)
    b = np.cross(n, a)
    s, lat = (flat - ctr) @ a, (flat - ctr) @ b

    # lateral width profile along a: housing = wide run, cable = thin tail
    bw = hs["bin_m"]
    edges = np.arange(s.min(), s.max() + bw, bw)
    idx = np.digitize(s, edges) - 1
    wide = np.zeros(len(edges), bool)
    cnt = np.zeros(len(edges), int)
    for i in np.unique(idx):
        m = idx == i
        cnt[i] = m.sum()
        if cnt[i] >= hs["min_bin_points"]:
            lo, hi = np.percentile(lat[m], [2, 98])
            wide[i] = (hi - lo) >= hs["wide_frac"] * W
    runs, start = [], None
    for i, w in enumerate(np.append(wide, False)):
        if w and start is None:
            start = i
        elif not w and start is not None:
            runs.append((start, i)); start = None
    if not runs:
        reasons.append("no housing-width section found")
        return out
    r0, r1 = max(runs, key=lambda r: r[1] - r[0])
    s_lo, s_hi = edges[r0], edges[r1 - 1] + bw

    # cable side = the run end that continues with (thin) points
    reach = hs["cable_probe_m"]
    cable_hi = ((s > s_hi) & (s < s_hi + reach)).sum() >= hs["min_points_cable"]
    cable_lo = ((s < s_lo) & (s > s_lo - reach)).sum() >= hs["min_points_cable"]
    if cable_hi == cable_lo:
        reasons.append("cannot tell tip from cable end (cable visible on %s side)"
                       % ("both" if cable_hi else "neither"))
        return out
    x = a if cable_hi else -a  # tip -> cable
    t = (flat - ctr) @ x
    in_run = (s >= s_lo) & (s <= s_hi)
    t_tip = t[in_run].min()
    y = np.cross(n, x)
    yv = (flat - ctr) @ y
    y_lo, y_hi = np.percentile(yv[in_run], [2, 98])
    top = np.percentile(hgt[in_run], 98)

    meas = dict(length=t[in_run].max() - t_tip, width=y_hi - y_lo, height=top)
    out["measured"] = meas
    for k, ref, tol in (("length", L, hs["length_tol_m"]), ("width", W, hs["width_tol_m"]),
                        ("height", H, hs["height_tol_m"])):
        if abs(meas[k] - ref) > tol:
            reasons.append(f"measured {k} {meas[k]*1e3:.1f} mm vs config {ref*1e3:.1f} mm")
    if reasons:
        return out

    # ctr lies on the support plane; offsets are measured from it
    center = ctr + x * (t_tip + L / 2) + y * (y_lo + y_hi) / 2 + n * (top - H / 2)
    R = np.stack([x, y, n], 1)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = center
    out.update(ok=True, T_BH=T)
    return out
