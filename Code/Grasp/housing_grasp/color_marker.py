"""Colour-marker grasp: colour thresholding in the image (any colour, see `color_marker.h / s_min / v_min`) -> marker point cloud -> table removal -> clusters -> the two marker planes -> grasp.

The grasp is searched ONLY on the marker points (the two marked side planes of the connector head).
"""
import cv2
import numpy as np
from scipy.ndimage import maximum_filter, minimum_filter
from sklearn.cluster import DBSCAN, KMeans

from .cloud import remove_outliers, voxel_downsample


def marker_mask(bgr, g):
    """Bool mask of colour marker pixels (OpenCV HSV: g['h'] range, saturation and value minimums)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h0, h1 = g["h"]  # OpenCV hue 0-179; a range that wraps past 179 (red) is written with h0 > h1, e.g. [170, 10]
    hue = (hsv[..., 0] >= h0) & (hsv[..., 0] <= h1) if h0 <= h1 else (hsv[..., 0] >= h0) | (hsv[..., 0] <= h1)
    m = hue & (hsv[..., 1] >= g["s_min"]) & (hsv[..., 2] >= g["v_min"])
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)


def marker_points(depth, bgr, K, T_BC, g):
    """Marker-coloured pixels -> cleaned base-frame points. Flying pixels are removed on the FULL depth before the mask is applied."""
    valid = np.isfinite(depth) & (depth > 0)
    hi = maximum_filter(np.where(valid, depth, 0.0), 3)
    lo = minimum_filter(np.where(valid, depth, np.inf), 3)
    valid &= (hi - lo) <= g["edge_jump_m"]
    vv, uu = np.nonzero(valid & marker_mask(bgr, g))
    z = depth[vv, uu].astype(float)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1)
    pb = pc @ T_BC[:3, :3].T + T_BC[:3, 3]
    if len(pb) == 0:
        return pb
    pts, roi = voxel_downsample(pb, np.zeros(len(pb), bool), g["voxel_m"])
    return remove_outliers(pts, roi, g["outlier_radius_m"], g["outlier_min_nb"])[0]


def merge_views(clouds, info, voxel):
    """Apply each view's ICP correction (info[k]['T'], view 0 = identity) to its cloud, stack, voxel-downsample."""
    out = [clouds[0]] + [c @ np.array(i["T"])[:3, :3].T + np.array(i["T"])[:3, 3] for c, i in zip(clouds[1:], info[1:])]
    out = np.vstack([o for o in out if len(o)])
    return voxel_downsample(out, np.zeros(len(out), bool), voxel)[0]


def ransac_plane(P, thresh=0.003, iters=800, seed=0):
    """Dominant plane (n, d) with n.p + d = 0, n pointing up, from the points P (the table)."""
    rng, best, best_n = np.random.default_rng(seed), None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < thresh
        if inl.sum() > best_n:
            best, best_n = (n, -n @ a), inl.sum()
    n, d = best
    inl = np.abs(P @ n + d) < thresh
    c = P[inl].mean(0)
    n = np.linalg.svd(P[inl] - c, full_matrices=False)[2][2]  # refit on all inliers
    n = n if n[2] > 0 else -n
    return n, -n @ c, int(inl.sum())


def dbscan(P, eps, min_samples):
    """Labels (-1 = noise) of DBSCAN on the points."""
    return DBSCAN(eps=eps, min_samples=min_samples).fit_predict(P) if len(P) else np.zeros(0, int)


def normals(P, k=15):
    """PCA normal of each point from its k nearest neighbours (sign arbitrary)."""
    from scipy.spatial import cKDTree
    idx = cKDTree(P).query(P, k=min(k, len(P)))[1]
    nb = P[idx] - P[idx].mean(1, keepdims=True)
    return np.linalg.svd(nb, full_matrices=False)[2][:, 2, :]


def fit_plane(P):
    c = P.mean(0)
    return c, np.linalg.svd(P - c, full_matrices=False)[2][2]


def marker_planes(G, g):
    """Split the marker points into the two marker planes with KMeans(k=2) on their horizontal normals.
    Returns [(points, centroid, normal), (points, centroid, normal)] with the normals pointing away from each other, or None."""
    N = normals(G, g["normal_knn"])
    N = N * np.where(np.einsum("ij,ij->i", N, G - G.mean(0)) < 0, -1.0, 1.0)[:, None]  # outward from the marker centre
    side = np.abs(N[:, 2]) < g["max_nz"]  # vertical-ish faces only
    if side.sum() < 2 * g["min_plane_points"]:
        return None
    H = N[side][:, :2]
    H = H / np.linalg.norm(H, axis=1, keepdims=True)
    lab = KMeans(n_clusters=2, n_init=10, random_state=0).fit_predict(H)
    planes = []
    for k in (0, 1):
        P = G[side][lab == k]
        if len(P) < g["min_plane_points"]:
            return None
        c, n = fit_plane(P)
        planes.append([P, c, n])
    (P1, c1, n1), (P2, c2, n2) = planes
    d = c2 - c1
    n1 = n1 if n1 @ d < 0 else -n1  # outward = away from the other plane
    n2 = n2 if n2 @ d > 0 else -n2
    return [(P1, c1, n1), (P2, c2, n2)]


def marker_frame(C):
    """Frame of one marker patch from PCA: long axis along it, jaw axis across it, normal = patch plane normal (up)."""
    c = C.mean(0)
    long_, mid, n = np.linalg.svd(C - c, full_matrices=False)[2]
    n = n if n[2] > 0 else -n
    size = [float(np.subtract(*np.percentile((C - c) @ a, [99, 1]))) for a in (long_, mid, n)]
    return dict(centroid=c, long_axis=long_, jaw_axis=mid, normal=n, approach=-n, length_m=size[0], width_m=size[1], thickness_m=size[2])


def largest_blob(mask, min_px):
    """Bounding box (u0, v0, u1, v1) of the largest connected blob in a bool mask, or None."""
    n, _, st, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[i, cv2.CC_STAT_AREA] < min_px:
        return None
    x, y, w, h = st[i, :4]
    return int(x), int(y), int(x + w), int(y + h)


def analyze_scan(P, G, cfg):
    """Merged-scan stages: table RANSAC -> above-table object points + clusters -> marker clusters on the object -> frame of the largest."""
    g, tz = cfg["color_marker"], cfg["scan"]["table_z_m"]
    band = P[np.abs(P[:, 2] - tz) < 0.03]
    sub = band[np.random.default_rng(0).choice(len(band), min(150000, len(band)), replace=False)]
    n, d, n_inl = ransac_plane(sub, g["table_thresh_m"])
    x0, x1, y0, y1 = cfg["workspace"]["region"]
    h = P @ n + d
    inbox = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
    obj, _ = voxel_downsample(P[inbox & (h > g["min_above_table_m"]) & (h < g["max_above_table_m"])], np.zeros(int((inbox & (h > g["min_above_table_m"]) & (h < g["max_above_table_m"])).sum()), bool), 0.003)
    hg = G @ n + d
    Ga = G[(hg > g["min_above_table_m"]) & (hg < g["max_above_table_m"])]
    glab = dbscan(Ga, g["marker_eps_m"], g["marker_min_pts"])
    clusters = sorted((Ga[glab == k] for k in set(glab) - {-1}), key=len, reverse=True)
    return dict(table_n=n, table_d=d, table_inliers=n_inl, object_points=obj, object_labels=dbscan(obj, g["cluster_eps_m"], g["cluster_min_pts"]),
                marker_above=Ga, marker_labels=glab, marker_clusters=clusters, frame=marker_frame(clusters[0]) if clusters else None)


def head_mask(bgr, K, T_BC, c, g):
    """Image-space segmentation of the head: dark or marker pixels connected to the sticker, within footprint_radius_m of it.
    c = sticker centroid in the base frame. Returns (bool mask, depth of c in the camera frame)."""
    cam = np.linalg.inv(T_BC) @ np.r_[c, 1]
    Z0 = cam[2]
    u0, v0 = K[0, 0] * cam[0] / Z0 + K[0, 2], K[1, 1] * cam[1] / Z0 + K[1, 2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    body = (hsv[..., 2] < g["dark_v"]) | marker_mask(bgr, g)
    vv, uu = np.mgrid[0:bgr.shape[0], 0:bgr.shape[1]]
    body &= np.hypot(uu - u0, vv - v0) < g["footprint_radius_m"] * K[0, 0] / Z0
    body = cv2.morphologyEx(body.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, lab = cv2.connectedComponents(body)
    k = lab[int(round(v0)), int(round(u0))]
    if k == 0:
        raise RuntimeError("no housing pixels around the marker sticker")
    return lab == k, Z0


def head_points(depth, bgr, K, T_BC, c, g):
    """Image mode: ONLY the head pixels (image-space mask), each lifted to 3D with its own depth. No flying-pixel filter here (it would erode the head
    outline by a pixel per side, and this cloud is used for xy extents; the grasp height comes from the sticker points). Pixels outside a wide height
    band around the sticker and isolated points are removed."""
    mask, _ = head_mask(bgr, K, T_BC, c, g)
    vv, uu = np.nonzero(np.isfinite(depth) & (depth > 0) & mask)
    z = depth[vv, uu].astype(float)
    pb = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1) @ T_BC[:3, :3].T + T_BC[:3, 3]
    below, above = g["head_band_m"]  # wide: the dark body and the sticker form one continuous surface on this head, do not cut it
    pb = pb[(pb[:, 2] > c[2] - below) & (pb[:, 2] < c[2] + above)]
    if len(pb) == 0:
        return pb
    pts, roi = voxel_downsample(pb, np.zeros(len(pb), bool), g["voxel_m"])
    return remove_outliers(pts, roi, g["outlier_radius_m"], g["outlier_min_nb"])[0]


def housing_footprint(bgr, depth, K, T_BC, c, g):
    """Top-face footprint of the dark housing around the marker sticker (colour image, back-projected at the sticker's depth).
    c = sticker centroid in the base frame. Returns dict(long_axis, jaw_axis (unit xy in base), length_m, width_m, n_px)."""
    mask, Z0 = head_mask(bgr, K, T_BC, c, g)
    vs, us = np.nonzero(mask)
    pc = np.stack([(us - K[0, 2]) * Z0 / K[0, 0], (vs - K[1, 2]) * Z0 / K[1, 1], np.full(len(us), Z0)], 1)
    pb = (pc @ T_BC[:3, :3].T + T_BC[:3, 3])[:, :2]
    m = pb.mean(0)
    ax = np.linalg.svd(pb - m, full_matrices=False)[2]
    ext = [float(np.subtract(*np.percentile((pb - m) @ a, [98, 2]))) for a in ax]
    return dict(long_axis=ax[0], jaw_axis=ax[1], length_m=ext[0], width_m=ext[1], n_px=int(len(us)), center_xy=m)


def synthesize_head_grasps(P, G, c, g, finger_w, max_open, margin, drop, search=True):
    """Image mode: antipodal grasp candidates on the head-only cloud P (base frame). Jaw axis = across the head (PCA of the xy footprint: the pads close on
    the two long sides), searched over yaw offsets g['yaw_steps_deg'] and g['n_offsets'] pad positions along the head within +- g['offset_span_m'] of the
    sticker. For each: pad window = head points within +- finger_w/2 along the head; width = 2nd-98th percentile extent across the jaw; opening = width + 2 margins;
    TCP `drop` below the sticker height G inside the window. Cost = unevenness of the width along the pad (parallel faces are best) + 0.1 x position offset
    + yaw_penalty x |yaw| / 45 + (height_rule penalize) height_penalty x |grasp height - sticker height| / max_z_spread_m.
    height_rule: penalize = positions where the surface height changes are ranked lower; drop = removed when more than max_z_spread_m from the best.
    search=False: ONE candidate (pad position at the sticker, yaw 0). Returns candidates best first:
    dict(p, jaw (unit 3-vector, horizontal), width_m, open_width_m, cost, offset_m, yaw_deg, z_top_m)."""
    m = P[:, :2].mean(0)
    a0, b0 = np.linalg.svd(P[:, :2] - m, full_matrices=False)[2]
    out = []
    for yaw in (g["yaw_steps_deg"] if search else [0]):
        r = np.radians(yaw)
        rot = lambda v: np.array([np.cos(r) * v[0] - np.sin(r) * v[1], np.sin(r) * v[0] + np.cos(r) * v[1]])
        a, b = rot(a0), rot(b0)
        s, t = (P[:, :2] - m) @ a, (P[:, :2] - m) @ b
        lo, hi = np.percentile(s, [2, 98])
        s_c = (c[:2] - m) @ a
        for off in (np.linspace(-g["offset_span_m"], g["offset_span_m"], g["n_offsets"]) if search else [0.0]):
            s0 = s_c + off
            win = np.abs(s - s0) <= finger_w / 2
            if not (lo + finger_w / 2 <= s0 <= hi - finger_w / 2) or win.sum() < g["min_window_points"]:
                continue
            tw = t[win]
            widths = [np.subtract(*np.percentile(tw[k], [98, 2])) for k in np.array_split(np.argsort(s[win]), 3)]
            t_lo, t_hi = np.percentile(tw, [2, 98])
            width = float(t_hi - t_lo)
            if width + 2 * margin > max_open:
                continue
            gw = np.abs((G[:, :2] - m) @ a - s0) <= finger_w / 2
            z_top = float(np.median(G[gw, 2])) if gw.sum() >= 5 else float(c[2])
            cost = np.std(widths) / width + 0.1 * abs(off) / (hi - lo) + g["yaw_penalty"] * abs(yaw) / 45.0
            if g["head_height_rule"] == "penalize":
                cost += g["height_penalty"] * abs(z_top - c[2]) / g["max_z_spread_m"]
            out.append(dict(p=np.r_[m + a * s0 + b * (t_lo + t_hi) / 2, z_top - drop], jaw=np.r_[b, 0.0], width_m=width, open_width_m=width + 2 * margin,
                            cost=float(cost), offset_m=float(off), yaw_deg=float(yaw), z_top_m=z_top))
    out = sorted(out, key=lambda d: d["cost"])
    if g["head_height_rule"] == "drop" and out:
        out = [q for q in out if abs(q["p"][2] - out[0]["p"][2]) <= g["max_z_spread_m"]]
    return out



def sticker_rect(bgr, K, T_BC, c, g):
    """Cloud mode: edge-based geometry of the marker sticker from the IMAGE only. Min-area rectangle of the marker blob at the sticker; its 4 corners are
    intersected with the horizontal plane z = c[2] (sticker height from the 3D median). The 3D sticker points themselves are NOT used for axes: on the glossy
    head their PCA / plane are distorted. Returns dict(long_axis, short_axis (unit xy), long_m, short_m, center_xy, fill)."""
    cam = np.linalg.inv(T_BC) @ np.r_[c, 1]
    u0, v0 = K[0, 0] * cam[0] / cam[2] + K[0, 2], K[1, 1] * cam[1] / cam[2] + K[1, 2]
    _, lab = cv2.connectedComponents(marker_mask(bgr, g).astype(np.uint8))
    k = lab[int(round(v0)), int(round(u0))]
    if k == 0:
        raise RuntimeError("no marker blob at the sticker position")
    vs, us = np.nonzero(lab == k)
    rect = cv2.minAreaRect(np.stack([us, vs], 1).astype(np.float32))
    corners = cv2.boxPoints(rect)
    o, R = T_BC[:3, 3], T_BC[:3, :3]
    P = []
    for u, v in corners:
        d = R @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        P.append((o + d * (c[2] - o[2]) / d[2])[:2])
    P = np.array(P)
    e1, e2 = P[1] - P[0], P[2] - P[1]
    (a, la), (b, lb) = sorted(((e1, np.linalg.norm(e1)), (e2, np.linalg.norm(e2))), key=lambda t: -t[1])
    return dict(long_axis=a / la, short_axis=b / lb, long_m=float(la), short_m=float(lb), center_xy=P.mean(0), fill=float(len(us) / max(1e-9, cv2.contourArea(corners.astype(np.float32)))))


def sticker_candidates(rect, c, g, finger_w, max_open, margin, drop, search=True):
    """Cloud mode: grasp candidates on the marker region. rect = dict(long_axis, short_axis, long_m, short_m, center_xy): the sticker rectangle from the image, or
    (extent_source: footprint) the head outline with the sticker centre. Jaws close along the LONG or SHORT edge of rect; the pads then touch the side faces
    perpendicular to the jaw, whose horizontal width is the OTHER extent (face_width_m). Which family is preferred (cost 0, the other 0.5):
      contact_faces edge_prior -> g['jaw_edge'] (a convention),  larger -> jaw along the SHORTER extent (pads on the wider faces),  smaller -> the opposite.
    search=True: both families x yaw offsets (g['yaw_steps_deg']) x pad positions (g['n_offsets'] across +- g['offset_span_m']); search=False: ONE closed-form
    candidate (preferred family, yaw 0, centred). Opening = extent along the jaw + 2 margins; TCP `drop` below the sticker height. Cost = family prior +
    0.3 x centring + yaw_penalty x |yaw| / 45. Returns candidates best first: dict(p, jaw (unit 3-vector), width_m, open_width_m, face_width_m, cost, offset_m, edge, yaw_deg)."""
    pref = dict(larger="short", smaller="long").get(g["contact_faces"], g["jaw_edge"])
    out = []
    for edge, jaw, perp, ext_jaw, ext_perp in (("long", rect["long_axis"], rect["short_axis"], rect["long_m"], rect["short_m"]),
                                              ("short", rect["short_axis"], rect["long_axis"], rect["short_m"], rect["long_m"])):
        if not search and edge != pref:
            continue
        prior = 0.0 if edge == pref else 0.5
        span = max(0.0, min(g["offset_span_m"], ext_perp / 2 - finger_w / 2)) if search else 0.0
        for yaw in (g["yaw_steps_deg"] if search else [0]):
            a = np.radians(yaw)
            j = np.array([np.cos(a) * jaw[0] - np.sin(a) * jaw[1], np.sin(a) * jaw[0] + np.cos(a) * jaw[1]])
            for off in (np.linspace(-span, span, g["n_offsets"]) if span > 0 else [0.0]):
                open_w = ext_jaw + 2 * margin
                if open_w > max_open:
                    continue
                centring = 0.5 * abs(off) / span if span > 0 else 0.0
                out.append(dict(p=np.r_[rect["center_xy"] + perp * off, c[2] - drop], jaw=np.r_[j, 0.0], width_m=float(ext_jaw), open_width_m=float(open_w),
                                face_width_m=float(ext_perp), cost=float(prior + 0.3 * centring + g["yaw_penalty"] * abs(yaw) / 45.0), offset_m=float(off), edge=edge, yaw_deg=float(yaw)))
    return sorted(out, key=lambda d: d["cost"])
