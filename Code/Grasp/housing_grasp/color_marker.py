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


def nearest_blob(mask, K, T_BC, target, min_px=30):
    """Keep only the connected marker blob (>= min_px) nearest to the pixel where the scan target (base frame) projects: other marker-coloured things in the
    view (a strip on the panel) are dropped."""
    cam = np.linalg.inv(T_BC) @ np.r_[target, 1]
    u0, v0 = K[0, 0] * cam[0] / cam[2] + K[0, 2], K[1, 1] * cam[1] / cam[2] + K[1, 2]
    n, lab, st, cen = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    best, bd = 0, np.inf
    for i in range(1, n):
        d = np.hypot(cen[i][0] - u0, cen[i][1] - v0)
        if st[i, cv2.CC_STAT_AREA] >= min_px and d < bd:
            best, bd = i, d
    return lab == best if best else np.zeros(mask.shape, bool)


def marker_points(depth, bgr, K, T_BC, g, target=None):
    """Marker-coloured pixels -> cleaned base-frame points. Flying pixels are removed on the FULL depth before the mask is applied.
    target (base-frame xyz of the scan target): only the marker blob nearest to it is used."""
    valid = np.isfinite(depth) & (depth > 0)
    hi = maximum_filter(np.where(valid, depth, 0.0), 3)
    lo = minimum_filter(np.where(valid, depth, np.inf), 3)
    valid &= (hi - lo) <= g["edge_jump_m"]
    m = marker_mask(bgr, g)
    if target is not None:
        m = nearest_blob(m, K, T_BC, target)
    vv, uu = np.nonzero(valid & m)
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
    small = [c for c in clusters if (c[:, :2].max(0) - c[:, :2].min(0)).max() <= g["marker_max_extent_m"]]   # the sticker is a small patch; a long strip of the same colour is not
    clusters = small + [c for c in clusters if all(c is not s for s in small)]   # sticker-sized clusters first (largest first), the large ones after
    return dict(table_n=n, table_d=d, table_inliers=n_inl, object_points=obj, object_labels=dbscan(obj, g["cluster_eps_m"], g["cluster_min_pts"]),
                marker_above=Ga, marker_labels=glab, marker_clusters=clusters, n_sticker_sized=len(small), frame=marker_frame(small[0]) if small else None)


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


def oriented_normals(P, k=15):
    """PCA normals of the cloud, flipped to point away from the cloud centre (top face up, side walls outward)."""
    n = normals(P, k)
    return np.where(((P - P.mean(0)) * n).sum(1, keepdims=True) < 0, -n, n)


def outline_normals(P, k=25):
    """Horizontal (xy) outward normals of the head outline: min-variance direction of each point's xy neighbourhood (a half-disc at the edge), pointing away from
    the cloud centre. Returned as 3-vectors with z = 0. Used for the pad contacts, because the 3D normals on this head follow the depth ramp, not the walls."""
    from scipy.spatial import cKDTree
    xy = P[:, :2]
    idx = cKDTree(xy).query(xy, k=min(k, len(xy)))[1]
    nb = xy[idx] - xy[idx].mean(1, keepdims=True)
    n = np.linalg.svd(nb, full_matrices=False)[2][:, 1, :]
    n = np.where(((xy - xy.mean(0)) * n).sum(1, keepdims=True) < 0, -n, n)
    return np.c_[n, np.zeros(len(n))]


def scene_points(depth, bgr, K, T_BC, c, g):
    """Non-head points around the head for the finger-collision test: every depth pixel outside the head mask within footprint_radius_m + 25 mm of c (base frame)."""
    mask, _ = head_mask(bgr, K, T_BC, c, g)
    vv, uu = np.nonzero(np.isfinite(depth) & (depth > 0) & ~mask)
    z = depth[vv, uu].astype(float)
    pb = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1) @ T_BC[:3, :3].T + T_BC[:3, 3]
    return pb[np.linalg.norm(pb[:, :2] - c[:2], axis=1) < g["footprint_radius_m"] + 0.025]


def wide_face_offset(G, yaw_ref, offsets_deg, min_tilt_deg=55.0):
    """Which of the jaw offsets lands on the plug's WIDER faces. The sticker sits on a wide face (the plug is 15.0 x 7.5 mm, STEP model), so the horizontal direction of the
    sticker plane's normal is the jaw direction for the wide-face grasp. G = sticker 3D points (base frame), yaw_ref = jaw axis at the home wrist orientation (rad).
    Returns (offset_deg or None, normal azimuth deg, plane tilt deg from horizontal); None when the plane is not steep enough to trust (< min_tilt_deg)."""
    _, _, vt = np.linalg.svd(G - G.mean(0), full_matrices=False)
    n = vt[2]
    tilt = float(np.degrees(np.arccos(abs(n[2]))))
    az = float(np.arctan2(n[1], n[0]) % np.pi)
    if tilt < min_tilt_deg:
        return None, float(np.degrees(az)), tilt
    dist = lambda o: abs(((az - (yaw_ref + np.radians(o)) + np.pi / 2) % np.pi) - np.pi / 2)
    return float(min(offsets_deg, key=dist)), float(np.degrees(az)), tilt


def grid_grasp_search(P, N, S, G, c, g, finger_w, finger_t, max_open, margin, drop, yaw_ref=None):
    """Image mode grasp search from the head cloud P (base frame, N = outward normals, S = non-head scene points, G = sticker points, c = sticker median).
    1. Grid: every g['grid_m'] over the head's xy extent, kept where the head cloud is (jaw centre).
    2. Yaw: every g['yaw_step_deg'] over 180 deg at each grid point (jaw axis b, pad strip along a). With yaw_ref (rad, the jaw axis at the home wrist orientation) and
       g['jaw_yaw_offsets_deg'] (e.g. [0, 90]) ONLY those jaw yaws are tried: wrist joint 6 at its home value, or +-90 deg (the same line, tried as the two jaw flips).
    3. Pad window: head points within +- finger_w/2 along a. Jaw gap = 2nd..98th percentile extent along b; jaw centre must sit within g['center_tol_m'] of the
       middle of that gap; opening = width + 2 margins <= max_open.
    4. Filters: >= g['min_window_points'] points; contact bands (g['contact_band_m'] at each end) hold >= 3 points.
    4b. Depth: every geometry above is tried at each g['extra_depth_m'] (e.g. 3..7 mm) BELOW the usual depth (sticker height - `drop`); at each depth both finger
       boxes (finger_t x finger_w, from g['finger_clearance_m'] below the tip z up g['finger_box_h_m']) may hit <= g['max_finger_hits'] scene points (the panel top rejects a depth that is too deep).
    5. Cost = w_anti x (1 - antipodal) + w_even x width unevenness + w_center x distance to the sticker / 10 mm + the three "grip quality" terms, each 0 for the best candidate and 1 for the worst: w_depth x (1 - extra depth / deepest extra depth) (deeper = more wall in the pads, within the collision limit), w_contact x (1 - contact points / most contact points) (covering more points) and w_face x (1 - face length / longest face length) (the wall the pad lands on is long along the pad: the wider face), where
       antipodal = mean of (outward normal of each contact band . its jaw direction), contact = min points of the two bands.
    Returns (candidates best first, stats). Candidate: dict(p, jaw (horizontal unit 3-vector), width_m, open_width_m, cost, offset_m (distance from the sticker),
    yaw_deg, z_top_m = sticker median height, depth_extra_m, anti, even, contact, face_len_m = length along the pad strip of the wall under the thinner pad, hits). stats: counts after every step."""
    from scipy.spatial import cKDTree
    xy, tree = P[:, :2], cKDTree(P[:, :2])
    gx, gy = np.meshgrid(np.arange(xy[:, 0].min(), xy[:, 0].max() + 1e-9, g["grid_m"]), np.arange(xy[:, 1].min(), xy[:, 1].max() + 1e-9, g["grid_m"]))
    grid = np.c_[gx.ravel(), gy.ravel()]
    grid = grid[tree.query(grid)[0] < g["grid_m"]]
    if yaw_ref is not None and g.get("jaw_yaw_offsets_deg"):
        yaws = np.unique(np.round((yaw_ref + np.deg2rad(g["jaw_yaw_offsets_deg"])) % np.pi, 9))
    else:
        yaws = np.deg2rad(np.arange(0.0, 180.0, g["yaw_step_deg"]))
    A, B = np.stack([-np.sin(yaws), np.cos(yaws)], 1), np.stack([np.cos(yaws), np.sin(yaws)], 1)   # pad-strip axis a, jaw axis b per yaw
    extras = list(g["extra_depth_m"])
    L = len(extras)   # every (grid point, yaw) is tried at L depths; an early filter rejects all L at once
    st = dict(grid_points=len(grid), yaws=len(yaws), depth_levels=L, generated=len(grid) * len(yaws) * L, few_points=0, centre_off=0, too_wide=0, no_contact=0, collision=0)
    out = []
    for q in grid:
        rel = xy - q
        Sa, Tb = rel @ A.T, rel @ B.T
        srel = S[:, :2] - q
        Sa_s, Tb_s = srel @ A.T, srel @ B.T
        for k in range(len(yaws)):
            win = np.abs(Sa[:, k]) <= finger_w / 2
            if win.sum() < g["min_window_points"]:
                st["few_points"] += L
                continue
            t = Tb[win, k]
            lo, hi = np.percentile(t, [2, 98])
            if abs(lo + hi) / 2 > g["center_tol_m"]:
                st["centre_off"] += L
                continue
            width = float(hi - lo)
            if width + 2 * margin > max_open:
                st["too_wide"] += L
                continue
            idx = np.nonzero(win)[0]
            left, right = idx[t < lo + g["contact_band_m"]], idx[t > hi - g["contact_band_m"]]
            if len(left) < 3 or len(right) < 3:
                st["no_contact"] += L
                continue
            b = B[k]
            anti = 0.5 * (np.clip(-(N[left, :2] @ b), 0, 1).mean() + np.clip(N[right, :2] @ b, 0, 1).mean())
            z_top = float(c[2])   # sticker median height: the 3D sticker points ramp ~27 mm over 11 mm (depth artefact), a per-window height would move the TCP with the window
            ow = width + 2 * margin
            cb = g["contact_band_m"]
            face = float(min(np.ptp(Sa[m_, k]) if m_.sum() >= 3 else 0.0 for m_ in ((Tb[:, k] > lo - cb) & (Tb[:, k] < lo + cb), (Tb[:, k] > hi - cb) & (Tb[:, k] < hi + cb))))   # wall length along the pad strip at each end of the gap, the shorter one
            sw = np.abs(Sa_s[:, k]) <= finger_w / 2
            sz = S[:, 2]
            side = sw & (np.abs(Tb_s[:, k]) >= ow / 2) & (np.abs(Tb_s[:, k]) <= ow / 2 + finger_t)
            order = np.argsort(Sa[win, k])
            even = float(np.std([np.subtract(*np.percentile(t[o], [98, 2])) for o in np.array_split(order, 3)]) / width)
            mid = np.array([q[0] + b[0] * (lo + hi) / 2, q[1] + b[1] * (lo + hi) / 2])
            for ex in extras:
                z_tcp = z_top - drop - ex
                hits = int((side & (sz >= z_tcp - g["finger_clearance_m"]) & (sz <= z_tcp + g["finger_box_h_m"])).sum())   # box also reaches finger_clearance_m below the tip: the tip must stay that far above the panel
                if hits > g["max_finger_hits"]:
                    st["collision"] += 1
                    continue
                out.append(dict(p=np.r_[mid, z_tcp], jaw=np.r_[b, 0.0], width_m=width, open_width_m=ow, offset_m=float(np.linalg.norm(mid - c[:2])),
                                yaw_deg=float(np.rad2deg(yaws[k])), z_top_m=z_top, depth_extra_m=float(ex), anti=float(anti), even=even,
                                contact=int(min(len(left), len(right))), face_len_m=face, hits=hits))
    st["valid"] = len(out)
    if out:
        cmax, fmax = max(q["contact"] for q in out), max(max(q["face_len_m"] for q in out), 1e-9)
        wide = g.get("wide_face_offset_deg")
        wide = None if isinstance(wide, str) else wide   # "auto" is resolved by the caller (wide_face_offset); unresolved = measure the face length
        if yaw_ref is not None and wide is not None:   # the offset that lands on the wider faces is known (config): the face term is 0 there and 1 on the other family
            for q in out:
                q["face_pen"] = 0.0 if abs(((np.radians(q["yaw_deg"]) - yaw_ref - np.radians(wide) + np.pi / 2) % np.pi) - np.pi / 2) < 0.02 else 1.0
        else:
            for q in out:
                q["face_pen"] = 1 - q["face_len_m"] / fmax
        for q in out:
            q["cost"] = float(g["w_anti"] * (1 - q["anti"]) + g["w_even"] * q["even"] + g["w_center"] * q["offset_m"] / 0.010
                              + g["w_depth"] * (1 - q["depth_extra_m"] / max(extras)) + g["w_contact"] * (1 - q["contact"] / cmax) + g["w_face"] * q["face_pen"])
    return sorted(out, key=lambda d: d["cost"]), st


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
