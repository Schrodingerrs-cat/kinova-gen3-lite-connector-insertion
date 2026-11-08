"""Green grasp markers: colour mask -> green point cloud -> table removal -> clusters -> the two marker planes -> grasp.

The grasp is searched ONLY on the green points (the two marked side planes of the connector head).
"""
import cv2
import numpy as np
from scipy.ndimage import maximum_filter, minimum_filter
from sklearn.cluster import DBSCAN, KMeans

from .cloud import remove_outliers, voxel_downsample


def green_mask(bgr, g):
    """Bool mask of green marker pixels (OpenCV HSV: g['h'] range, saturation and value minimums)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] >= g["h"][0]) & (hsv[..., 0] <= g["h"][1]) & (hsv[..., 1] >= g["s_min"]) & (hsv[..., 2] >= g["v_min"])
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)


def green_points(depth, bgr, K, T_BC, g):
    """Green pixels -> cleaned base-frame points. Flying pixels are removed on the FULL depth before the mask is applied."""
    valid = np.isfinite(depth) & (depth > 0)
    hi = maximum_filter(np.where(valid, depth, 0.0), 3)
    lo = minimum_filter(np.where(valid, depth, np.inf), 3)
    valid &= (hi - lo) <= g["edge_jump_m"]
    vv, uu = np.nonzero(valid & green_mask(bgr, g))
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
    """Split the green points into the two marker planes with KMeans(k=2) on their horizontal normals.
    Returns [(points, centroid, normal), (points, centroid, normal)] with the normals pointing away from each other, or None."""
    N = normals(G, g["normal_knn"])
    N = N * np.where(np.einsum("ij,ij->i", N, G - G.mean(0)) < 0, -1.0, 1.0)[:, None]  # outward from the green centre
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


def green_frame(C):
    """Frame of one green patch from PCA: long axis along it, jaw axis across it, normal = patch plane normal (up)."""
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
    """Merged-scan stages: table RANSAC -> above-table object points + clusters -> green clusters on the object -> frame of the largest."""
    g, tz = cfg["green"], cfg["scan"]["table_z_m"]
    band = P[np.abs(P[:, 2] - tz) < 0.03]
    sub = band[np.random.default_rng(0).choice(len(band), min(150000, len(band)), replace=False)]
    n, d, n_inl = ransac_plane(sub, g["table_thresh_m"])
    x0, x1, y0, y1 = cfg["heads"]["region"]
    h = P @ n + d
    inbox = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
    obj, _ = voxel_downsample(P[inbox & (h > g["min_above_table_m"]) & (h < g["max_above_table_m"])], np.zeros(int((inbox & (h > g["min_above_table_m"]) & (h < g["max_above_table_m"])).sum()), bool), 0.003)
    hg = G @ n + d
    Ga = G[(hg > g["min_above_table_m"]) & (hg < g["max_above_table_m"])]
    glab = dbscan(Ga, g["green_eps_m"], g["green_min_pts"])
    clusters = sorted((Ga[glab == k] for k in set(glab) - {-1}), key=len, reverse=True)
    return dict(table_n=n, table_d=d, table_inliers=n_inl, object_points=obj, object_labels=dbscan(obj, g["cluster_eps_m"], g["cluster_min_pts"]),
                green_above=Ga, green_labels=glab, green_clusters=clusters, frame=green_frame(clusters[0]) if clusters else None)


def housing_footprint(bgr, depth, K, T_BC, c, g):
    """Top-face footprint of the dark housing around the green sticker (colour image, back-projected at the sticker's depth).
    c = sticker centroid in the base frame. Returns dict(long_axis, jaw_axis (unit xy in base), length_m, width_m, n_px)."""
    cam = np.linalg.inv(T_BC) @ np.r_[c, 1]
    Z0 = cam[2]
    u0, v0 = K[0, 0] * cam[0] / Z0 + K[0, 2], K[1, 1] * cam[1] / Z0 + K[1, 2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    body = (hsv[..., 2] < g["dark_v"]) | green_mask(bgr, g)
    vv, uu = np.mgrid[0:bgr.shape[0], 0:bgr.shape[1]]
    body &= np.hypot(uu - u0, vv - v0) < g["footprint_radius_m"] * K[0, 0] / Z0
    body = cv2.morphologyEx(body.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(body)
    k = lab[int(round(v0)), int(round(u0))]
    if k == 0:
        raise RuntimeError("no housing pixels around the green sticker")
    vs, us = np.nonzero(lab == k)
    pc = np.stack([(us - K[0, 2]) * Z0 / K[0, 0], (vs - K[1, 2]) * Z0 / K[1, 1], np.full(len(us), Z0)], 1)
    pb = (pc @ T_BC[:3, :3].T + T_BC[:3, 3])[:, :2]
    m = pb.mean(0)
    ax = np.linalg.svd(pb - m, full_matrices=False)[2]
    ext = [float(np.subtract(*np.percentile((pb - m) @ a, [98, 2]))) for a in ax]
    return dict(long_axis=ax[0], jaw_axis=ax[1], length_m=ext[0], width_m=ext[1], n_px=int(len(us)), center_xy=m)
