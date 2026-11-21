"""Forward move after alignment: wall plane from depth, clearance of whatever the gripper holds, travel plan."""
import cv2
import numpy as np


def backproject(depth_m, mask, K, distortion):
    """Camera-frame points (n,3) of the masked pixels that have depth."""
    v, u = np.nonzero(mask & np.isfinite(depth_m) & (depth_m > 0))
    if not len(u):
        return np.zeros((0, 3))
    xy = cv2.undistortPoints(np.column_stack([u, v]).astype(np.float64).reshape(-1, 1, 2),
                             np.asarray(K, float), np.asarray(distortion, float)).reshape(-1, 2)
    return np.column_stack([xy, np.ones(len(xy))]) * depth_m[v, u, None]


def fit_wall(depth_m, region, exclude, K, distortion, lo, hi, inlier_m=.004, min_points=3000, min_fraction=.4):
    """RANSAC plane over valid depth in region (x0, y0, x1, y1), skipping `exclude` pixels.
    Returns unit normal n pointing at the camera and d, so n.P + d is the distance of P to the wall."""
    x0, y0, x1, y1 = region
    mask = np.zeros(depth_m.shape, bool)
    mask[max(0, y0):y1, max(0, x0):x1] = True
    mask &= ~exclude & (depth_m >= lo) & (depth_m <= hi)
    P = backproject(depth_m, mask, K, distortion)
    if len(P) < min_points:
        raise ValueError("wall not measured: %d valid depth points (need %d)" % (len(P), min_points))
    rng, best = np.random.default_rng(0), None
    for _ in range(300):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        inliers = np.abs((P - a) @ (n / np.linalg.norm(n))) < inlier_m
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < min_points or best.mean() < min_fraction:
        raise ValueError("wall not measured: no plane with enough support")
    Q = P[best]
    n = np.linalg.svd(Q - Q.mean(0), full_matrices=False)[2][-1]
    n = n if n[2] < 0 else -n
    return n, float(-n @ Q.mean(0)), int(best.sum())


def clearance(points, n, d):
    """Smallest distance (m) from any point to the wall plane; negative = already past it."""
    return float((np.asarray(points) @ n + d).min())


def plan_travel(clearance_m, requested_m, margin_m, allow_contact):
    """(travel, trimmed). Without allow_contact the travel stops margin_m short of the wall."""
    if allow_contact:
        return requested_m, False
    planned = max(0., min(requested_m, clearance_m - margin_m))
    return planned, planned < requested_m
