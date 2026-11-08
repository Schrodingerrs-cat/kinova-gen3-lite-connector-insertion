"""Stage 1: depth image -> clean point cloud in the robot base frame B (meters)."""
import numpy as np
from scipy.ndimage import maximum_filter, minimum_filter
from scipy.spatial import cKDTree


def cloud_from_depth(depth_m, K, T_BC, roi_px, margin_px=120, stride=1, edge_jump_m=0.003):
    """Back-project an expanded ROI window. Returns (points_B Nx3, in_roi N bool = central half of the ROI box).

    Flying pixels (3x3 depth range > edge_jump_m, i.e. object borders and walls seen edge-on)
    are dropped: the solid part is then described by its top surfaces only.
    """
    u0, v0, u1, v1 = roi_px
    h, w = depth_m.shape
    a, b = max(int(v0 - margin_px), 0), min(int(v1 + margin_px), h)
    c, d = max(int(u0 - margin_px), 0), min(int(u1 + margin_px), w)
    win = depth_m[a:b, c:d]
    valid = np.isfinite(win) & (win > 0)
    hi = maximum_filter(np.where(valid, win, 0.0), 3)
    lo = minimum_filter(np.where(valid, win, np.inf), 3)
    valid &= (hi - lo) <= edge_jump_m
    vv, uu = np.mgrid[a:b, c:d]
    vv, uu, z = vv[valid][::stride], uu[valid][::stride], win[valid][::stride]
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1)
    pb = pc @ T_BC[:3, :3].T + T_BC[:3, 3]
    # the object is whatever stands at the CENTER of the ROI (central half of the box)
    cu, cv, hu, hv = (u0 + u1) / 2, (v0 + v1) / 2, (u1 - u0) / 4, (v1 - v0) / 4
    in_roi = (np.abs(uu - cu) < hu) & (np.abs(vv - cv) < hv)
    return pb, in_roi


def voxel_downsample(points, in_roi, size):
    """One averaged point per voxel (uniform density); a voxel is in the ROI if any of its points is."""
    key = np.floor(points / size).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    pts = np.stack([np.bincount(inv, points[:, i]) for i in range(3)], 1) / cnt[:, None]
    roi = np.bincount(inv, in_roi.astype(float)) > 0
    return pts, roi


def remove_outliers(points, in_roi, radius, min_neighbors):
    """Drop isolated points (sensor speckle): fewer than min_neighbors others within radius."""
    nb = cKDTree(points).query_ball_point(points, radius, return_length=True) - 1
    keep = nb >= min_neighbors
    return points[keep], in_roi[keep]


def clean_cloud(points, in_roi, cp):
    """cp = config['cloud']: voxel_m, outlier_radius_m, outlier_min_nb."""
    pts, roi = voxel_downsample(points, in_roi, cp["voxel_m"])
    return remove_outliers(pts, roi, cp["outlier_radius_m"], cp["outlier_min_nb"])
