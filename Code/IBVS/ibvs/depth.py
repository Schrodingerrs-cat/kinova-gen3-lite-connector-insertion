import numpy as np
import cv2


def observed_voxels(depth_m, K, distortion, camera_in_base, lo, hi, voxel=.01, stride=4):
    """Freeze measured environment surfaces in base coordinates; no extrapolated geometry."""
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = depth_m[::stride, ::stride]
    valid = np.isfinite(z) & (z >= lo) & (z <= hi)
    pixels = np.column_stack([u[valid], v[valid]]).astype(float)
    if not len(pixels):
        raise ValueError('no measured environment depth for clearance check')
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), np.asarray(K), np.asarray(distortion)).reshape(-1, 2)
    points = np.column_stack([rays, np.ones(len(rays))])*z[valid, None]
    points = points @ camera_in_base[:3, :3].T + camera_in_base[:3, 3]
    return (np.unique(np.floor(points/voxel).astype(np.int64), axis=0)+.5)*voxel


def validate_frozen_target(snapshot, camera_in_base, pixels, measured_depth, max_pixels=20., max_depth=.02):
    """Require the current target to agree with the frozen scene before reusing it."""
    K, D = snapshot['K'], snapshot['distortion']
    rays = cv2.undistortPoints(snapshot['snapshot_corners_px'].reshape(4, 1, 2).astype(float), K, D).reshape(4, 2)
    points = np.column_stack([rays, np.ones(4)])*snapshot['snapshot_depths_m'][:, None]
    relative = np.linalg.inv(camera_in_base) @ snapshot['camera_in_base']
    points = points @ relative[:3, :3].T + relative[:3, 3]
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        raise RuntimeError('frozen target is not in front of the current camera')
    predicted, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, D)
    error = float(np.linalg.norm(predicted.reshape(4, 2)-pixels, axis=1).max())
    depth_error = abs(float(points[:, 2].mean())-measured_depth)
    if not np.isfinite(error+depth_error) or error > max_pixels or depth_error > max_depth:
        raise RuntimeError('target no longer matches frozen scene: %.1f px / %.1f mm' % (error, depth_error*1000))
    return dict(max_corner_difference_px=error, mean_depth_difference_m=depth_error)


def corner_depths(depth_m, px, lo, hi, patch=2):
    """Median depth in a (2*patch+1)^2 window per corner. px (n,2). Invalid (0, NaN, out of [lo,hi]) -> NaN."""
    h, w = depth_m.shape
    out = np.full(len(px), np.nan)
    for i, (u, v) in enumerate(px):
        u, v = int(round(u)), int(round(v))
        win = depth_m[max(0, v - patch):min(h, v + patch + 1), max(0, u - patch):min(w, u + patch + 1)]
        ok = win[np.isfinite(win) & (win >= lo) & (win <= hi)]
        if ok.size > win.size // 2:
            out[i] = np.median(ok)
    return out


def fill_depths(Z):
    """Invalid corners take the median of the valid ones; None if no corner is valid (abort)."""
    ok = np.isfinite(Z)
    if not ok.any():
        return None
    return np.where(ok, Z, np.median(Z[ok]))


def scene_near_depth(settings, T_EC, T_tcp):
    """Nearest range (m) from the camera that is treated as scene: closer returns are the gripper and what it holds."""
    tcp_in_camera = np.linalg.inv(T_EC) @ T_tcp
    return settings.get('scene_min_depth_m', max(.18, float(tcp_in_camera[2, 3]) + .04))
