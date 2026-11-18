"""Full-camera-pose IBVS steps for the free-space reference-image test."""
import numpy as np
from scipy.linalg import expm, logm

from .control import damped_pinv, interaction
from .frames import skew


def marker_plane_depths(depth, pixels, K, distortion, lo, hi, inlier_m, min_points, min_fraction):
    """Fit the dominant measured depth plane inside the marker; reject unsupported fits."""
    import cv2

    pixels = np.asarray(pixels, float)
    mask = np.zeros(depth.shape, np.uint8)
    inner = pixels.mean(axis=0) + 0.8 * (pixels - pixels.mean(axis=0))
    cv2.fillConvexPoly(mask, np.rint(inner).astype(np.int32), 1)
    v, u = np.nonzero(mask & np.isfinite(depth) & (depth >= lo) & (depth <= hi))
    if len(u) < min_points:
        raise ValueError("too few valid marker depth pixels")
    stride = max(1, len(u) // 800)
    u, v = u[::stride], v[::stride]
    rays = cv2.undistortPoints(np.column_stack([u, v]).astype(float).reshape(-1, 1, 2),
                               np.asarray(K, float), np.asarray(distortion, float)).reshape(-1, 2)
    points = np.column_stack([rays, np.ones(len(rays))]) * depth[v, u, None]
    rng, best = np.random.default_rng(0), None
    for _ in range(100):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-10:
            continue
        normal /= length
        inliers = np.abs((points - a) @ normal) < inlier_m
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < min_points or best.mean() < min_fraction:
        raise ValueError("marker depth has no sufficiently supported plane")
    selected = points[best]
    center = selected.mean(axis=0)
    _, _, vectors = np.linalg.svd(selected - center, full_matrices=False)
    normal = vectors[-1]
    corner_rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), np.asarray(K, float),
                                     np.asarray(distortion, float)).reshape(-1, 2)
    denominator = np.column_stack([corner_rays, np.ones(4)]) @ normal
    if np.any(np.abs(denominator) < 1e-6):
        raise ValueError("marker depth plane is parallel to a corner ray")
    depths = (center @ normal) / denominator
    if not np.isfinite(depths).all() or np.any(depths < lo) or np.any(depths > hi):
        raise ValueError("fitted corner depths are outside the configured depth range")
    return depths, dict(samples=len(points), inliers=int(best.sum()), inlier_fraction=float(best.mean()))


def camera_step(xy, target, depths, gain, damping, max_translation, max_rotation):
    """Return a camera-frame SE(3) increment and the predicted image change.

    Error is measured in the image. Depth only scales the interaction matrix.
    Translation is preconditioned by median depth to balance metres and radians.
    """
    xy, target, depths = np.asarray(xy), np.asarray(target), np.asarray(depths)
    if xy.shape != (4, 2) or target.shape != xy.shape or depths.shape != (4,):
        raise ValueError("four corresponding image corners and depths are required")
    if not all(np.isfinite(a).all() for a in (xy, target, depths)) or np.any(depths <= 0):
        raise ValueError("features and depths must be finite, with positive depths")
    if not all(np.isfinite(v) and v > 0 for v in (gain, damping, max_translation, max_rotation)):
        raise ValueError("step settings must be finite and positive")
    L = interaction(xy, depths)
    scale = np.diag([float(np.median(depths))] * 3 + [1.0] * 3)
    J = L @ scale
    if np.linalg.matrix_rank(J) < 6:
        raise ValueError("marker geometry does not constrain all six camera axes")
    delta = -gain * scale @ damped_pinv(J, damping) @ (xy - target).ravel()
    factor = min(1.0, max_translation / max(np.linalg.norm(delta[:3]), 1e-12),
                 max_rotation / max(np.linalg.norm(delta[3:]), 1e-12))
    delta *= factor
    generator = np.zeros((4, 4))
    generator[:3, :3], generator[:3, 3] = skew(delta[3:]), delta[:3]
    return expm(generator), (L @ delta).reshape(4, 2)


def camera_joint_jacobian(fk, q, T_EC, epsilon=1e-5):
    """Camera body Jacobian from robot-model FK; no physical probe motion."""
    q = np.asarray(q, float)
    origin = np.linalg.inv(fk(q) @ T_EC)
    J = np.zeros((6,6))
    for index in range(6):
        shifted = q.copy()
        shifted[index] += epsilon
        generator = logm(origin @ fk(shifted) @ T_EC).real / epsilon
        J[:,index] = np.r_[generator[:3,3], generator[2,1], generator[0,2], generator[1,0]]
    return J


def fit_camera_tool_mapping(tool_twists, feature_changes, xy, depths):
    """Identify the local rigid camera/tool mapping from measured probe motions."""
    X, Y = np.asarray(tool_twists, float), np.asarray(feature_changes, float)
    scale = np.array([0.004]*3 + [np.radians(1.0)]*3)
    if X.ndim != 2 or X.shape[1] != 6 or Y.shape != (len(X), 8) or len(X) < 12:
        raise ValueError("at least twelve measured six-axis probes are required")
    if not np.isfinite(X).all() or not np.isfinite(Y).all() or np.linalg.matrix_rank(X / scale) < 6:
        raise ValueError("probe motions do not independently excite all six axes")
    J_tool = np.linalg.lstsq(X / scale, Y, rcond=None)[0].T / scale
    L = interaction(xy, depths)
    rotation_raw = np.linalg.lstsq(L[:, :3], J_tool[:, :3], rcond=None)[0]
    U, singular, Vt = np.linalg.svd(rotation_raw)
    R_CE = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    residual = J_tool[:, 3:] - L[:, 3:] @ R_CE
    basis = np.column_stack([(-L[:, :3] @ R_CE @ skew(axis)).ravel() for axis in np.eye(3)])
    offset = np.linalg.lstsq(basis, residual.ravel(), rcond=None)[0]
    predicted = np.column_stack([L[:, :3] @ R_CE, -L[:, :3] @ R_CE @ skew(offset) + L[:, 3:] @ R_CE])
    relative_error = float(np.linalg.norm(Y - X @ predicted.T) / max(np.linalg.norm(Y), 1e-12))
    if singular.min() < 0.4 or singular.max() > 1.8 or relative_error > 0.30 or np.linalg.norm(offset) > 0.25:
        raise ValueError("local camera/tool fit is unsupported (relative residual %.3f, singular values %s, offset %.3f m)" %
                         (relative_error, np.round(singular, 3), np.linalg.norm(offset)))
    T_EC = np.eye(4)
    T_EC[:3, :3], T_EC[:3, 3] = R_CE.T, offset
    return T_EC, dict(relative_residual=relative_error, translation_singular_values=singular.tolist(),
                      offset_norm_m=float(np.linalg.norm(offset)), probe_count=len(X))


def path_times(q_rad, speed_deg_s, min_interval=0.05):
    """Timing for quintic joint segments with zero velocity/acceleration at knots."""
    q = np.asarray(q_rad, float)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError("a finite six-joint path with at least two points is required")
    if not np.isfinite(speed_deg_s) or speed_deg_s <= 0:
        raise ValueError("joint speed must be positive")
    distance = np.degrees(np.abs(np.diff(q, axis=0))).max(axis=1)
    return np.r_[0.1, 0.1 + np.cumsum(np.maximum(min_interval, 1.875 * distance / speed_deg_s))]


def within_limits(T_BE, T_EC, T_tcp, origin_BC, floor_z, max_travel, max_angle):
    """Geometric checks for an alignment target, relative to its starting camera pose."""
    from scipy.spatial.transform import Rotation

    camera = T_BE @ T_EC
    if min((T_BE @ T_tcp)[2, 3], camera[2, 3]) < floor_z:
        raise ValueError("camera or fingertip would cross the configured table clearance")
    if origin_BC is not None:
        if np.linalg.norm(camera[:3, 3] - origin_BC[:3, 3]) > max_travel:
            raise ValueError("alignment target exceeds the camera travel limit")
        if Rotation.from_matrix(origin_BC[:3, :3].T @ camera[:3, :3]).magnitude() > max_angle:
            raise ValueError("alignment target exceeds the camera rotation limit")
