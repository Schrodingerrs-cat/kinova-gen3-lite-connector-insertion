"""Undamped IBVS with image center/size features and final corner features."""
import cv2
import numpy as np

from .control import interaction


def center_size_features(xy):
    xy = np.asarray(xy, float)
    if xy.shape != (4, 2) or not np.isfinite(xy).all():
        raise ValueError("four finite corresponding corners are required")
    following, preceding = np.roll(xy, -1, axis=0), np.roll(xy, 1, axis=0)
    area = .5 * np.sum(xy[:, 0]*following[:, 1]-xy[:, 1]*following[:, 0])
    if abs(area) < 1e-10:
        raise ValueError("rectangle area is too small")
    derivative = np.zeros((3, 8))
    derivative[0, 0::2], derivative[1, 1::2] = .25, .25
    derivative[2, 0::2] = (following[:, 1]-preceding[:, 1])/(4*area)
    derivative[2, 1::2] = (preceding[:, 0]-following[:, 0])/(4*area)
    return np.r_[xy.mean(axis=0), .5*np.log(abs(area))], derivative


def shape_metrics(pixels, target):
    current, _ = center_size_features(pixels)
    desired, _ = center_size_features(target)
    errors = np.linalg.norm(np.asarray(pixels)-np.asarray(target), axis=1)
    return dict(center_error_px=float(np.linalg.norm(current[:2]-desired[:2])),
                size_ratio=float(np.exp(current[2]-desired[2])),
                rms_px=float(np.sqrt(np.mean(errors**2))), max_corner_px=float(errors.max()))


def task_cost(xy, target, focal, task):
    if task == "corners":
        return float(np.mean(np.sum(((xy-target)*focal)**2, axis=1)))
    current, _ = center_size_features(xy)
    desired, _ = center_size_features(target)
    return float(np.sum(((current[:2]-desired[:2])*focal/5.)**2)
                 + ((current[2]-desired[2])/.1)**2)


def shape_ibvs_step(xy, target, depths, camera_jacobian, gain, task,
                    joint_weights, joint_limits_rad, max_translation, max_rotation):
    """One undamped image-task pseudoinverse, with uniform motion limiting."""
    xy, target = np.asarray(xy, float), np.asarray(target, float)
    depths, camera_jacobian = np.asarray(depths, float), np.asarray(camera_jacobian, float)
    weights, limits = np.asarray(joint_weights, float), np.asarray(joint_limits_rad, float)
    if (xy.shape != (4, 2) or target.shape != xy.shape or depths.shape != (4,)
            or camera_jacobian.shape != (6, 6) or weights.shape != (6,) or limits.shape != (6,)):
        raise ValueError("invalid image task or robot Jacobian shape")
    if (not all(np.isfinite(v).all() for v in (xy, target, depths, camera_jacobian, weights, limits))
            or np.any(depths <= 0) or np.any(weights <= 0) or np.any(limits <= 0)
            or not all(np.isfinite(v) and v > 0 for v in (gain, max_translation, max_rotation))):
        raise ValueError("finite observations and positive depths/gains/limits are required")
    J = interaction(xy, depths) @ camera_jacobian
    if task == "center_size":
        current, derivative = center_size_features(xy)
        desired, _ = center_size_features(target)
        J, error = derivative @ J, current-desired
    elif task == "corners":
        error = (xy-target).ravel()
    else:
        raise ValueError("unknown image task")
    scale = np.diag(weights)
    dq = -gain * scale @ np.linalg.pinv(J @ scale) @ error
    camera_delta = camera_jacobian @ dq
    factor = min(1., float(np.min(limits/np.maximum(np.abs(dq), 1e-12))),
                 max_translation/max(np.linalg.norm(camera_delta[:3]), 1e-12),
                 max_rotation/max(np.linalg.norm(camera_delta[3:]), 1e-12))
    return dq*factor


def project_motion(xy, depths, relative_camera, K, distortion):
    points = np.column_stack([xy, np.ones(4)])*np.asarray(depths)[:, None]
    points = points @ relative_camera[:3, :3].T + relative_camera[:3, 3]
    if np.any(points[:, 2] <= 0):
        raise ValueError("proposed feature lies behind the camera")
    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                                 np.asarray(K, float), np.asarray(distortion, float))
    return pixels.reshape(4, 2)


STEP_SCALES = (1., .5, .25, .1, .05, .01)
