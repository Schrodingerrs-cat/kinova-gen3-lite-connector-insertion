"""IBVS law for point features on the normalized image plane.

s = (x, y) = ((u-cx)/fx, (v-cy)/fy). Camera twist xi = [vx vy vz wx wy wz] in the camera frame gives ds/dt = L xi.
Only 3 DoF are driven (lateral x_S, y_S and yaw about z_S, all in camera coordinates): xi = A u, u = -lambda (L A)^+ W e.
Depth Z enters L only, not the error. Labelled hybrid IBVS (image error + measured depth in the interaction matrix).
"""
import numpy as np


def interaction(xy, Z):
    """Stacked 2x6 point-feature blocks for points xy (n,2) at depths Z (n,)."""
    L = np.zeros((2 * len(xy), 6))
    for i, ((x, y), z) in enumerate(zip(xy, Z)):
        L[2 * i] = [-1 / z, 0, x / z, x * y, -(1 + x * x), y]
        L[2 * i + 1] = [0, -1 / z, y / z, 1 + y * y, -x * y, -x]
    return L


def axes_matrix(socket_axes_in_C):
    """6x3: columns = lateral x_S, lateral y_S, yaw about z_S."""
    R = np.asarray(socket_axes_in_C, float)
    A = np.zeros((6, 3))
    A[:3, 0], A[:3, 1], A[3:, 2] = R[:, 0], R[:, 1], R[:, 2]
    return A


def damped_pinv(J, damping):
    return J.T @ np.linalg.inv(J @ J.T + damping ** 2 * np.eye(J.shape[0]))


def servo_twist(xy, xy_star, Z, A, lam, damping, max_cond, weights=None):
    """Rates u = (lateral_x, lateral_y, yaw) (3,), or None if J is ill-conditioned. weights (n,) per feature, default 1."""
    e = (np.asarray(xy) - np.asarray(xy_star)).ravel()
    w = np.ones(len(xy)) if weights is None else np.asarray(weights, float)
    W = np.repeat(w, 2)
    J = (interaction(xy, Z) @ A) * W[:, None]
    if np.linalg.cond(J) > max_cond:
        return None
    return -lam * damped_pinv(J, damping) @ (W * e)


def limit(u3, prev3, lin_max, yaw_max, acc, dt):
    """u3 = (lateral_x, lateral_y, yaw) rates. Cap magnitude, then slew-limit vs the previous command."""
    u = np.array(u3, float)
    n = np.linalg.norm(u[:2])
    if n > lin_max:
        u[:2] *= lin_max / n
    u[2] = np.clip(u[2], -yaw_max, yaw_max)
    return prev3 + np.clip(u - prev3, -acc * dt, acc * dt)


class Convergence:
    def __init__(self, tol, frames):
        self.tol, self.frames, self.n = tol, frames, 0

    def update(self, rms_px):
        self.n = self.n + 1 if rms_px < self.tol else 0
        return self.n >= self.frames
