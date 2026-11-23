import numpy as np
from scipy.spatial.transform import Rotation as Rot


def matrix(xyz, quat_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rot.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = xyz
    return T


def skew(t):
    return np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])


def adjoint(T):
    """Twist [v, w] in frame B -> same rigid body twist in frame A, T = T_AB (B pose in A)."""
    R, t = T[:3, :3], T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3], Ad[:3, 3:], Ad[3:, 3:] = R, skew(t) @ R, R
    return Ad
