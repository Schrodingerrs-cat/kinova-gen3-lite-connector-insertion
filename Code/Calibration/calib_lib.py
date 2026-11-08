"""Pure functions of the calibration tools (NumPy / SciPy / OpenCV, no ROS): poses, plane fit, hand-eye (Park-Martin), intrinsics, ArUco."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def to_T(xyz, quat):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rot.from_quat(quat).as_matrix(), xyz
    return T


def xyz_quat(T):
    return T[:3, 3].tolist(), Rot.from_matrix(T[:3, :3]).as_quat().tolist()


def wrap180(d):
    return (np.asarray(d, float) + 180.0) % 360.0 - 180.0


def view_poses(sc, T0, hand_eye_T):
    """Tool poses (base frame) for the scan views [x, y, camera_height (, tilt_deg (, yaw_deg))].

    Orientation = the tool orientation at scan start (home, camera looking down). A tilt turns the
    camera about a horizontal axis so it looks toward sc['look_at'] (workspace centre) from the side.
    The camera is placed at (x, y, table_z + height); tool position = camera - R @ hand-eye translation.
    """
    R_home, t_cam, ez = T0[:3, :3], hand_eye_T[:3, 3], hand_eye_T[:3, :3] @ [0.0, 0.0, 1.0]  # optical axis in tool frame
    poses = []
    for v in sc["views"]:
        x, y, h = v[:3]
        tilt = v[3] if len(v) > 3 else 0.0
        R0 = Rot.from_euler("z", v[4] if len(v) > 4 else 0.0, degrees=True).as_matrix() @ R_home  # yaw about base z
        cam = np.array([x, y, sc["table_z_m"] + h])
        R = R0
        if tilt:
            to = np.array([*sc["look_at"], sc["table_z_m"]]) - cam
            to[2] = 0.0
            to /= np.linalg.norm(to)
            axis = np.cross([0.0, 0.0, -1.0], to)
            for sign in (1, -1):  # pick the turn direction that tips the optical axis toward the workspace centre
                Rs = Rot.from_rotvec(sign * np.radians(tilt) * axis / np.linalg.norm(axis)).as_matrix() @ R0
                if (Rs @ ez) @ to > (R0 @ ez) @ to:
                    R = Rs
                    break
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, cam - R @ t_cam
        poses.append(T)
    return poses


# ---------------- table plane (calibration scan) ----------------
def fit_plane(P, thresh=0.003, iters=500, seed=0):
    """RANSAC plane n.p + d = 0 (n up) through the points P; refit on the inliers. Returns (n, d, n_inliers)."""
    rng, best, best_n = np.random.default_rng(seed), None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        k = int((np.abs((P - a) @ n) < thresh).sum())
        if k > best_n:
            best, best_n = (n, a), k
    n, a = best
    inl = np.abs((P - a) @ n) < thresh
    c = P[inl].mean(0)
    n = np.linalg.svd(P[inl] - c, full_matrices=False)[2][2]
    n = n if n[2] > 0 else -n
    return n, float(-n @ c), int(inl.sum())


def plane_tilt_height(n, d, xy):
    """Tilt of the plane from horizontal (deg) and its height z at the base-frame point xy."""
    return float(np.degrees(np.arccos(np.clip(n[2], -1, 1)))), float((-d - n[0] * xy[0] - n[1] * xy[1]) / n[2])


def backproject(depth, K, T_BC, step=4):
    """Valid depth pixels (every `step`-th) -> base-frame points."""
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[v, u].astype(float)
    ok = np.isfinite(z) & (z > 0.05) & (z < 1.5)
    p = np.stack([(u[ok] - K[0, 2]) * z[ok] / K[0, 0], (v[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]], 1)
    return p @ T_BC[:3, :3].T + T_BC[:3, 3]


# ---------------- hand-eye: A X = X B with X = tool_T_camera (Park-Martin) ----------------
def hand_eye(T_bt, T_cm):
    """T_bt[i] = base_T_tool, T_cm[i] = marker pose in the camera frame, marker FIXED in the base: T_bt X T_cm is the same for all i.
    Pairs (i, j): A = inv(T_bt_j) T_bt_i, B = T_cm_j inv(T_cm_i), A X = X B. Rotation from the log maps (Kabsch), translation by least squares.
    Needs at least 3 samples with rotations about at least 2 different axes. Returns X (4x4)."""
    A, B = [], []
    for i in range(len(T_bt)):
        for j in range(i + 1, len(T_bt)):
            A.append(np.linalg.inv(T_bt[j]) @ T_bt[i])
            B.append(T_cm[j] @ np.linalg.inv(T_cm[i]))
    al = np.array([Rot.from_matrix(a[:3, :3]).as_rotvec() for a in A])
    be = np.array([Rot.from_matrix(b[:3, :3]).as_rotvec() for b in B])
    U, _, Vt = np.linalg.svd(be.T @ al)
    R = Vt.T @ np.diag([1, 1, np.linalg.det(Vt.T @ U.T)]) @ U.T
    M = np.vstack([a[:3, :3] - np.eye(3) for a in A])
    y = np.concatenate([R @ b[:3, 3] - a[:3, 3] for a, b in zip(A, B)])
    t = np.linalg.lstsq(M, y, rcond=None)[0]
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t
    return X


def hand_eye_residual(T_bt, T_cm, X):
    """Where the (fixed) marker lands in the base frame for every sample: position spread (mm, std per axis and max distance from the mean) and
    orientation spread (deg, max angle from the mean). Zero for a perfect hand-eye."""
    Tm = [a @ X @ b for a, b in zip(T_bt, T_cm)]
    pos = np.array([T[:3, 3] for T in Tm])
    mean_R = Rot.from_matrix([T[:3, :3] for T in Tm]).mean()
    ang = [np.degrees((Rot.from_matrix(T[:3, :3]) * mean_R.inv()).magnitude()) for T in Tm]
    return dict(std_mm=(pos.std(0) * 1e3).tolist(), max_dev_mm=float(np.linalg.norm(pos - pos.mean(0), axis=1).max() * 1e3), max_angle_deg=float(max(ang)))


def pose_difference(Xa, Xb):
    """Translation (mm) and rotation (deg) between two transforms."""
    return float(np.linalg.norm(Xa[:3, 3] - Xb[:3, 3]) * 1e3), float(np.degrees((Rot.from_matrix(Xa[:3, :3]) * Rot.from_matrix(Xb[:3, :3]).inv()).magnitude()))


# ---------------- ArUco marker pose ----------------
def marker_pose(img, K, dist, dictionary, marker_id, size_m):
    """Pose of the ArUco marker in the camera frame (4x4) or None. size_m = side of the black square."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    if ids is None or marker_id not in ids.ravel():
        return None
    c = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)  # same order as the detector: TL, TR, BR, BL
    ok, rvec, tvec = cv2.solvePnP(obj, c.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return T


# ---------------- intrinsics (chessboard) ----------------
def chessboard_points(cols, rows, square_m):
    g = np.zeros((rows * cols, 3), np.float32)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return g


def intrinsics(obj_pts, img_pts, size_wh):
    """cv2.calibrateCamera. Returns (rms reprojection error px, K, distortion)."""
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size_wh, None, None)
    return float(rms), K, dist.ravel()
