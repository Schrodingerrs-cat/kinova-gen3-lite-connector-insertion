"""Workspace scan: fuse depth views taken from predefined poses into one dense cloud (base frame).

Each view is cleaned (cloud.py), then aligned to the views already merged with trimmed ICP, so
small camera-pose errors (hand-eye, kinematics) do not smear the fused cloud. The size of each
correction is a direct measure of how consistent the calibration is.
LIMIT: on a flat table ICP constrains only height, roll and pitch. In-plane shift and yaw are NOT
observable from table points and are not corrected (a second ICP stage on off-table points was tried
and diverged on real frames: partial object views do not overlap well enough).
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

from .cloud import clean_cloud, cloud_from_depth, voxel_downsample


def view_cloud(depth, K, T_BC, cp):
    """One full-frame depth image -> cleaned cloud in the base frame."""
    h, w = depth.shape
    pts, roi = cloud_from_depth(depth.astype(float), K, T_BC, (0, 0, w, h), margin_px=0, edge_jump_m=cp["edge_jump_m"])
    return clean_cloud(pts, roi, cp)[0]


def _kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    R = Vt.T @ np.diag([1, 1, np.linalg.det(Vt.T @ U.T)]) @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cd - R @ cs
    return T


def icp(src, dst_tree, dst, iters=25, trim=0.7, max_dist=0.02):
    """Trimmed point-to-point ICP of src onto dst. Returns (T 4x4, rms of the kept pairs after alignment)."""
    T, p = np.eye(4), src.copy()
    for _ in range(iters):
        d, i = dst_tree.query(p, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            break
        keep = np.flatnonzero(ok)[np.argsort(d[ok])[: int(trim * ok.sum())]]
        dT = _kabsch(p[keep], dst[i[keep]])
        p = p @ dT[:3, :3].T + dT[:3, 3]
        T = dT @ T
    d, _ = dst_tree.query(p, distance_upper_bound=max_dist)
    d = d[np.isfinite(d)]
    return T, float(np.sqrt(np.mean(d ** 2))) if len(d) else float("nan")


def fuse_clouds(clouds, voxel_m=0.001, icp_voxel_m=0.003):
    """clouds: list of Nx3 base-frame clouds (first = reference). Returns (fused Nx3, per-view info list)."""
    merged = clouds[0]
    info = [dict(view=0, shift_mm=0.0, rot_deg=0.0, rms_before_mm=None, rms_after_mm=None)]
    for k, c in enumerate(clouds[1:], 1):
        dst, _ = voxel_downsample(merged, np.zeros(len(merged), bool), icp_voxel_m)
        src, _ = voxel_downsample(c, np.zeros(len(c), bool), icp_voxel_m)
        tree = cKDTree(dst)
        d0, _ = tree.query(src, distance_upper_bound=0.02)
        T, rms = icp(src, tree, dst)
        c = c @ T[:3, :3].T + T[:3, 3]
        ang = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))
        info.append(dict(view=k, shift_mm=float(np.linalg.norm(T[:3, 3]) * 1e3), rot_deg=float(ang),
                         rms_before_mm=float(np.sqrt(np.mean(d0[np.isfinite(d0)] ** 2)) * 1e3) if np.isfinite(d0).any() else None,
                         rms_after_mm=rms * 1e3, T=T.tolist()))
        merged = np.vstack([merged, c])
    return voxel_downsample(merged, np.zeros(len(merged), bool), voxel_m)[0], info


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
