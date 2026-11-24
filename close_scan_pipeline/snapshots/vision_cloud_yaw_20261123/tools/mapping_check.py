#!/usr/bin/env python3
"""Checks how well the camera cloud maps into the base frame. Moves nothing.

  python3 ~/kinova/work/tools/mapping_check.py record NAME    save depth + arm pose at the current arm pose (put the arm there by hand first)
  python3 ~/kinova/work/tools/mapping_check.py compare        register every recorded view onto the first one in the base frame and print the offsets

A static scene seen from different arm poses must give the same cloud in the base frame. The offsets printed by compare are the real size of the
camera mapping error (arm pose x hand-eye x depth) for this scene. Needs the stack up (run_stack_method2.sh). Keep the scene still between views
(no hands, nothing moved). Take 4 or more views with clearly different camera positions and tilts, all seeing the cradle.
Recordings go to ~/kinova/work/calibration/mapping_check/NAME.npz, the hand-eye is applied at compare time so it can be changed without recording again.
"""
import os
import pathlib
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

HOME = pathlib.Path.home()
OUT = HOME / "kinova/work/calibration/mapping_check"
TABLE = -0.041


def hand_eye():
    v = yaml.safe_load(open(HOME / "kinova/work/calibration/camera_calibration_values.yaml"))["hand_eye_in_use"]
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = Rot.from_quat(v["quat_xyzw"]).as_matrix(), v["xyz_m"]
    return X


def record(name):
    import rclpy
    sys.path.insert(0, str(HOME / "kinova/ins/repo/Code/Grasp/ros2_iface"))
    from robot_ros import Robot
    rclpy.init()
    robot = Robot("mapping_check", background=True)
    robot.wait_ready(10)
    depth = np.median(np.stack([robot.grab()[1] for _ in range(5)]), axis=0)
    color, _, K = robot.grab()
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez(OUT / (name + ".npz"), depth=depth, color=color, K=K, T_base_tool=robot.tool_matrix(), joints_deg=robot.joints_deg())
    print("saved %s, tool xyz %s, valid depth %.0f %%" % (name, np.round(robot.tool_matrix()[:3, 3], 4), 100 * np.mean(depth > 0)))
    os._exit(0)


def base_cloud(d, X):
    depth, K = d["depth"], d["K"]
    T = d["T_base_tool"] @ X
    v, u = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
    z = depth[::2, ::2]
    ok = (z > 0.1) & (z < 0.7)
    P = np.c_[(u[ok] - K[0, 2]) * z[ok] / K[0, 0], (v[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]] @ T[:3, :3].T + T[:3, 3]
    box = (P[:, 0] > 0.15) & (P[:, 0] < 0.45) & (P[:, 1] > -0.22) & (P[:, 1] < 0.28) & (P[:, 2] > TABLE + 0.006) & (P[:, 2] < 0.08)
    return P[box]


def register(src, ref):
    """Trimmed point-to-point ICP, src onto ref, rigid 6 DOF. Returns (R, t, rms of the best 70 %, kept fraction within 3 mm)."""
    tree = cKDTree(ref)
    R, t = np.eye(3), np.zeros(3)
    for it in range(40):
        P = src @ R.T + t
        d, idx = tree.query(P, distance_upper_bound=0.03)
        ok = np.isfinite(d)
        keep = np.flatnonzero(ok)[np.argsort(d[ok])[:int(0.7 * ok.sum())]]
        a, b = P[keep], ref[idx[keep]]
        ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        dR = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        dt = cb - dR @ ca
        R, t = dR @ R, dR @ t + dt
    P = src @ R.T + t
    d, _ = tree.query(P)
    best = np.sort(d)[:int(0.7 * len(d))]
    return R, t, float(np.sqrt(np.mean(best ** 2))), float(np.mean(d < 0.003))


def compare():
    files = sorted(OUT.glob("*.npz"))
    if len(files) < 2:
        sys.exit("need at least 2 recordings in %s" % OUT)
    X = hand_eye()
    views = [(f.stem, np.load(f)) for f in files]
    clouds = [base_cloud(d, X) for _, d in views]
    print("hand-eye in use applied; reference view: %s (%d points)\n" % (views[0][0], len(clouds[0])))
    print("%-14s %-22s %9s %9s  %s" % ("view", "camera xyz (m)", "shift mm", "turn deg", "(shift x, y, z mm)  rms 70%  within 3 mm"))
    for (name, d), cl in zip(views, clouds):
        cam = (d["T_base_tool"] @ X)[:3, 3]
        if cl is clouds[0]:
            print("%-14s %-22s %9s %9s" % (name, np.round(cam, 3), "-", "-"))
            continue
        R, t, rms, frac = register(cl, clouds[0])
        print("%-14s %-22s %9.1f %9.2f  (%+.1f, %+.1f, %+.1f)  %.1f mm  %.0f %%" % (name, np.round(cam, 3), np.linalg.norm(t) * 1e3, np.degrees(np.linalg.norm(Rot.from_matrix(R).as_rotvec())), *(t * 1e3), rms * 1e3, 100 * frac))
    print("\nA perfect mapping gives shifts near 0 (1-2 mm). Shifts of several mm that grow with camera distance or tilt point at the hand-eye (rotation part) or the arm model.")


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "record":
        record(sys.argv[2])
    elif len(sys.argv) == 2 and sys.argv[1] == "compare":
        compare()
    else:
        print(__doc__)
