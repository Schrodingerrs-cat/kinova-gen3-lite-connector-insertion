#!/usr/bin/env python3
"""Aligns the CAD fixture (cradle parts) to the LIVE camera depth cloud and prints the fixture_pose.yaml values that make them coincide. Moves nothing.

   python3 ~/kinova/work/tools/align_fixture_to_cloud.py            measure and print (also tests a left-right mirrored CAD)
   python3 ~/kinova/work/tools/align_fixture_to_cloud.py --apply    also write the best offset/yaw into setup_cad/fixture_pose.yaml (a backup .bak is kept)

Needs the stack up (run_stack_method2.sh) with the cradle in the camera view from the current pose (the arm at home is fine).
Method: 5 depth frames median -> cloud in the base frame with the hand-eye in use; keep points above the table in the cradle region; trimmed ICP (cloud -> CAD surface samples, 4 degrees of freedom:
x, y, z, yaw) starting from the current fixture_pose.yaml. Accuracy is limited by the calibration floor (a few mm), the cable and plug points are ignored by the trimming.
"""
import argparse
import pathlib
import shutil
import sys

import numpy as np
import rclpy
import trimesh
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

HOME = pathlib.Path.home()
sys.path.insert(0, str(HOME / "kinova/ins/repo/Code/Grasp/ros2_iface"))
from robot_ros import Robot  # noqa: E402

D = HOME / "kinova/work/setup_cad"
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--apply", action="store_true")
ap.add_argument("--dump", help="save the cloud used (npz) for offline checks")
args = ap.parse_args()

vals = yaml.safe_load(open(HOME / "kinova/work/calibration/camera_calibration_values.yaml"))["hand_eye_in_use"]
X = np.eye(4)
X[:3, :3], X[:3, 3] = Rot.from_quat(vals["quat_xyzw"]).as_matrix(), vals["xyz_m"]
pose = yaml.safe_load(open(D / "fixture_pose.yaml"))
anchor, offset, yaw0 = np.array(pose["anchor_m"], float), np.array(pose["offset_m"], float), np.radians(float(pose["yaw_deg"]))

rclpy.init()
robot = Robot("align_fixture", background=True)
robot.wait_ready(10)
frames = []
for _ in range(5):
    color, depth, K = robot.grab()
    frames.append(depth)
depth = np.median(np.stack(frames), axis=0)
T_bc = robot.tool_matrix() @ X
v, u = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
z = depth[::2, ::2]
ok = (z > 0.1) & (z < 0.7)
P = np.c_[(u[ok] - K[0, 2]) * z[ok] / K[0, 0], (v[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]] @ T_bc[:3, :3].T + T_bc[:3, 3]
TABLE = -0.041
box = (P[:, 0] > 0.15) & (P[:, 0] < 0.45) & (P[:, 1] > -0.22) & (P[:, 1] < 0.28) & (P[:, 2] > TABLE + 0.004) & (P[:, 2] < 0.08)
cloud = P[box]
print("cloud: %d points in the cradle region above the table" % len(cloud), flush=True)
if args.dump:
    np.savez(args.dump, cloud=cloud)

parts = yaml.safe_load(open(D / "parts_index.yaml"))
rng = np.random.default_rng(0)
cad = []
for p in parts:
    if p["role"] == "acrylic" or not (0.10 < p["centre_m"][0] < 0.45):
        continue
    m = trimesh.load(D / "parts" / p["file"])
    cad.append(m.sample(max(200, int(m.area * 60000)), return_index=False) if hasattr(m, "sample") else trimesh.sample.sample_surface(m, 2000)[0])
cad = np.vstack(cad)
print("CAD cradle samples: %d points" % len(cad), flush=True)


def place(Pn, tx, ty, tz, yaw, mirror):
    Q = Pn.copy()
    if mirror:
        Q[:, 1] = 2 * anchor[1] - Q[:, 1]
    R = Rot.from_euler("z", yaw).as_matrix()
    return (Q - anchor) @ R.T + anchor + np.array([tx, ty, tz])


def icp(mirror, start):
    tx, ty, tz, yaw = start
    tree = None
    for it in range(60):
        Q = place(cad, tx, ty, tz, yaw, mirror)
        tree = cKDTree(Q)
        d, idx = tree.query(cloud, distance_upper_bound=0.03 if it < 30 else 0.012)
        good = np.isfinite(d)
        if good.sum() < 200:
            break
        keep = np.flatnonzero(good)[np.argsort(d[good])[:int(0.6 * good.sum())]]
        a, b = Q[idx[keep]], cloud[keep]
        ca, cb = a.mean(0), b.mean(0)
        H = (a[:, :2] - ca[:2]).T @ (b[:, :2] - cb[:2])
        dyaw = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R2 = Rot.from_euler("z", dyaw).as_matrix()[:2, :2]
        sh_xy = cb[:2] - R2 @ ca[:2]
        yaw += dyaw
        c0 = anchor[:2]
        # compose: new = R2 (old) + sh_xy ; keep the anchor-based parametrisation
        shift_old = np.array([tx, ty])
        new_shift = R2 @ (c0 + shift_old) + sh_xy - c0
        tx, ty = new_shift
        tz += float(cb[2] - ca[2])
    Q = place(cad, tx, ty, tz, yaw, mirror)
    d, _ = cKDTree(Q).query(cloud)
    keep = np.sort(d)[:int(0.5 * len(d))]
    return (tx, ty, tz, yaw), float(np.sqrt(np.mean(keep ** 2))), float(np.mean(d < 0.003))


start = (offset[0], offset[1], offset[2], yaw0)
d0, _ = cKDTree(place(cad, *start, False)).query(cloud)
print("current fixture_pose: median cloud-to-CAD distance %.1f mm, %.0f %% of cloud points within 3 mm" % (np.median(d0) * 1e3, 100 * np.mean(d0 < 0.003)))
best = None
for mirror in (False, True):
    sol, rms, frac = icp(mirror, start)
    print("%-10s -> offset (%.4f, %.4f, %.4f) m, yaw %.2f deg | trimmed rms %.1f mm, %.0f %% of cloud points within 3 mm" % ("mirrored" if mirror else "as mapped", *sol[:3], np.degrees(sol[3]), rms * 1e3, 100 * frac), flush=True)
    if best is None or rms < best[1]:
        best = (sol, rms, frac, mirror)
sol, rms, frac, mirror = best
print("\nbest: %s. Change from the current pose: dx %+.1f mm, dy %+.1f mm, dz %+.1f mm, yaw %+.1f deg" % ("MIRRORED left-right" if mirror else "as mapped", (sol[0] - offset[0]) * 1e3, (sol[1] - offset[1]) * 1e3, (sol[2] - offset[2]) * 1e3, np.degrees(sol[3] - yaw0)))
print("fixture_pose.yaml values: offset_m: [%.4f, %.4f, %.4f]  yaw_deg: %.2f" % (*sol[:3], np.degrees(sol[3])))
if mirror:
    print("NOTE: the mirrored fit is better, so the CAD-to-base mapping must be mirrored in y (not applied automatically).")
elif args.apply:
    shutil.copy(D / "fixture_pose.yaml", D / "fixture_pose.yaml.bak")
    pose["offset_m"], pose["yaw_deg"] = [round(float(v), 4) for v in sol[:3]], round(float(np.degrees(sol[3])), 2)
    text = (D / "fixture_pose.yaml").read_text().split("\nanchor_m")[0]
    (D / "fixture_pose.yaml").write_text(text + "\nanchor_m: %s\noffset_m: %s\nyaw_deg: %s\n" % (pose["anchor_m"], pose["offset_m"], pose["yaw_deg"]))
    print("written to fixture_pose.yaml (backup .bak)")
robot.destroy_node()
rclpy.shutdown()
