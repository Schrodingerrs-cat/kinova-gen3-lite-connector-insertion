#!/usr/bin/env python3
"""Calibration scan over the EMPTY table: how consistently do the views see the same flat plane?

python3 calib_scan.py              move the arm through calibration.yaml `calib_scan.views`, save every view, analyse
python3 calib_scan.py --analyze DIR   analyse saved views again (no robot), e.g. a scan folder Media/Grasp/<run>/scan/<ts>

A perfect hand-eye + intrinsics gives every view the SAME plane in the base frame: tilt 0 and the same height. The spread is the calibration error.
Needs the arm driver + MoveIt + camera running (Code/Grasp/run_ros.sh). Start from the home pose with the table EMPTY inside `scene.region`.
"""
import argparse
import json
import pathlib
import sys
import time

import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib_lib as cl  # noqa: E402

cfg = yaml.safe_load(open(HERE / "calibration.yaml"))
TZ, REGION = cfg["scene"]["table_z_m"], cfg["scene"]["region"]


def analyze(out):
    out = pathlib.Path(out)
    ref = ((REGION[0] + REGION[1]) / 2, (REGION[2] + REGION[3]) / 2)
    rows = []
    for f in sorted(out.glob("view_*.npz")):
        v = np.load(f)
        P = cl.backproject(v["depth"], v["K"], v["T_BC"])
        P = P[(P[:, 0] > REGION[0]) & (P[:, 0] < REGION[1]) & (P[:, 1] > REGION[2]) & (P[:, 1] < REGION[3]) & (np.abs(P[:, 2] - TZ) < 0.06)]
        k = int(f.stem[5:])
        if len(P) < 2000:
            rows.append(dict(view=k, points=len(P), tilt_deg=None, height_mm=None))
            continue
        n, d, inl = cl.fit_plane(P)
        tilt, h = cl.plane_tilt_height(n, d, ref)
        rows.append(dict(view=k, points=len(P), inliers=inl, tilt_deg=tilt, height_mm=h * 1e3))
    ok = [r for r in rows if r["tilt_deg"] is not None]
    if not ok:
        sys.exit("no view has enough table points inside scene.region")
    tilt, h = np.array([r["tilt_deg"] for r in ok]), np.array([r["height_mm"] for r in ok])
    summary = dict(views_used=len(ok), views_skipped=len(rows) - len(ok), tilt_deg_mean=float(tilt.mean()), tilt_deg_max=float(tilt.max()),
                   height_mm_mean=float(h.mean()), height_mm_std=float(h.std()), height_mm_range=float(np.ptp(h)), reference_xy_m=ref)
    (out / "calib_scan.json").write_text(json.dumps(dict(summary=summary, views=rows), indent=1))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for name, y, unit, fn in (("Table plane tilt seen by each view", tilt, "deg", "tilt_per_view.png"), ("Table height at the region centre seen by each view", h, "mm", "height_per_view.png")):
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar([str(r["view"]) for r in ok], y, color="tab:blue")
        ax.axhline(y.mean(), color="k", lw=1, label="mean %.2f %s" % (y.mean(), unit))
        ax.set_xlabel("view"); ax.set_ylabel(unit); ax.set_title(name + " (0 / equal = perfect calibration)"); ax.legend()
        fig.tight_layout(); fig.savefig(out / fn, dpi=100); plt.close(fig)
    print(json.dumps(summary, indent=1))
    return summary


def scan():
    import rclpy
    from arm import Arm
    from robot_ros import Robot
    rclpy.init()
    robot = Robot("calib_scan", background=True)
    g, c = cfg["gripper"], cfg["camera"]
    arm = Arm(robot, cfg["motion"], cl.to_T(g["tcp_in_tool_xyz_m"], g["tcp_in_tool_quat_xyzw"]), TZ - 0.005, g["close_cmd"])
    if not arm.ready():
        sys.exit("no joint states / controller / IK: driver and move_group up? same ros_env.sh?")
    X = cl.to_T(c["hand_eye_xyz_m"], c["hand_eye_quat_xyzw"])
    sc = dict(cfg["scene"], views=cfg["calib_scan"]["views"])
    out = HERE / "results" / time.strftime("scan_%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    T0, q0 = arm.tool_T(), arm.q_deg()
    print("start pose kept as the view orientation; %d views planned, results in %s" % (len(sc["views"]), out), flush=True)
    last, done = q0, 0
    for k, T in enumerate(cl.view_poses(sc, T0, X)):
        q = arm.ik_near(T, [q0, last], cfg["motion"]["max_joint_dev_deg"])
        if q is None:
            print("view %d: no safe IK, skipped" % k, flush=True)
            continue
        last = q
        arm.go_joints(q)
        time.sleep(0.7)
        frames = [robot.grab() for _ in range(cfg["calib_scan"]["frames_per_view"])]
        depth = np.nan_to_num(np.nanmedian(np.where(np.stack([f[1] for f in frames]) > 0, np.stack([f[1] for f in frames]), np.nan), 0)).astype(np.float32)
        T_BT = arm.tool_T()
        np.savez_compressed(out / ("view_%02d.npz" % k), depth=depth, color=frames[-1][0], K=frames[-1][2], T_BT=T_BT, T_BC=T_BT @ X)
        done += 1
        print("view %d captured (%d)" % (k, done), flush=True)
    arm.go_joints(q0)
    analyze(out)
    import os
    os._exit(0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", metavar="DIR")
    a = ap.parse_args()
    analyze(a.analyze) if a.analyze else scan()
