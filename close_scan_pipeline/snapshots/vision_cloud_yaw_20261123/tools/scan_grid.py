#!/usr/bin/env python3
"""Grid of camera views around the plug: left-right (y) and front-back (x) in small steps, same height and orientation. Saved like method 2's 3 views so every offline tool works on it.

  python3 ~/kinova/work/tools/scan_grid.py                       plan only: checks every leg (limits, joint steps, planning scene), moves nothing
  python3 ~/kinova/work/tools/scan_grid.py --execute --name X    REAL MOTION: home -> each grid view (stop 1 s, capture) -> home. No grasp, the gripper does not move.
  options: --nx 3 --ny 3 --dx 0.03 --dy 0.05   views along x (front-back) and y (left-right) with these spacings in metres, centred on method 2's middle view

Output: grasp_runs/run_<time>_grid_<name>/method2_perception/scan/view_00.npz ... (depth median of 5 frames, colour, K, T_BT, T_BC), grid.json, then the stage 1 check on all views (not logged).
Needs the stack up (run_stack_method2.sh), arm at project home (within 2 deg), gripper open, e-stop in reach, ros_env.sh sourced.
Legs are straight Cartesian lines at constant orientation, 2 mm samples, the same screens as method 2 (joint step <= 5 deg, 6 deg for joint 6), plus joint limits minus margin,
tool height >= 0.20 m, and the MoveIt planning scene (CAD fixture + table) checked along each leg.
"""
import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation, Slerp

HOME_DIR = pathlib.Path.home()
G = HOME_DIR / "kinova/ins/repo/Code/Grasp"
M2 = HOME_DIR / "kinova/ins/method2_src/Code/Grasp"
sys.path.insert(0, str(M2 / "ros2_iface"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from robot_ros import BASE, TOOL, Robot, matrix  # noqa: E402  (method 2's Robot: the line planner the 3-view runner uses, plus the planning-scene path check)
from example_interfaces.msg import Bool  # noqa: E402

HOME_Q = (np.array([3.0266, 356.8712, 93.4037, 271.4634, 272.3364, 277.3673]) + 180) % 360 - 180
MIN_TOOL_Z = 0.20
STEP_LIMITS = np.array([5, 5, 5, 5, 5, 6.0])


def samples(T0, T1, step=0.002, rot_step=1.0):
    d = np.linalg.norm(T1[:3, 3] - T0[:3, 3])
    ang = np.degrees(Rotation.from_matrix(T0[:3, :3].T @ T1[:3, :3]).magnitude())
    n = max(1, int(np.ceil(d / step)), int(np.ceil(ang / rot_step)))
    sl = Slerp([0, 1], Rotation.from_matrix([T0[:3, :3], T1[:3, :3]]))
    out = []
    for f in np.linspace(0, 1, n + 1)[1:]:
        T = np.eye(4)
        T[:3, :3] = sl(f).as_matrix()
        T[:3, 3] = T0[:3, 3] + f * (T1[:3, 3] - T0[:3, 3])
        out.append(T)
    return out


def fresh_frame(robot, X):
    frames, stamps = [], []
    for _ in range(5):
        color, depth, K = robot.grab()
        msg, col = robot._img["depth"], robot._img["color"]
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        cns = col.header.stamp.sec * 10**9 + col.header.stamp.nanosec
        if stamps and ns <= stamps[-1]:
            raise RuntimeError("repeated depth frame during capture")
        if not 0 <= (robot.get_clock().now().nanoseconds - ns) / 1e9 < 1 or abs(cns - ns) > 100000000:
            raise RuntimeError("stale or misaligned frame timestamps")
        frames.append(depth)
        stamps.append(ns)
    when = rclpy.time.Time.from_msg(msg.header.stamp)
    if not robot.spin_until(lambda: robot.tfbuf.can_transform(BASE, TOOL, when), 2):
        raise RuntimeError("no tool TF at the depth timestamp")
    tr = robot.tfbuf.lookup_transform(BASE, TOOL, when).transform
    B = matrix([tr.translation.x, tr.translation.y, tr.translation.z], [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
    stack = np.stack(frames)
    depth = np.ma.median(np.ma.masked_where((stack <= 0) | ~np.isfinite(stack), stack), axis=0).filled(0).astype("float32")
    return dict(depth=depth, color=color, K=K, T_BT=B, T_BC=B @ X, depth_stamp_ns=np.array(stamps, dtype="int64"), color_stamp_ns=np.array(cns, dtype="int64"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nx", type=int, default=3)
    ap.add_argument("--ny", type=int, default=3)
    ap.add_argument("--dx", type=float, default=0.03)
    ap.add_argument("--dy", type=float, default=0.05)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--name", default="test")
    a = ap.parse_args()

    taught = json.loads((M2 / "config/nearby_camera_poses_sideways.json").read_text())
    X = np.array(taught["T_tool_camera_at_teaching"])
    centre = np.array(taught["poses"][1]["T_base_tool"])
    xs = (np.arange(a.nx) - (a.nx - 1) / 2) * a.dx
    ys = (np.arange(a.ny) - (a.ny - 1) / 2) * a.dy
    grid = []
    for i, ox in enumerate(xs):
        for oy in (ys if i % 2 == 0 else ys[::-1]):
            T = centre.copy()
            T[:3, 3] += [ox, oy, 0.0]
            grid.append((ox, oy, T))
    motion = yaml.safe_load(open(G / "config/grasp.yaml"))["motion"]
    lim = np.array(motion["joint_limits_deg"], float) - motion["joint_margin_deg"]

    rclpy.init()
    robot = Robot("scan_grid", background=True)
    fault = {"on": False}
    robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("on", bool(m.data)), 10)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    robot.spin_until(lambda: False, 1.0)
    q_now = np.degrees(robot.q)
    away = float(np.max(np.abs(q_now - HOME_Q)))
    if away > 2:
        sys.exit("arm is %.1f deg from project home: go home first" % away)
    if robot.grip_pos_msr is None or robot.grip_pos_msr > 0.05:
        sys.exit("gripper is not fully open")
    if fault["on"]:
        sys.exit("arm is in fault")

    start = robot.tool_matrix()
    targets = [g[2] for g in grid] + [start]
    plans, seed, leg_start = [], robot.q.copy(), start.copy()
    print("%d views: x offsets %s mm, y offsets %s mm around tool (%.4f, %.4f, %.4f)" % (len(grid), np.round(xs * 1e3, 0), np.round(ys * 1e3, 0), *centre[:3, 3]))
    for i, T in enumerate(targets):
        label = "view %d" % i if i < len(grid) else "return home"
        if T[2, 3] < MIN_TOOL_Z:
            sys.exit("%s: tool height %.3f below %.2f m" % (label, T[2, 3], MIN_TOOL_Z))
        path = robot.line_path(T, seed, max_step_m=0.002, min_fraction=1 - 1e-6, waypoints=samples(leg_start, T))
        if path is None or len(path) < 2:
            sys.exit("%s: no complete Cartesian path, nothing moved" % label)
        Q = np.vstack([seed, np.array(path)])
        peaks = np.degrees(np.abs(np.diff(Q, axis=0))).max(axis=0)
        if not np.isfinite(Q).all() or np.any(peaks > STEP_LIMITS):
            sys.exit("%s: joint step screen rejected %s, nothing moved" % (label, np.round(peaks, 1)))
        qd = (np.degrees(Q) + 180) % 360 - 180
        if np.any(np.abs(qd) > lim):
            sys.exit("%s: a waypoint is outside the joint limits minus margin, nothing moved" % label)
        for k in range(0, len(Q) - 25, 25):
            hit = robot.path_collision(np.degrees(Q[k]), np.degrees(Q[k + 25]))
            if hit:
                sys.exit("%s: %s, nothing moved" % (label, hit))
        hit = robot.path_collision(np.degrees(Q[-26 if len(Q) > 26 else 0]), np.degrees(Q[-1]))
        if hit:
            sys.exit("%s: %s, nothing moved" % (label, hit))
        dt = np.maximum(0.25, 2 * np.max(np.abs(np.diff(Q, axis=0)), axis=1) / np.deg2rad(6))
        times = np.cumsum(dt)
        plans.append(dict(target=T, path=path, times=times.tolist(), seed=seed.copy()))
        print("%-11s tool xyz %s, max joint step %.1f deg, %.1f s" % (label, np.round(T[:3, 3], 4), peaks.max(), times[-1]), flush=True)
        seed, leg_start = np.array(path[-1]), T
    total = sum(p["times"][-1] for p in plans) + 4 * len(grid)
    print("PLAN OK: %d legs, about %.0f s with stops and captures." % (len(plans), total), flush=True)
    if not a.execute:
        print("Nothing moved. Add --execute --name NAME to run it.", flush=True)
        os._exit(0)

    run = HOME_DIR / "kinova/work/grasp_runs" / ("run_%s_grid_%s" % (time.strftime("%Y%m%d_%H%M%S"), a.name))
    scan = run / "method2_perception/scan"
    scan.mkdir(parents=True)
    (run / "plots").mkdir()
    meta = dict(nx=a.nx, ny=a.ny, dx=a.dx, dy=a.dy, offsets_m=[[g[0], g[1]] for g in grid], joints_deg=[], started=time.strftime("%Y-%m-%dT%H:%M:%S"))
    robot.grab()
    print("REAL MOTION to %d views and back home. Gripper unchanged." % len(grid), flush=True)
    for i, plan in enumerate(plans):
        if np.max(np.abs(robot.q - plan["seed"])) > np.deg2rad(1):
            sys.exit("start state differs before leg %d, stopping" % i)
        print("leg %d/%d, %.1f s" % (i + 1, len(plans), plan["times"][-1]), flush=True)
        if fault["on"]:
            sys.exit("arm fault, stopping")
        robot.send_joint_path(plan["path"], plan["times"], velocities=np.zeros_like(plan["path"]).tolist(), timeout=plan["times"][-1] + 30)
        m = robot.tool_matrix()
        if np.linalg.norm(m[:3, 3] - plan["target"][:3, 3]) > 0.003 or Rotation.from_matrix(plan["target"][:3, :3].T @ m[:3, :3]).magnitude() > np.deg2rad(1):
            sys.exit("leg %d endpoint mismatch, stopping" % i)
        if i < len(grid):
            robot.spin_until(lambda: False, 1.0)
            before = robot.q.copy()
            frame = fresh_frame(robot, X)
            if np.max(np.abs(robot.q - before)) > np.deg2rad(0.2):
                sys.exit("arm moved during capture, stopping")
            np.savez_compressed(scan / ("view_%02d.npz" % i), **frame)
            meta["joints_deg"].append(np.degrees(robot.q).round(3).tolist())
            print("view %d/%d saved" % (i + 1, len(grid)), flush=True)
    meta["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    meta["home_error_deg"] = float(np.max(np.abs(np.degrees(robot.q) - HOME_Q)))
    (run / "grid.json").write_text(json.dumps(meta, indent=1))
    print("back home (max joint error %.2f deg), fault=%s. Saved to %s" % (meta["home_error_deg"], fault["on"], run), flush=True)
    robot.destroy_node()
    rclpy.shutdown()
    import stage1_find
    res, P, B = stage1_find.evaluate(run)
    print("\nStage 1 check on all %d views (not logged):" % len(grid))
    stage1_find.report(res)
    stage1_find.summary_image(run, P, B)
    (run / "stage1.json").write_text(json.dumps(res, indent=1))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
