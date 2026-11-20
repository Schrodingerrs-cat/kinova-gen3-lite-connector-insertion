#!/usr/bin/env python3
"""Close scan around the marker: detect it, centre the camera above it, then sweep from its left to its right, tilted slightly toward it so the sides and depth show.

  python3 ~/kinova/work/tools/scan_orbit.py                              plan only: finds the marker from the current (home) view, designs and checks every pose and leg, moves nothing
  python3 ~/kinova/work/tools/scan_orbit.py --execute --name X           REAL MOTION: home -> close pose over the marker (capture, re-detect) -> left end -> ... -> right end (capture at each) -> home
  options: --height 0.20  camera height above the board top (m)      --lateral 0.06  sweep half width (m)      --n 5  poses incl. the centre one (3, 5 or 7)
           --min-tcp-z 0.10  lowest allowed fingertip height (m)     --min-tcp-dist 0.09  closest allowed fingertip to the aim point (m)

Sequence: (1) marker blob from the home view, no motion (tools/find_plug_m3.py seed). (2) Close pose: camera straight above the marker at --height, marker in the image centre, capture, re-detect (a sharper
seed); if the refined marker is more than 8 mm from the planned one the sweep is re-planned from there. (3) Sweep: camera positions (marker x, marker y + s, board top + height) for s from +lateral (left) to
-lateral (right), each aimed at the plug middle, so the tilt grows toward the ends. Camera roll about its axis is free and chosen for the smallest joint motion.
Checks before any motion: IK for every pose, joint limits minus margin, joint step <= 5 deg per 2 mm sample (6 for joint 6), fingertip (tcp) height and distance floors, planning scene (CAD fixture + table, placed
from the measured board) along every leg. The fingertips sit about 12.5 cm ahead of the camera along its axis, so a low camera puts them near the board: the plan prints the fingertip height above the board
and relative to the plug top for each pose. Output like scan_grid.py: grasp_runs/run_<time>_orbit_<name>/method2_perception/scan/view_NN.npz + orbit.json.
Needs the stack up (run_stack_method2.sh), arm at project home (within 2 deg), gripper open, e-stop in reach, ros_env.sh sourced.
"""
import argparse
import json
import os
import pathlib
import sys
import tempfile
import time

import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import connector  # noqa: E402
import plug_points as pp  # noqa: E402
import scan_grid  # noqa: E402  (puts method 2's robot_ros on the path)
from example_interfaces.msg import Bool  # noqa: E402
from robot_ros import Robot  # noqa: E402

HOME_DIR = pathlib.Path.home()
HOME_Q, STEP_LIMITS, M2, G = scan_grid.HOME_Q, scan_grid.STEP_LIMITS, scan_grid.M2, scan_grid.G
BOARD_TOP, PLUG_TOP = -0.018, 0.025   # measured: board strip top face and plug top


class PlanError(Exception):
    pass


def tcp_matrix(cfg):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(cfg["gripper"]["tcp_in_tool_quat_xyzw"]).as_matrix()
    T[:3, 3] = cfg["gripper"]["tcp_in_tool_xyz_m"]
    return T


def aimed_camera(c, target):
    """Camera pose at position c with the optical axis on target, image up toward base +x (roll is then searched)."""
    z = (target - c) / np.linalg.norm(target - c)
    ref = np.array([-1.0, 0, 0])
    y = ref - np.dot(ref, z) * z
    y /= np.linalg.norm(y)
    T = np.eye(4)
    T[:3, :3] = np.c_[np.cross(y, z), y, z]
    T[:3, 3] = c
    return T


class Planner:
    def __init__(self, robot, X, T_tcp, lim, a):
        self.r, self.X, self.T_tcp, self.lim, self.a = robot, X, T_tcp, lim, a

    def pose(self, T_cam0, target, seed_rad):
        """Best roll: feasible IK with the smallest joint change from the seed. Returns (T_tool, T_cam, q_deg, roll) or raises PlanError with the reason."""
        best, reasons = None, []
        for roll in range(-180, 180, 30):
            Rz = np.eye(4)
            Rz[:3, :3] = Rotation.from_euler("z", roll, degrees=True).as_matrix()
            T_cam = T_cam0 @ Rz
            T_tool = T_cam @ np.linalg.inv(self.X)
            q = self.r.ik(T_tool, np.degrees(seed_rad))
            if q is None:
                reasons.append("no IK")
                continue
            qr = (np.asarray(q, float) + 180) % 360 - 180
            tcp = (T_tool @ self.T_tcp)[:3, 3]
            dev = np.abs((qr - np.degrees(seed_rad) + 180) % 360 - 180).max()
            if np.any(np.abs(qr) > self.lim):
                reasons.append("joint limit (%s)" % np.round(qr, 0))
            elif dev > 100:
                reasons.append("joint jump %.0f deg" % dev)
            elif tcp[2] < self.a.min_tcp_z:
                reasons.append("fingertip height %.3f m < %.2f" % (tcp[2], self.a.min_tcp_z))
            elif np.linalg.norm(tcp - target) < self.a.min_tcp_dist:
                reasons.append("fingertip only %.3f m from the plug" % np.linalg.norm(tcp - target))
            elif self.r.path_collision(q, q):
                reasons.append("collides in the planning scene: " + self.r.path_collision(q, q))
            elif best is None or dev < best[0]:
                best = (dev, T_tool, T_cam, q, roll)
        if best is None:
            kinds = {}
            for r in reasons:
                kinds[r.split(" (")[0].split(":")[0]] = kinds.get(r.split(" (")[0].split(":")[0], 0) + 1
            raise PlanError("no usable camera roll out of %d tried: %s; first reason: %s" % (12, ", ".join("%s x%d" % kv for kv in sorted(kinds.items(), key=lambda kv: -kv[1])), reasons[0] if reasons else "none"))
        return best[1:]

    def legs(self, start_T, seed_rad, tool_poses, labels):
        plans, seed, leg_start = [], np.array(seed_rad, float), np.array(start_T)
        for T, label in zip(tool_poses, labels):
            smp = scan_grid.samples(leg_start, T)
            low = min((s @ self.T_tcp)[2, 3] for s in smp)
            if low < self.a.min_tcp_z:
                raise PlanError("%s: fingertip would drop to %.3f m on the way" % (label, low))
            path = self.r.line_path(T, seed, max_step_m=0.002, min_fraction=1 - 1e-6, waypoints=smp)
            if path is None or len(path) < 2:
                raise PlanError("%s: no complete Cartesian path" % label)
            Q = np.vstack([seed, np.array(path)])
            peaks = np.degrees(np.abs(np.diff(Q, axis=0))).max(axis=0)
            if not np.isfinite(Q).all() or np.any(peaks > STEP_LIMITS):
                raise PlanError("%s: joint step screen rejected %s" % (label, np.round(peaks, 1)))
            if np.any(np.abs((np.degrees(Q) + 180) % 360 - 180) > self.lim):
                raise PlanError("%s: a waypoint is outside the joint limits minus margin" % label)
            for k in range(0, len(Q) - 25, 25):
                hit = self.r.path_collision(np.degrees(Q[k]), np.degrees(Q[k + 25]))
                if hit:
                    raise PlanError("%s: %s" % (label, hit))
            dt = np.maximum(0.25, 2 * np.max(np.abs(np.diff(Q, axis=0)), axis=1) / np.deg2rad(6))
            times = np.cumsum(dt)
            plans.append(dict(label=label, target=T, path=path, times=times.tolist(), seed=seed.copy(), peak=float(peaks.max()), low=float(low)))
            seed, leg_start = np.array(path[-1]), T
        return plans


def sweep_poses(planner, target, seed_rad, a, offsets):
    out, seed = [], np.array(seed_rad, float)
    for s in offsets:
        c = np.array([target[0], target[1] + s, BOARD_TOP + a.height])
        T_tool, T_cam, q, roll = planner.pose(aimed_camera(c, target), target, seed)
        tilt = np.degrees(np.arctan2(abs(s), c[2] - target[2]))
        tz = (T_tool @ planner.T_tcp)[2, 3]
        print("  s %+5.0f mm: camera xyz %s, %3.0f mm above the board, tilt %4.1f deg, fingertip %3.0f mm above the board / %3.0f mm %s the plug top, roll %d" % (s * 1e3, np.round(T_cam[:3, 3], 3), (c[2] - BOARD_TOP) * 1e3, tilt, (tz - BOARD_TOP) * 1e3, abs(tz - PLUG_TOP) * 1e3, "above" if tz > PLUG_TOP else "BELOW", roll), flush=True)
        out.append((s, T_tool, T_cam))
        seed = np.radians(q)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connector", default="usb", help="profile in ~/kinova/work/connectors (usb, aux): scan settings and the plug prior")
    ap.add_argument("--n", type=int)
    ap.add_argument("--height", type=float)
    ap.add_argument("--lateral", type=float)
    ap.add_argument("--aim-z", type=float)
    ap.add_argument("--min-tcp-z", type=float)
    ap.add_argument("--min-tcp-dist", type=float)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--name", default="test")
    a = ap.parse_args()
    prof = connector.load(a.connector)
    for k, key in (("n", "n"), ("height", "height_m"), ("lateral", "lateral_m"), ("aim_z", "aim_z_m"), ("min_tcp_z", "min_tcp_z_m")):
        if getattr(a, k) is None:
            setattr(a, k, prof["scan"][key])
    if a.min_tcp_dist is None:
        a.min_tcp_dist = prof["scan"].get("min_tcp_dist_m", 0.09)
    if a.n not in (3, 5, 7):
        sys.exit("--n must be 3, 5 or 7")

    taught = json.loads((M2 / "config/nearby_camera_poses_sideways.json").read_text())
    X = np.array(taught["T_tool_camera_at_teaching"])
    cfg = yaml.safe_load(open(M2 / "config/grasp.yaml"))
    motion = yaml.safe_load(open(G / "config/grasp.yaml"))["motion"]
    lim = np.array(motion["joint_limits_deg"], float) - motion["joint_margin_deg"]

    rclpy.init()
    robot = Robot("scan_orbit", background=True)
    fault = {"on": False}
    robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("on", bool(m.data)), 10)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    robot.spin_until(lambda: False, 1.0)
    away = float(np.max(np.abs(np.degrees(robot.q) - HOME_Q)))
    if away > 2:
        sys.exit("arm is %.1f deg from project home: go home first" % away)
    if robot.grip_pos_msr is None or robot.grip_pos_msr > 0.05:
        sys.exit("gripper is not fully open")
    if fault["on"]:
        sys.exit("arm is in fault")
    planner = Planner(robot, X, tcp_matrix(cfg), lim, a)

    import find_plug_m3
    tmp = pathlib.Path(tempfile.mkdtemp()) / "home_view.npz"
    np.savez_compressed(tmp, **scan_grid.fresh_frame(robot, X))
    pr = pp.prior([tmp], prof["part"])
    blob, px = find_plug_m3.seed(tmp, near=pr["plug_centre"][:2])
    print("0. connector %s: predicted from the board x %.4f y %.4f" % (a.connector, *pr["plug_centre"][:2]), flush=True)
    target = np.array([blob[0], blob[1], a.aim_z])
    print("1. marker found from home: blob of %d px at x %.4f y %.4f (z %.3f); aim point %s" % (px, blob[0], blob[1], blob[2], np.round(target, 4)), flush=True)

    start, home_seed = robot.tool_matrix(), robot.q.copy()
    half = (a.n - 1) // 2
    steps = np.linspace(a.lateral, -a.lateral, a.n)           # left (+y) to right (-y)
    offsets = [0.0] + [s for s in steps if abs(s) > 1e-9]       # close pose first, then the sweep without the centre pose
    try:
        print("2./3. poses (camera straight above the marker first, then left to right):", flush=True)
        poses = sweep_poses(planner, target, home_seed, a, offsets)
        labels = ["close pose over the marker"] + ["sweep s %+.0f mm" % (p[0] * 1e3) for p in poses[1:]] + ["return home"]
        plans = planner.legs(start, home_seed, [p[1] for p in poses] + [start], labels)
        abort = planner.legs(poses[0][1], np.array(plans[0]["path"][-1]), [start], ["abort to home"])
    except PlanError as e:
        print("PLAN FAILED: %s. Nothing moved. Adjust --height / --lateral / --min-tcp-z." % e, flush=True)
        os._exit(1)
    for p in plans:
        print("  leg %-28s max joint step %.1f deg, lowest fingertip %.3f m, %.1f s" % (p["label"], p["peak"], p["low"], p["times"][-1]), flush=True)
    print("PLAN OK: %d views, %d legs, about %.0f s with stops and captures." % (len(poses), len(plans), sum(p["times"][-1] for p in plans) + 4 * len(poses)), flush=True)
    if not a.execute:
        print("Nothing moved. Add --execute --name NAME to run it.", flush=True)
        os._exit(0)

    run = HOME_DIR / "kinova/work/grasp_runs" / ("run_%s_orbit_%s" % (time.strftime("%Y%m%d_%H%M%S"), a.name))
    scan = run / "method2_perception/scan"
    scan.mkdir(parents=True)
    (run / "plots").mkdir()
    meta = dict(connector=a.connector, height_m=a.height, lateral_m=a.lateral, offsets_m=list(offsets), aim_point_home=target.tolist(), joints_deg=[], started=time.strftime("%Y-%m-%dT%H:%M:%S"))
    robot.grab()

    def run_leg(plan, i):
        if np.max(np.abs(robot.q - plan["seed"])) > np.deg2rad(1):
            raise RuntimeError("start state differs before %s" % plan["label"])
        if fault["on"]:
            raise RuntimeError("arm fault")
        print("%s, %.1f s" % (plan["label"], plan["times"][-1]), flush=True)
        robot.send_joint_path(plan["path"], plan["times"], velocities=np.zeros_like(plan["path"]).tolist(), timeout=plan["times"][-1] + 30)
        m = robot.tool_matrix()
        if np.linalg.norm(m[:3, 3] - plan["target"][:3, 3]) > 0.003 or Rotation.from_matrix(plan["target"][:3, :3].T @ m[:3, :3]).magnitude() > np.deg2rad(1):
            raise RuntimeError("endpoint mismatch at %s" % plan["label"])

    def capture(i):
        robot.spin_until(lambda: False, 1.0)
        before = robot.q.copy()
        frame = scan_grid.fresh_frame(robot, X)
        if np.max(np.abs(robot.q - before)) > np.deg2rad(0.2):
            raise RuntimeError("arm moved during capture")
        np.savez_compressed(scan / ("view_%02d.npz" % i), **frame)
        meta["joints_deg"].append(np.degrees(robot.q).round(3).tolist())
        print("view %d saved" % i, flush=True)
        return scan / ("view_%02d.npz" % i)

    print("REAL MOTION. Gripper unchanged.", flush=True)
    run_leg(plans[0], 0)
    first = capture(0)
    try:
        refined, px2 = find_plug_m3.seed(first)
        u, v = np.load(first)["K"][0, 2], np.load(first)["K"][1, 2]
        print("close view: marker blob of %d px re-detected at x %.4f y %.4f (home view said %.4f %.4f), moved %.1f mm" % (px2, refined[0], refined[1], target[0], target[1], np.hypot(*(refined[:2] - target[:2])) * 1e3), flush=True)
        meta["refined_marker"] = refined.tolist()
        if np.hypot(*(refined[:2] - target[:2])) > 0.008:
            target = np.array([refined[0], refined[1], a.aim_z])
            print("more than 8 mm from the planned marker: re-planning the sweep around the refined position", flush=True)
            q_now = robot.q.copy()
            poses2 = sweep_poses(planner, target, q_now, a, offsets[1:])
            labels2 = ["sweep s %+.0f mm" % (p[0] * 1e3) for p in poses2] + ["return home"]
            plans = [plans[0]] + planner.legs(robot.tool_matrix(), q_now, [p[1] for p in poses2] + [start], labels2)
            meta["replanned"] = True
        for i, plan in enumerate(plans[1:], start=1):
            run_leg(plan, i)
            if i < len(plans) - 1:
                capture(i)
    except (PlanError, RuntimeError) as e:
        print("STOPPING: %s. Going home on the planned abort leg." % e, flush=True)
        if plans and np.max(np.abs(robot.q - abort[0]["seed"])) < np.deg2rad(1):
            run_leg(abort[0], 0)
        (run / "orbit.json").write_text(json.dumps(meta, indent=1))
        os._exit(1)
    meta["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    meta["home_error_deg"] = float(np.max(np.abs(np.degrees(robot.q) - HOME_Q)))
    (run / "orbit.json").write_text(json.dumps(meta, indent=1))
    print("back home (max joint error %.2f deg), fault=%s. Saved to %s" % (meta["home_error_deg"], fault["on"], run), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
