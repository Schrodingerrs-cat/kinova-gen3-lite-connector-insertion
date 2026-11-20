#!/usr/bin/env python3
"""Grasp the plug from a PASSED stage 1 close scan. Plan only unless --execute.

  python3 ~/kinova/work/tools/grasp_plug.py --run RUN_DIR                      plan from saved views, moves nothing
  python3 ~/kinova/work/tools/grasp_plug.py --fresh --execute --name X         REAL MOTION: close scan, stage 1 check, then the grasp if it passed
  python3 ~/kinova/work/tools/grasp_plug.py --run RUN_DIR --execute            REAL MOTION: grasp from these views (arm at home, views must be current)

Grasp: jaws close along the plug's thin axis (normal of the wide faces, yaw from stage 1), approach straight down. Depth is measured: tcp (fingertip) z = board top + 12 mm, jaw band up to +12 mm more,
and the plug top measured from the plug points must leave 8 mm above the jaw band. Pre-grasp 90 mm above the grasp point, descent 0.04 m/s to 20 mm above then 0.015 m/s (photo at both), lift 0.04 m/s, jaws open to 31.5 mm.
After closing the finger reading is only logged (the fingers are underactuated: a thin part pinched at the tips reads fully closed, so it cannot decide). Lift 5 mm, then 80 mm, then the wrist camera
decides (depth between the tips and the table, same rule as the grasp node's hold check). Held: stays holding, no release. Not held: opens and stops. Evidence in RUN/grasp/.
Needs the stack up, arm at project home, gripper open, e-stop in reach, ros_env.sh sourced.
"""
import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import connector  # noqa: E402
import scan_grid  # noqa: E402  (method 2 robot_ros on the path)
import plug_points as pp  # noqa: E402
import stage1_close  # noqa: E402
from example_interfaces.msg import Bool  # noqa: E402
from robot_ros import Robot  # noqa: E402

M2 = scan_grid.M2
sys.path.insert(0, str(M2))
from housing_grasp.config import load_config  # noqa: E402
from arm import Arm  # noqa: E402

HOME_Q = scan_grid.HOME_Q


def pose(x, y, z, yaw_deg, flip=1):
    a = np.radians(yaw_deg)
    yg = flip * np.array([np.cos(a), np.sin(a), 0.0])
    zg = np.array([0, 0, -1.0])
    T = np.eye(4)
    T[:3, :3] = np.c_[np.cross(yg, zg), yg, zg]
    T[:3, 3] = [x, y, z]
    return T


def hold_check(robot, h):
    """Same rule as the grasp node: object in hand if the depth between the tips is short and the table in the lower corners is farther."""
    depth = np.median(np.stack([robot.grab()[1] for _ in range(3)]), 0)

    def med(r0, r1, c0, c1):
        d = depth[r0:r1, c0:c1]
        d = d[(d > 0.05) & (d < 1.5)]
        return float(np.median(d)) if len(d) > 200 else float("nan")
    tips = med(400, 480, 340, 510)
    table = float(np.nanmedian([med(320, 480, 0, 255), med(320, 480, 595, 848)]))
    return bool(tips < h["tip_max_m"] and table - tips > h["margin_m"]), tips, table


def sticker_top(views, res, board, spread=0.006):
    """Barrel top: per view the 90th percentile height of the bright sticker blob nearest the plug centre, median over views."""
    tops = []
    for v in views:
        f = np.load(v)
        d, c, K, T = f["depth"].astype(np.float32), f["color"], f["K"], f["T_BC"]
        ok = (d > 0.08) & (d < 0.8) & (np.abs(d - cv2.medianBlur(d, 5)) < 0.004)
        hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        m = ((hsv[..., 0] >= 75) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 100) & (hsv[..., 2] >= 80)).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        cam = np.linalg.inv(T) @ np.r_[res["x"], res["y"], board + 0.01, 1]
        u0, v0 = K[0, 0] * cam[0] / cam[2] + K[0, 2], K[1, 1] * cam[1] / cam[2] + K[1, 2]
        cand = [i for i in range(1, n) if st[i, 4] >= 40]
        if not cand:
            continue
        i = min(cand, key=lambda j: np.hypot(cen[j][0] - u0, cen[j][1] - v0))
        vv, uu = np.nonzero((lab == i) & ok)
        z = d[vv, uu]
        P = np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
        if len(P) >= 25:
            tops.append(float(np.percentile(P[:, 2], 90)))
    print("barrel top per view above the board (mm): %s" % [round((t - board) * 1e3, 1) for t in tops])
    if len(tops) < 3 or np.ptp(tops) > spread:
        raise SystemExit("barrel top not consistent between views: no grasp")
    return float(np.median(tops)), len(tops)


def measure(run, prof):
    gr = prof["grasp"]
    views = sorted((run / "method2_perception/scan").glob("view_*.npz"))
    pr = pp.prior(views, prof["part"])
    data = pp.points(views, pr, prof["points"]["radius_m"], top_tol=prof["points"].get("top_tol_m", 0.006), open_mm=prof["points"].get("open_mm"), mode=prof["points"].get("mode", "teal_or_dark"))
    res = json.loads((run / "stage1_close.json").read_text())
    if prof["grasp"]["top_rule"] == "sticker_p90":
        top, n = sticker_top(views, res, pr["board_top"], gr.get("top_spread_m", 0.006))
        return pr["board_top"], top, n, res
    core = np.vstack([d["pts"] for d in data])
    core = core[np.hypot(core[:, 0] - res["x"], core[:, 1] - res["y"]) <= 0.006]
    return pr["board_top"], float(np.percentile(core[:, 2], 90)), len(core), res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=pathlib.Path)
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--name", default="grasp")
    ap.add_argument("--connector", default="usb", help="profile in ~/kinova/work/connectors")
    ap.add_argument("--hold-then-open", type=float, default=0.0, help="after the camera says HELD, wait this many s (operator looks), then just open the jaws where the arm is. 0 = keep holding")
    a = ap.parse_args()
    prof = connector.load(a.connector)
    gr = prof["grasp"]
    CLEAR, BAND, TOP_MARGIN, STANDOFF, LIFT_TEST, LIFT = gr["clear_m"], gr["band_m"], gr["top_margin_m"], gr["standoff_m"], gr["lift_test_m"], gr["lift_m"]
    if a.fresh and not a.execute:
        sys.exit("--fresh moves the arm: add --execute")
    if a.fresh:
        cmd = [sys.executable, "-u", str(HERE / "scan_orbit.py"), "--connector", a.connector, "--execute", "--name", a.name]
        out = subprocess.run(cmd, capture_output=True, text=True).stdout
        print(out)
        m = re.search(r"Saved to (\S+)", out)
        if not m:
            sys.exit("scan did not finish, no grasp")
        a.run = pathlib.Path(m.group(1))
    if not a.run:
        sys.exit("give --run RUN_DIR or --fresh")
    run = a.run.resolve()
    res = stage1_close.evaluate(run, None, a.connector)
    (run / "stage1_close.json").write_text(json.dumps(res, indent=1))
    for g in res["gates"]:
        print("  [%s] %s: %s" % ("ok " if g["ok"] else "FAIL", g["name"], g["detail"]))
    if not res["passed"]:
        sys.exit("stage 1 did not pass: no grasp")
    board, top, ncore, res = measure(run, prof)
    exposed = top - board
    z_tcp = board + CLEAR
    print("depth: board top %.4f m, plug top %.4f m (rule %s, %d samples) -> stands %.0f mm above the board" % (board, top, gr["top_rule"], ncore, exposed * 1e3))
    print("       fingertip z %.4f (%.0f mm above the board), jaw band to %.4f, plug top %.0f mm above the band" % (z_tcp, CLEAR * 1e3, z_tcp + BAND, (top - z_tcp - BAND) * 1e3))
    if gr.get("min_exposed_m") and exposed < gr["min_exposed_m"]:
        sys.exit("measured stand-off %.0f mm is below the plausible minimum %.0f mm: no grasp" % (exposed * 1e3, gr["min_exposed_m"] * 1e3))
    if ncore < 3 or top - (z_tcp + BAND) < TOP_MARGIN:
        sys.exit("plug top not measured well enough or too low for this jaw band: no grasp")
    def chain(flip):
        T = pose(res["x"], res["y"], z_tcp, res["yaw_deg"], flip)
        zg = T[:3, 2]
        pre, l5, lift = T.copy(), T.copy(), T.copy()
        pre[:3, 3] -= zg * STANDOFF
        l5[:3, 3] -= zg * LIFT_TEST
        lift[:3, 3] -= zg * LIFT
        return T, pre, l5, lift

    cfg = load_config(M2 / "config/grasp.yaml")
    gp, mo = cfg["gripper"], cfg["motion"]
    tab = np.array(gp["position_to_width_m"])
    width = gr["open_width_m"]
    open_pos = float(np.interp(width, tab[::-1, 1], tab[::-1, 0]))
    to_tool = np.linalg.inv(scan_grid_tcp(gp))
    rclpy.init()
    robot = Robot("grasp_plug", background=True)
    fault = {"on": False}
    robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("on", bool(m.data)), 10)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    robot.spin_until(lambda: False, 1.0)
    arm = Arm(robot, mo, scan_grid_tcp(gp), cfg["scan"]["table_z_m"] - 0.005, gp["close_cmd"])
    if np.max(np.abs(np.degrees(robot.q) - HOME_Q)) > 2 or fault["on"] or robot.grip_pos_msr is None or robot.grip_pos_msr > 0.05:
        sys.exit("arm must be at project home, no fault, gripper open")
    best = None
    for flip in (1, -1):
        T_BG, T_pre, T_lift5, T_lift = chain(flip)
        plan, why = arm.plan_chain(T_pre @ to_tool, T_BG @ to_tool, T_lift @ to_tool, cfg["grasp"]["max_joint_dev_deg"])
        print("jaw axis %s: %s" % ("+y side" if flip > 0 else "-y side", why if plan is None else "ok, largest joint change from home %.0f deg" % np.abs((plan["q_pre"] - np.degrees(robot.q) + 180) % 360 - 180).max()))
        if plan is not None:
            dev = np.abs((plan["q_pre"] - np.degrees(robot.q) + 180) % 360 - 180).max()
            if best is None or dev < best[0]:
                best = (dev, plan, (T_BG, T_pre, T_lift5, T_lift))
    if best is None:
        sys.exit("motion plan failed for both jaw directions")
    _, plan, (T_BG, T_pre, T_lift5, T_lift) = best
    print("grasp: tcp x %.4f y %.4f z %.4f, jaw axis yaw %.1f deg, approach straight down, pre-grasp %.0f mm above" % (*T_BG[:3, 3], np.degrees(np.arctan2(T_BG[1, 1], T_BG[0, 1])) % 360, STANDOFF * 1e3))
    lim = np.array(yaml_limits()) - 3
    allq = np.vstack([np.radians(plan["q_pre"])[None], np.array(plan["down"][0]), np.array(plan["lift"][0])])
    if np.any(np.abs((np.degrees(allq) + 180) % 360 - 180) > lim):
        sys.exit("a waypoint is outside the joint limits minus margin")
    print("plan ok: pre-grasp joints %s deg, descent %.1f s, lift %.1f s, jaws open to %.1f mm (gripper %.3f)" % (np.round(plan["q_pre"], 1), plan["down"][1], plan["lift"][1], width * 1e3, open_pos))
    if not a.execute:
        print("Nothing moved. Add --execute to grasp.")
        os._exit(0)

    out = run / "grasp"
    out.mkdir(exist_ok=True)
    log = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"), connector=a.connector, standoff_m=STANDOFF, board_top=board, plug_top=top, T_BG=T_BG.tolist(), open_width_m=width, stages=[])

    def snap(name):
        color, depth, _ = robot.grab()
        cv2.imwrite(str(out / (name + ".png")), color)

    def stage(name, **kw):
        log["stages"].append(dict(name=name, t=round(time.time(), 1), gripper=float(robot.grip_pos_msr), **kw))
        print(name, kw, flush=True)

    def need_ok(name, goal=None):
        if fault["on"]:
            raise RuntimeError("fault at " + name)
        if goal is not None:
            m = robot.tool_matrix()
            if np.linalg.norm(m[:3, 3] - goal[:3, 3]) > 0.003 or Rotation.from_matrix(goal[:3, :3].T @ m[:3, :3]).magnitude() > np.deg2rad(1.5):
                raise RuntimeError("not at the %s pose" % name)

    if not prof.get("scene_include_plugs"):
        subprocess.run([sys.executable, str(HERE / "add_fixture_to_scene.py"), "--remove", "--include-plugs"], capture_output=True)
        subprocess.run([sys.executable, str(HERE / "add_fixture_to_scene.py")], capture_output=True)
    else:
        subprocess.run([sys.executable, str(HERE / "add_fixture_to_scene.py"), "--include-plugs"], capture_output=True)
    print("REAL MOTION. Grasp. E-stop in reach.", flush=True)
    try:
        arm.go_joints(plan["q_pre"])
        need_ok("pre-grasp", T_pre @ to_tool)
        stage("at pre-grasp")
        snap("1_pregrasp")
        arm.grip_to(open_pos, gp["close_effort"], rate=0.3)
        stage("jaws open", position=float(robot.grip_pos_msr))
        T_mid = T_BG.copy()
        T_mid[:3, 3] -= T_BG[:3, 2] * 0.02
        mid = robot.line_path(T_mid @ to_tool, np.radians(plan["q_pre"]))
        robot.send_line(mid, max(2.0, (STANDOFF - 0.02) / 0.04))
        need_ok("20 mm above the grasp", T_mid @ to_tool)
        robot.spin_until(lambda: False, 1.0)
        snap("1b_20mm_above")
        stage("20 mm above the grasp point", tcp=np.round((robot.tool_matrix() @ scan_grid_tcp(gp))[:3, 3], 4).tolist())
        fin = robot.line_path(T_BG @ to_tool, mid[-1])
        robot.send_line(fin, max(2.5, 0.02 / 0.015))
        need_ok("grasp", T_BG @ to_tool)
        robot.spin_until(lambda: False, 1.0)
        snap("2_at_grasp")
        stage("at grasp pose", tcp=np.round((robot.tool_matrix() @ scan_grid_tcp(gp))[:3, 3], 4).tolist())
        pos, contact = arm.grip_to(gp["close_cmd"], gp["close_effort"], rate=0.2)
        gap = float(np.interp(pos, tab[:, 0], tab[:, 1]))
        stage("closed", position=float(pos), contact=bool(contact), gap_mm=round(gap * 1e3, 1), note="finger reading does not decide")
        snap("3_closed")
        pts = robot.line_path(T_lift5 @ to_tool, robot.q)
        robot.send_line(pts, 2.0 + LIFT_TEST / 0.04)
        robot.spin_until(lambda: False, 1.0)
        stage("5 mm lift", finger_change=round(abs(float(robot.grip_pos_msr) - float(pos)), 4))
        pts = robot.line_path(T_lift @ to_tool, robot.q)
        robot.send_line(pts, 2.0 + (LIFT - LIFT_TEST) / 0.04)
        robot.spin_until(lambda: False, 2.0)
        snap("4_lifted")
        held, tips, table = hold_check(robot, cfg["hold_check"])
        stage("lifted 80 mm, camera check", held=held, depth_between_tips=round(tips, 3), table_depth=round(table, 3), position=float(robot.grip_pos_msr))
        if not held:
            arm.grip_to(open_pos, gp["close_effort"], rate=0.3)
            stage("camera: nothing between the tips, jaws opened")
        elif a.hold_then_open > 0:
            robot.spin_until(lambda: False, a.hold_then_open)
            arm.grip_to(0.0, gp["close_effort"], rate=0.3)
            stage("held for %.0f s, then jaws opened where the arm is (no put-back)" % a.hold_then_open)
    except Exception as e:
        stage("STOPPED", reason=str(e))
        print("STOPPED: %s. Not moving further." % e, flush=True)
    log["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    log["note"] = "Physical hold is NOT verified by software: judge from the images and the operator."
    (out / "grasp.json").write_text(json.dumps(log, indent=1))
    print("saved %s" % (out / "grasp.json"), flush=True)
    try:
        import report_run
        report_run.main(run)
    except Exception as e:
        print("report failed: %s" % e, flush=True)
    os._exit(0)


def scan_grid_tcp(gp):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(gp["tcp_in_tool_quat_xyzw"]).as_matrix()
    T[:3, 3] = gp["tcp_in_tool_xyz_m"]
    return T


def yaml_limits():
    import yaml
    return yaml.safe_load(open(scan_grid.G / "config/grasp.yaml"))["motion"]["joint_limits_deg"]


if __name__ == "__main__":
    main()
