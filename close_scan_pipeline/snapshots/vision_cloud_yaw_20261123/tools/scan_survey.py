#!/usr/bin/env python3
"""Colour survey: camera 25 cm above the board, tilted toward the middle of the three plugs from the left, centre, right, then from in front of the plugs with the wrist tilted back toward them, to see the markers and the cable sides.

  python3 scan_survey.py                         plan only, moves nothing
  python3 scan_survey.py --execute --name X      REAL MOTION: home -> N poses along the markers (capture at each) -> home. The gripper does not move.

Marker positions come from the home view by colour (profiles in connectors/*.yaml). Needs the stack up, arm at project home, gripper open, e-stop in reach, ros_env.sh sourced.
"""
import argparse
import json
import os
import pathlib
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import connector  # noqa: E402
import scan_grid  # noqa: E402
import scan_orbit as so  # noqa: E402
import vision  # noqa: E402
from robot_ros import Robot  # noqa: E402

HOME_DIR = pathlib.Path.home()
HOME_Q, M2, G = scan_grid.HOME_Q, scan_grid.M2, scan_grid.G


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=0.25, help="camera height above the board (m)")
    ap.add_argument("--lateral", type=float, default=0.09, help="camera offset left and right of the middle (m)")
    ap.add_argument("--fb", type=float, default=0.07, help="camera offset forward (+x) and back (-x) of the middle (m)")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--name", default="survey")
    a = ap.parse_args()
    import json as js
    X = np.array(js.loads((M2 / "config/nearby_camera_poses_sideways.json").read_text())["T_tool_camera_at_teaching"])
    cfg = yaml.safe_load(open(M2 / "config/grasp.yaml"))
    motion = yaml.safe_load(open(G / "config/grasp.yaml"))["motion"]
    lim = np.array(motion["joint_limits_deg"], float) - motion["joint_margin_deg"]
    rclpy.init()
    robot = Robot("scan_survey", background=True)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    robot.spin_until(lambda: False, 1.0)
    away = float(np.max(np.abs(np.degrees(robot.q) - HOME_Q)))
    if away > 2:
        sys.exit("arm is %.1f deg from project home: go home first" % away)
    if robot.grip_pos_msr is None or robot.grip_pos_msr > 0.05:
        sys.exit("gripper is not fully open")
    tmp = pathlib.Path(tempfile.mkdtemp()) / "home_view.npz"
    np.savez_compressed(tmp, **scan_grid.fresh_frame(robot, X))
    found = {}
    for name in ("power", "aux", "usb"):
        m = vision.find_marker(tmp, connector.load(name)["marker"])
        found[name] = m
        print("%-5s marker at x %.4f y %.4f (%d px), board top around it %.4f" % (name, m["P"][0], m["P"][1], m["px"], m["board_z"]), flush=True)
    ys = [m["P"][1] for m in found.values()]
    x_mid = float(np.median([m["P"][0] for m in found.values()]))
    board_top = float(np.nanmedian([m["board_z"] for m in found.values()]))
    plug_top = max(m["top"] for m in found.values())
    y_mid = float(np.mean(ys))
    aim = np.array([x_mid, y_mid, board_top + 0.02])
    offsets = [(0, a.lateral), (0, 0), (0, -a.lateral), (a.fb, 0), (1.8 * a.fb, 0)]   # forward of the plugs with the wrist tilted back toward them, never from behind
    names = ["left", "centre", "right", "front", "front far"]
    args = SimpleNamespace(min_tcp_z=0.05, min_tcp_dist=0.06, board_top=board_top, plug_top=plug_top, height=a.height)
    planner = so.Planner(robot, X, so.tcp_matrix(cfg), lim, args)
    start, seed = robot.tool_matrix(), robot.q.copy()
    poses, s = [], seed.copy()
    try:
        for (dx, dy), nm in zip(offsets, names):
            c = aim + np.array([dx, dy, a.height - 0.02])
            T_tool, T_cam, q, roll = planner.pose(so.aimed_camera(c, aim), aim, s)
            tilt = np.degrees(np.arctan2(np.hypot(dx, dy), a.height - 0.02))
            print("  %-6s camera xyz %s, tilt %4.1f deg, fingertip %.0f mm above the board, roll %d" % (nm, np.round(T_cam[:3, 3], 3), tilt, ((T_tool @ planner.T_tcp)[2, 3] - board_top) * 1e3, roll), flush=True)
            poses.append(T_tool)
            s = np.radians(q)
        plans = planner.legs(start, seed, poses + [start], names + ["return home"])
    except so.PlanError as e:
        print("PLAN FAILED: %s. Nothing moved." % e, flush=True)
        os._exit(1)
    for p in plans:
        print("  leg %-12s max joint step %.1f deg, lowest fingertip %.3f m, %.1f s" % (p["label"], p["peak"], p["low"], p["times"][-1]), flush=True)
    print("PLAN OK: %d legs, about %.0f s." % (len(plans), sum(p["times"][-1] for p in plans) + 4 * len(names)), flush=True)
    if not a.execute:
        print("Nothing moved.", flush=True)
        os._exit(0)
    run = HOME_DIR / "kinova/work/grasp_runs" / ("run_%s_survey_%s" % (time.strftime("%Y%m%d_%H%M%S"), a.name))
    scan = run / "method2_perception/scan"
    scan.mkdir(parents=True)
    robot.grab()
    for i, plan in enumerate(plans):
        if np.max(np.abs(robot.q - plan["seed"])) > np.deg2rad(1):
            print("start state differs before %s: stopping" % plan["label"], flush=True)
            os._exit(1)
        print(plan["label"], "%.1f s" % plan["times"][-1], flush=True)
        robot.send_joint_path(plan["path"], plan["times"], velocities=np.zeros_like(plan["path"]).tolist(), timeout=plan["times"][-1] + 30)
        if i < len(plans) - 1:
            robot.spin_until(lambda: False, 1.0)
            np.savez_compressed(scan / ("view_%02d.npz" % i), **scan_grid.fresh_frame(robot, X))
            print("view %d saved" % i, flush=True)
    (run / "survey.json").write_text(json.dumps(dict(markers={k: dict(P=v["P"].tolist(), px=v["px"]) for k, v in found.items()}, board_top=board_top, offsets=offsets, aim=aim.tolist()), indent=1))
    print("back home. Saved to %s" % run, flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
