#!/usr/bin/env python3
"""Pick plugs one after another, each in one process: find the plug by its marker colour, close scan, stage 1 check, grasp, report. Name them in the order you want; repeats are fine.

  python3 pick.py aux                          plan only for one plug: finds the marker, checks the scan plan, moves nothing
  python3 pick.py usb aux power --execute      REAL MOTION: USB, then aux, then power (arm goes home between picks)
  python3 pick.py usb aux usb --execute        USB, aux, USB again
  options: --name X (run folder label)   --hold-then-open S (seconds to hold, then open where the arm is; 0 = keep holding)   --scan-only (scan and stage 1 only, no grasp)

A pick that fails (marker not found, stage 1 or grasp refused, scan stopped) is logged and the series goes on with the next plug. Plan only runs the same finding and checks for every name without moving.
Needs the stack up, arm at project home (go_home.py --execute), gripper open, e-stop in reach, ros_env.sh sourced. Plug settings are in ~/kinova/work/connectors/<name>.yaml.
"""
import argparse
import os
import pathlib
import sys
import time

import numpy as np
import rclpy
from example_interfaces.msg import Bool

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import connector  # noqa: E402
import go_home  # noqa: E402
import grasp_plug  # noqa: E402
import scan_orbit  # noqa: E402
import stage1_close  # noqa: E402
from robot_ros import Robot  # noqa: E402

HOME_Q = scan_orbit.HOME_Q


def one_pick(robot, fault, plug, a, label):
    t0 = time.time()
    scan_args = ["--connector", plug, "--name", label] + (["--execute"] if a.execute else [])
    run = scan_orbit.main(scan_args, robot, fault)
    if run is None:
        return "planned"
    if a.scan_only:
        res = stage1_close.evaluate(run, None, plug)
        return "stage 1 PASS" if res["passed"] else "stage 1 FAIL"
    log = grasp_plug.main(["--run", str(run), "--connector", plug, "--execute", "--hold-then-open", str(a.hold_then_open)], robot, fault)
    held = any(s.get("held") for s in (log or {}).get("stages", []))
    return "HELD" if held else "grasp done, not held"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plugs", nargs="+", choices=["usb", "aux", "power"], help="plugs in the order to pick them, repeats allowed")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--name", default="pick")
    ap.add_argument("--hold-then-open", type=float, default=5.0)
    ap.add_argument("--scan-only", action="store_true")
    a = ap.parse_args()
    for p in a.plugs:
        connector.load(p)
    rclpy.init()
    robot = Robot("pick", background=True)
    fault = {"on": False}
    robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("on", bool(m.data)), 10)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    robot.spin_until(lambda: False, 1.0)
    summary = []
    for i, plug in enumerate(a.plugs, start=1):
        print("\n===== pick %d of %d: %s =====" % (i, len(a.plugs), plug), flush=True)
        t0 = time.time()
        try:
            result = one_pick(robot, fault, plug, a, "%s_%d_%s" % (a.name, i, plug))
        except SystemExit as e:
            result = "FAILED: %s" % (e.code if isinstance(e.code, str) else "exit %s" % e.code)
        except Exception as e:
            result = "FAILED: %s" % e
        print("pick %d (%s): %s" % (i, plug, result), flush=True)
        summary.append((i, plug, result, time.time() - t0))
        if a.execute and i < len(a.plugs):
            robot.spin_until(lambda: False, 0.5)
            if np.max(np.abs(((np.degrees(robot.q) - HOME_Q) + 180) % 360 - 180)) > 2:
                if fault["on"]:
                    print("arm fault: series stopped", flush=True)
                    break
                go_home.go_home(robot)
                robot.spin_until(lambda: False, 1.0)
    if a.execute:
        robot.spin_until(lambda: False, 0.5)
        if np.max(np.abs(((np.degrees(robot.q) - HOME_Q) + 180) % 360 - 180)) > 2 and not fault["on"]:
            go_home.go_home(robot)
    print("\nsummary:")
    for i, plug, result, sec in summary:
        print("  %d. %-5s %-45s %3.0f s" % (i, plug, result, sec))


if __name__ == "__main__":
    code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, flush=True)
            code = 1
        else:
            code = e.code or 0
    sys.stdout.flush()
    os._exit(code)
