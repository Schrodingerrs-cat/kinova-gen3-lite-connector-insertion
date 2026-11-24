#!/usr/bin/env python3
"""Move the arm to the project home pose (the grasp node's /grasp/home), nothing else. Same supervised move as method 2's move_home_once.py.

  python3 go_home.py --execute        REAL MOTION. Without --execute it only says what it would do.
pick.py calls go_home(robot) between picks.
"""
import argparse
import os
import pathlib
import sys

M2 = pathlib.Path.home() / "kinova/ins/method2_src/Code/Grasp/ros2_iface"
sys.path.insert(0, str(M2))
import numpy as np  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from run_seven_view_scan import call, dry  # noqa: E402


def go_home(robot):
    if robot.ik(robot.tool_matrix(), np.degrees(robot.q)) is None:
        raise RuntimeError("current pose IK failed, no home request sent")
    req = GetParameters.Request()
    req.names = ["dry_run", "pregrasp_only", "allow_grasp"]
    res = call(robot, GetParameters, "/grasp_node/get_parameters", req)
    if [v.bool_value for v in res.values] != [True, True, False]:
        raise RuntimeError("grasp node must be dry_run=True, pregrasp_only=True, allow_grasp=False")
    dry(robot, False)
    try:
        print("Moving to project home.", flush=True)
        result = call(robot, Trigger, "/grasp/home", Trigger.Request(), 120)
        print(result.message, flush=True)
    finally:
        try:
            dry(robot, True)
        except Exception as exc:
            print("dry run restore unconfirmed: %s" % exc, flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()
    if not args.execute:
        print("No motion. --execute sends the arm to the project home pose.")
        raise SystemExit(0)
    import rclpy
    from robot_ros import Robot
    rclpy.init()
    robot = Robot("go_home")
    if not robot.wait_ready(10):
        print("robot not ready")
        os._exit(1)
    go_home(robot)
    os._exit(0)
