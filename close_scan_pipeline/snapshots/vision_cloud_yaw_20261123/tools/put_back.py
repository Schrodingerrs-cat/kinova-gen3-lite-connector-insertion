#!/usr/bin/env python3
"""Put a held plug back: reverse of grasp_plug.py. Straight down to the grasp pose of that grasp, open the jaws, lift clear, jaws fully open, home.

  python3 ~/kinova/work/tools/put_back.py RUN/grasp/grasp.json            checks the arm is where the lift ended, moves nothing
  python3 ~/kinova/work/tools/put_back.py RUN/grasp/grasp.json --execute  REAL MOTION
"""
import argparse
import json
import os
import pathlib
import subprocess
import sys
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import grasp_plug as gp_  # noqa: E402
import scan_grid  # noqa: E402
from example_interfaces.msg import Bool  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402
from robot_ros import Robot  # noqa: E402
from arm import Arm  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("grasp_json", type=pathlib.Path)
ap.add_argument("--execute", action="store_true")
a = ap.parse_args()
log = json.loads(a.grasp_json.read_text())
T_BG = np.array(log["T_BG"])
cfg = load_config(scan_grid.M2 / "config/grasp.yaml")
gp = cfg["gripper"]
Ttcp = gp_.scan_grid_tcp(gp)
to_tool = np.linalg.inv(Ttcp)
tab = np.array(gp["position_to_width_m"])
open_pos = float(np.interp(log["open_width_m"], tab[::-1, 1], tab[::-1, 0]))
zg = T_BG[:3, 2]
T_mid, T_pre = T_BG.copy(), T_BG.copy()
T_mid[:3, 3] -= zg * 0.02
T_pre[:3, 3] -= zg * 0.08

rclpy.init()
robot = Robot("put_back", background=True)
fault = {"on": False}
robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("on", bool(m.data)), 10)
robot.wait_ready(10)
robot.spin_until(lambda: False, 1.0)
arm = Arm(robot, cfg["motion"], Ttcp, cfg["scan"]["table_z_m"] - 0.005, gp["close_cmd"])
now = robot.tool_matrix()
err = np.linalg.norm(now[:3, 3] - (T_pre @ to_tool)[:3, 3])
print("arm is %.1f mm from the end of that lift; gripper %.3f; fault %s" % (err * 1e3, robot.grip_pos_msr, fault["on"]))
if err > 0.01 or fault["on"]:
    sys.exit("arm is not where the lift ended: not moving")
if not a.execute:
    print("Nothing moved. Add --execute.")
    os._exit(0)


def leg(T, seed, dur, label):
    pts = robot.line_path(T @ to_tool, seed)
    if pts is None or np.abs(np.diff(np.array(pts), axis=0)).max() > 0.1:
        raise RuntimeError(label + ": no safe straight line")
    robot.send_line(pts, dur)
    m = robot.tool_matrix()
    if fault["on"] or np.linalg.norm(m[:3, 3] - (T @ to_tool)[:3, 3]) > 0.003:
        raise RuntimeError(label + ": did not arrive")
    return np.array(pts[-1])


try:
    q = leg(T_mid, robot.q, 2.5, "down to 20 mm above")
    q = leg(T_BG, q, 2.5, "down to the grasp pose")
    robot.spin_until(lambda: False, 1.0)
    arm.grip_to(open_pos, gp["close_effort"], rate=0.3)
    print("jaws opened to %.1f mm, plug released at its seat" % (log["open_width_m"] * 1e3), flush=True)
    q = leg(T_pre, q, 2.5, "lift clear")
    arm.grip_to(0.0, gp["close_effort"], rate=0.3)
    print("jaws fully open (%.3f), lifted clear" % robot.grip_pos_msr, flush=True)
except Exception as e:
    print("STOPPED: %s. Not moving further." % e, flush=True)
    os._exit(1)
subprocess.run([sys.executable, str(HERE / "add_fixture_to_scene.py"), "--include-plugs"], capture_output=True)
os._exit(0)
