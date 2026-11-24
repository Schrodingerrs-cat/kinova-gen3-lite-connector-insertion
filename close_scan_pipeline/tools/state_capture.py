#!/usr/bin/env python3
"""No motion: print arm joints vs project home and the gripper, save one colour+depth frame. python3 state_capture.py [out.npz]"""
import sys
import pathlib
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from scan_grid import HOME_Q, Robot  # noqa: E402

import rclpy
rclpy.init()
robot = Robot("state_capture", background=True)
time.sleep(2.0)
q = np.degrees(robot.q)
print("q deg", np.round(q, 1), flush=True)
print("max dev from home deg", round(float(np.max(np.abs(((q - HOME_Q) + 180) % 360 - 180))), 2), flush=True)
print("gripper", robot.grip_pos_msr, flush=True)
if len(sys.argv) > 1:
    import json
    import scan_grid
    X = np.array(json.loads((scan_grid.M2 / "config/nearby_camera_poses_sideways.json").read_text())["T_tool_camera_at_teaching"])
    frame = scan_grid.fresh_frame(robot, X)
    np.savez_compressed(sys.argv[1], **frame)
    print("saved", sys.argv[1], {k: np.shape(v) for k, v in frame.items()}, flush=True)
import os
os._exit(0)
